"""Static, pre-flight validation of a mission plan.

The second safety barrier. Runs on every plan, whether it came from the LLM or
the rule parser, before anything moves. It *rejects* rather than silently
"fixing" anything that changes what the operator asked for (an altitude above
the ceiling is an error, not a clamp). The only normalisation is structural and
always safe: prepend a takeoff and append a return-home if missing.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from kestrel.config import Config, WorldConfig
from kestrel.geofence import Geofence
from kestrel.guidance.trajectory import Path
from kestrel.mission.geometry import DEFAULT_ALTITUDE, GeometryError, task_geometry
from kestrel.mission.schema import (
    HoldTask,
    LandTask,
    Mission,
    OrbitTask,
    ReturnHomeTask,
    SearchTask,
    SurveyTask,
    TakeoffTask,
    TrackTask,
)
from kestrel.safety.energy import EnergyModel


@dataclass
class ValidationReport:
    mission: Mission
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    est_duration_s: float = 0.0
    est_energy_wh: float = 0.0
    usable_energy_wh: float = 0.0

    @property
    def ok(self) -> bool:
        return not self.errors

    def summary(self) -> str:
        lines = [f"plan: {self.mission.describe()}"]
        lines.append(
            f"estimate: {self.est_duration_s / 60:.1f} min, {self.est_energy_wh:.1f} Wh "
            f"of {self.usable_energy_wh:.1f} Wh usable"
        )
        lines += [f"warning: {w}" for w in self.warnings]
        lines += [f"ERROR: {e}" for e in self.errors]
        lines.append("result: " + ("ACCEPTED" if self.ok else "REJECTED"))
        return "\n".join(lines)


def _normalise(m: Mission, notes: list[str]) -> Mission:
    tasks = list(m.tasks)
    if not isinstance(tasks[0], TakeoffTask):
        first_alt = next((getattr(t, "altitude", None) for t in tasks if getattr(t, "altitude", None)), None)
        tasks.insert(0, TakeoffTask(altitude=first_alt or DEFAULT_ALTITUDE))
        notes.append("added takeoff at the start")
    if not isinstance(tasks[-1], LandTask | ReturnHomeTask):
        tasks.append(ReturnHomeTask())
        notes.append("added return_home at the end")
    return m.model_copy(update={"tasks": tasks})


def validate(mission: Mission, cfg: Config, world: WorldConfig) -> ValidationReport:
    lim = cfg.safety
    warnings: list[str] = []
    mission = _normalise(mission, warnings)
    rep = ValidationReport(mission=mission, warnings=warnings)
    fence = Geofence(world.fence)
    energy = EnergyModel(cfg.vehicle, lim)
    rep.usable_energy_wh = cfg.vehicle.battery_wh * (1 - lim.energy_reserve_fraction)

    takeoffs = [i for i, t in enumerate(mission.tasks) if isinstance(t, TakeoffTask)]
    if takeoffs != [0]:
        rep.errors.append("takeoff is only allowed as the first task")
    for i, t in enumerate(mission.tasks[:-1]):
        if isinstance(t, LandTask | ReturnHomeTask):
            rep.errors.append(f"task {i + 1} ({t.type}) ends the flight but more tasks follow")

    standoffs = [t.standoff_m for t in mission.tasks if isinstance(t, TrackTask)]
    if mission.on_contact == "track":
        standoffs.append(mission.contact_standoff_m)
    for s in standoffs:
        if s < lim.min_separation + 3.0:
            rep.errors.append(
                f"tracking standoff {s:.0f} m is below the {lim.min_separation + 3.0:.0f} m minimum "
                f"(separation {lim.min_separation:.0f} m + 3 m margin)"
            )

    pos = np.array(world.home, dtype=float)
    alt = DEFAULT_ALTITUDE
    duration = 0.0
    wh = 0.0
    for i, task in enumerate(mission.tasks):
        label = f"task {i + 1} ({task.type})"
        try:
            g = task_geometry(task, pos, alt, mission, world, lim)
        except GeometryError as exc:
            rep.errors.append(f"{label}: {exc}")
            continue
        if not isinstance(task, LandTask):
            if not (lim.min_altitude <= g.altitude <= lim.max_altitude):
                rep.errors.append(
                    f"{label}: altitude {g.altitude:.0f} m outside "
                    f"[{lim.min_altitude:.0f}, {lim.max_altitude:.0f}] m"
                )
            alt = g.altitude
        speed = getattr(task, "speed", None)
        if speed is not None and speed > lim.max_horizontal_speed:
            rep.errors.append(f"{label}: speed above {lim.max_horizontal_speed} m/s limit")
        if isinstance(task, OrbitTask) and task.radius < 5.0:
            rep.errors.append(f"{label}: orbit radius must be at least 5 m")
        if isinstance(task, SurveyTask) and task.spacing < 5.0:
            rep.errors.append(f"{label}: survey spacing must be at least 5 m")
        if isinstance(task, HoldTask | SearchTask | TrackTask) and g.dwell_s > 1800:
            rep.errors.append(f"{label}: duration over 30 min")
        if len(g.points):
            full = np.vstack([pos[None, :], g.points])
            outside = [p for p in full if fence.signed_distance(p) < lim.fence_margin]
            if outside:
                worst = min(fence.signed_distance(p) for p in outside)
                rep.errors.append(
                    f"{label}: path comes within {worst:.0f} m of the geofence "
                    f"(margin {lim.fence_margin:.0f} m)"
                    if worst >= 0
                    else f"{label}: path leaves the geofence by {-worst:.0f} m"
                )
            length = Path(full).length
            wh += energy.flight_wh(length, g.speed)
            duration += length / max(g.speed, 0.1) + 4.0
            pos = g.points[-1].copy()
        if g.dwell_s > 0:
            wh += energy.hover_wh(g.dwell_s)
            duration += g.dwell_s
    if mission.on_contact == "track":
        # budget for one contact interruption
        wh += energy.hover_wh(mission.contact_track_s) * 1.2
        duration += mission.contact_track_s
    landing = max(pos[2], 0.0) / lim.land_speed
    wh += energy.hover_wh(landing)
    duration += landing
    rep.est_duration_s = duration
    rep.est_energy_wh = wh
    if wh > rep.usable_energy_wh:
        rep.errors.append(
            f"estimated energy {wh:.0f} Wh exceeds the {rep.usable_energy_wh:.0f} Wh usable "
            f"({lim.energy_reserve_fraction:.0%} reserve held back)"
        )
    return rep
