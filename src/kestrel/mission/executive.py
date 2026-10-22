"""Mission executive: turns a validated :class:`Mission` into setpoints.

A small behaviour machine. Each task becomes a :class:`Behaviour` when it is
reached (so it is planned from where the aircraft actually is, not where the
planner guessed it would be). Two things can interrupt the queue:

* **Contacts.** With ``on_contact="track"``, the first confirmed track of a new
  aircraft pre-empts the current behaviour with a standoff track. The
  interrupted behaviour is kept on a stack and *resumed* afterwards from the
  point it had reached, so a survey is not silently skipped.
* **Aborts.** Operator emergencies and latched safety requests replace the whole
  queue with hold, return-home or land. Aborts are one-way.

**Contact watch.** The only sensor is one forward camera, and an aircraft that is
not in view cannot be avoided. So once an intruder has been seen, whenever the
observer is within ``watch_radius_m`` of its last known position it points the
camera (heading and gimbal) at it, flying sideways if the path requires. That
keeps the aircraft that matters in front of the sensor, and so in front of the
safety monitor.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from kestrel.config import Config, WorldConfig
from kestrel.geometry import Vec, wrap_pi
from kestrel.guidance.observer import StandoffParams, standoff_setpoint
from kestrel.guidance.trajectory import Path, PathFollower, ReferenceFilter
from kestrel.mission.geometry import DEFAULT_ALTITUDE, TaskGeometry, task_geometry
from kestrel.mission.parser import Emergency
from kestrel.mission.schema import (
    GotoTask,
    HoldTask,
    LandTask,
    Mission,
    OrbitTask,
    ReturnHomeTask,
    SearchTask,
    SurveyTask,
    TakeoffTask,
    Task,
    TrackTask,
)
from kestrel.perception.tracker import TrackEstimate
from kestrel.safety.monitor import SafetyRequest
from kestrel.types import Setpoint, VehicleState
from kestrel.vehicle.base import Vehicle

SWEEP_DEG = 40.0  # camera sweep either side of the direction of travel
YIELD_HORIZON_S = 6.0  # look this far ahead for a conflict with a known contact
YIELD_MARGIN_M = 6.0  # yield if the predicted closest approach is under min separation + this
YIELD_CLEAR_S = 2.0  # conflict must stay clear this long before the path resumes
YIELD_MAX_S = 60.0  # give up on the task (skip it) if it stays blocked this long


@dataclass
class Context:
    t: float
    state: VehicleState
    vehicle: Vehicle
    tracks: list[TrackEstimate]


@dataclass
class Event:
    t: float
    kind: str
    detail: str


class Behaviour(ABC):
    name = "behaviour"
    low_altitude_ok = False  # lets the safety floor relax during takeoff/landing

    @abstractmethod
    def start(self, ctx: Context) -> None: ...

    @abstractmethod
    def step(self, ctx: Context) -> Setpoint | None: ...

    @abstractmethod
    def done(self, ctx: Context) -> bool: ...

    def resume(self, ctx: Context) -> None:
        self.start(ctx)

    def gimbal_tilt(self, ctx: Context) -> float | None:
        """Commanded camera elevation in degrees, or ``None`` for the default."""
        return None


# ---------------------------------------------------------------------------- behaviours


class TakeoffB(Behaviour):
    name = "takeoff"
    low_altitude_ok = True

    def __init__(self, altitude: float) -> None:
        self.altitude = altitude
        self.commanded = False

    def start(self, ctx: Context) -> None:
        self.commanded = False

    def step(self, ctx: Context) -> Setpoint | None:
        if not ctx.state.armed:
            ctx.vehicle.arm()
            return None
        if not self.commanded:
            ctx.vehicle.takeoff(self.altitude)
            self.commanded = True
        return None

    def done(self, ctx: Context) -> bool:
        return ctx.state.armed and ctx.state.pos[2] > self.altitude - 0.8


class PathB(Behaviour):
    """Follow a path; yaw either along track, towards a point, or scanning."""

    def __init__(
        self,
        name: str,
        points: Vec,
        speed: float,
        a_max: float,
        yaw_mode: str = "along",
        face: Vec | None = None,
        min_time_s: float = 0.0,
        repeat: bool = False,
        sweep_deg: float = 0.0,
        sweep_period_s: float = 12.0,
    ) -> None:
        self.name = name
        self.sweep = math.radians(sweep_deg)
        self.sweep_period = sweep_period_s
        self.points = np.asarray(points, dtype=float)
        self.speed = speed
        self.a_max = a_max
        self.yaw_mode = yaw_mode
        self.face = face
        self.min_time_s = min_time_s
        self.repeat = repeat
        self.follower: PathFollower | None = None
        self.t0 = 0.0
        self.started_at: float | None = None
        self.yaw = 0.0
        self.progress_idx = 0  # index into self.points already passed

    def _build(self, ctx: Context, from_idx: int) -> None:
        pts = np.vstack([ctx.state.pos[None, :], self.points[from_idx:]])
        path = Path(pts)
        # start at the speed we already have along the path, so resuming does not stall
        v0 = max(0.0, float(ctx.state.vel @ path.at(0.0)[1]))
        self.follower = PathFollower(path, self.speed, self.a_max, v0=v0)
        self.t0 = ctx.t
        self._offset = from_idx

    def start(self, ctx: Context) -> None:
        if self.started_at is None:
            self.started_at = ctx.t
        self.yaw = ctx.state.yaw
        self._build(ctx, 0)

    def resume(self, ctx: Context) -> None:
        self.yaw = ctx.state.yaw
        self._build(ctx, self.progress_idx)

    def step(self, ctx: Context) -> Setpoint | None:
        assert self.follower is not None
        if self.repeat and self.follower.done(ctx.t - self.t0):
            self._build(ctx, 0)
        sp, _ = self.follower.sample(ctx.t - self.t0, self.yaw)
        # remember how far along the original waypoint list we are, for resume()
        d = np.linalg.norm(self.points[self.progress_idx :] - sp.pos, axis=1)
        if len(d):
            k = int(np.argmin(d))
            if d[k] < 3.0:
                self.progress_idx += k
        if self.yaw_mode == "along" and float(np.linalg.norm(sp.vel[0:2])) > 0.8:
            heading = math.atan2(sp.vel[1], sp.vel[0])
            # Sweep the camera across the direction of travel: one forward camera then
            # covers about hfov + 2*sweep instead of hfov, so crossing traffic is seen.
            phase = 2 * math.pi * (ctx.t - (self.started_at or 0.0)) / self.sweep_period
            self.yaw = wrap_pi(heading + self.sweep * math.sin(phase))
        elif self.yaw_mode == "face" and self.face is not None:
            self.yaw = math.atan2(self.face[1] - sp.pos[1], self.face[0] - sp.pos[0])
        elif self.yaw_mode == "scan":
            self.yaw = wrap_pi(self.yaw + math.radians(20.0) / 20.0)
            return sp.with_(yaw=self.yaw, yaw_rate=math.radians(20.0))
        return sp.with_(yaw=self.yaw)

    def done(self, ctx: Context) -> bool:
        assert self.follower is not None and self.started_at is not None
        if ctx.t - self.started_at < self.min_time_s:
            return False
        if self.repeat:
            return True
        end = self.follower.path.points[-1]
        return self.follower.done(ctx.t - self.t0) and float(np.linalg.norm(ctx.state.pos - end)) < 2.0


class HoldB(Behaviour):
    def __init__(self, duration_s: float, scan: bool = False, name: str = "hold") -> None:
        self.name = name
        self.duration = duration_s
        self.scan = scan
        self.t0 = 0.0
        self.pos = np.zeros(3)
        self.yaw = 0.0

    def start(self, ctx: Context) -> None:
        self.t0 = ctx.t
        self.pos = ctx.state.pos.copy()
        self.yaw = ctx.state.yaw

    def resume(self, ctx: Context) -> None:
        self.pos = ctx.state.pos.copy()  # keep the clock, hold where we are now

    def step(self, ctx: Context) -> Setpoint | None:
        if self.scan:
            self.yaw = wrap_pi(self.yaw + math.radians(25.0) / 20.0)
            return Setpoint(pos=self.pos, yaw=self.yaw, yaw_rate=math.radians(25.0))
        return Setpoint(pos=self.pos, yaw=self.yaw)

    def done(self, ctx: Context) -> bool:
        return ctx.t - self.t0 >= self.duration


class TrackB(Behaviour):
    """Standoff observation of the best confirmed track; scan when there is none."""

    name = "track"

    def __init__(
        self,
        params: StandoffParams,
        duration_s: float,
        a_max: float,
        a_max_close: float | None = None,
        lost_timeout_s: float = 20.0,
    ) -> None:
        self.p = params
        self.a_max = a_max
        self.a_max_close = a_max_close if a_max_close is not None else a_max
        self.close = False
        self.ref: ReferenceFilter | None = None
        self.last_t = 0.0
        self.last_pos: Vec | None = None
        self.duration = duration_s
        self.lost_timeout = lost_timeout_s
        self.t0 = 0.0
        self.last_seen = 0.0
        self.target_id: int | None = None
        self.hold = HoldB(1e9, scan=True)
        self.direction = 1.0

    def start(self, ctx: Context) -> None:
        self.t0 = ctx.t
        self.last_seen = ctx.t
        self.hold.start(ctx)
        self._reset_ref(ctx)

    def resume(self, ctx: Context) -> None:
        self.hold.resume(ctx)
        self._reset_ref(ctx)

    def _reset_ref(self, ctx: Context) -> None:
        self.ref = ReferenceFilter(
            ctx.state.pos, ctx.state.vel, omega=0.8, v_max=self.p.max_speed, a_max=self.a_max, vz_max=2.0
        )
        self.last_t = ctx.t

    def _pick(self, tracks: list[TrackEstimate]) -> TrackEstimate | None:
        if not tracks:
            return None
        for tr in tracks:
            if tr.ident == self.target_id:
                return tr
        return max(tracks, key=lambda tr: tr.hits)

    def step(self, ctx: Context) -> Setpoint | None:
        tr = self._pick(ctx.tracks)
        if tr is None or ctx.t - tr.last_hit_t > 1.0:
            if tr is None:
                self.target_id = None
            if self.last_pos is not None:
                # Lost a contact that was close: back away from where it was, then scan.
                # Hovering in place lets it pass overhead, outside the gimbal's range.
                away = ctx.state.pos - self.last_pos
                away[2] = 0.0
                n = float(np.linalg.norm(away))
                if n < 0.8 * self.p.standoff_m:
                    u = away / n if n > 1e-3 else np.array([1.0, 0.0, 0.0])
                    self.hold.pos = ctx.state.pos + u * (0.8 * self.p.standoff_m - n + 5.0)
                self.last_pos = None
            self._reset_ref(ctx)
            return self.hold.step(ctx)
        if tr.ident != self.target_id:
            self.target_id = tr.ident
        self.last_seen = ctx.t
        self.last_pos = tr.pos.copy()
        goal = standoff_setpoint(ctx.state.pos, tr.pos, tr.vel, self.p, self.direction)
        assert self.ref is not None
        # Inside 85% of the standoff the target is closing on us: stop smoothing for comfort
        # and back off with the full acceleration budget (hysteresis avoids chattering).
        rng = float(np.linalg.norm(tr.pos - ctx.state.pos))
        self.close = rng < (0.95 if self.close else 0.85) * self.p.standoff_m
        self.ref.a_max = self.a_max_close if self.close else self.a_max
        self.ref.omega = 1.6 if self.close else 0.8
        dt = max(ctx.t - self.last_t, 1e-3)
        self.last_t = ctx.t
        pos, vel, acc = self.ref.step(goal.pos, goal.vel, dt)
        self.hold.pos = pos.copy()
        self.hold.yaw = goal.yaw
        return Setpoint(pos=pos, vel=vel, acc=acc, yaw=goal.yaw, yaw_rate=goal.yaw_rate)

    def done(self, ctx: Context) -> bool:
        return ctx.t - self.t0 >= self.duration or ctx.t - self.last_seen > self.lost_timeout

    def gimbal_tilt(self, ctx: Context) -> float | None:
        tr = self._pick(ctx.tracks)
        if tr is None:
            return None
        d = tr.pos - ctx.state.pos
        return math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))


class LandB(Behaviour):
    name = "land"
    low_altitude_ok = True

    def __init__(self) -> None:
        self.commanded = False

    def start(self, ctx: Context) -> None:
        self.commanded = False

    def step(self, ctx: Context) -> Setpoint | None:
        if not self.commanded:
            ctx.vehicle.land()
            self.commanded = True
        return None

    def done(self, ctx: Context) -> bool:
        return self.commanded and not ctx.state.armed


# ---------------------------------------------------------------------------- executive


@dataclass
class ExecutiveStatus:
    behaviour: str
    task_index: int
    n_tasks: int
    aborted: str | None
    finished: bool


class Executive:
    def __init__(self, mission: Mission, cfg: Config, world: WorldConfig) -> None:
        self.mission = mission
        self.cfg = cfg
        self.world = world
        self.tasks: list[Task] = list(mission.tasks)
        self.index = -1
        self.current: Behaviour | None = None
        self.stack: list[Behaviour] = []
        self.altitude = DEFAULT_ALTITUDE
        self.events: list[Event] = []
        self.aborted: str | None = None
        self.finished = False
        self.handled_contacts: set[int] = set()
        self.reported_contacts: set[int] = set()
        self.contact_cooldown_until = 0.0
        self.last_contact: tuple[float, Vec] | None = None  # (time, position)
        self.yield_since: float | None = None
        self.yield_pos: Vec | None = None
        self.yield_vel: Vec = np.zeros(3)
        self.yield_clear_since: float | None = None
        self.watch_radius_m = 70.0
        self.watch_memory_s = 180.0
        self.home = np.array(world.home, dtype=float)

    # ------------------------------------------------------------------ helpers
    def _log(self, t: float, kind: str, detail: str) -> None:
        self.events.append(Event(t, kind, detail))

    def _standoff(self, standoff: float) -> StandoffParams:
        lim = self.cfg.safety
        return StandoffParams(
            standoff_m=standoff,
            max_speed=lim.max_horizontal_speed * 0.75,
            min_altitude=lim.min_altitude + 1.0,
        )

    def _behaviour_for(self, task: Task, ctx: Context) -> Behaviour:
        lim = self.cfg.safety
        g: TaskGeometry = task_geometry(task, ctx.state.pos, self.altitude, self.mission, self.world, lim)
        if not isinstance(task, LandTask):
            self.altitude = g.altitude
        a = lim.max_acceleration * 0.6
        if isinstance(task, TakeoffTask):
            return TakeoffB(task.altitude)
        if isinstance(task, GotoTask):
            return PathB("goto", g.points, g.speed, a, sweep_deg=SWEEP_DEG)
        if isinstance(task, OrbitTask):
            c = g.points.mean(axis=0) if task.laps >= 1 else None
            return PathB("orbit", g.points, g.speed, a, yaw_mode="face" if c is not None else "along", face=c)
        if isinstance(task, SurveyTask):
            return PathB("survey", g.points, g.speed, a, sweep_deg=SWEEP_DEG)
        if isinstance(task, SearchTask):
            return PathB(
                "search", g.points, g.speed, a, yaw_mode="scan", min_time_s=task.duration_s, repeat=True
            )
        if isinstance(task, HoldTask):
            return HoldB(task.duration_s)
        if isinstance(task, TrackTask):
            return TrackB(
                self._standoff(task.standoff_m),
                task.duration_s,
                lim.max_acceleration * 0.6,
                lim.max_acceleration,
            )
        if isinstance(task, ReturnHomeTask):
            return self._rtl_behaviour(ctx)
        if isinstance(task, LandTask):
            return LandB()
        raise ValueError(f"no behaviour for {task!r}")

    def _rtl_behaviour(self, ctx: Context) -> Behaviour:
        lim = self.cfg.safety
        alt = float(np.clip(ctx.state.pos[2], lim.min_altitude + 2.0, lim.max_altitude))
        pts = np.array([[ctx.state.pos[0], ctx.state.pos[1], alt], [self.home[0], self.home[1], alt]])
        return PathB("return_home", pts, lim.rtl_speed, lim.max_acceleration * 0.6, sweep_deg=SWEEP_DEG)

    def _activate(self, b: Behaviour, ctx: Context) -> None:
        self.current = b
        b.start(ctx)
        self._log(ctx.t, "behaviour", b.name)

    def _advance(self, ctx: Context) -> None:
        if isinstance(self.current, TrackB):
            self.contact_cooldown_until = ctx.t + 90.0
        if self.stack:
            b = self.stack.pop()
            self.current = b
            b.resume(ctx)
            self._log(ctx.t, "resume", b.name)
            return
        self.index += 1
        if self.index >= len(self.tasks):
            self.current = None
            self.finished = True
            self._log(ctx.t, "mission", "complete")
            return
        task = self.tasks[self.index]
        b = self._behaviour_for(task, ctx)
        self._activate(b, ctx)
        if isinstance(task, ReturnHomeTask):
            # returning home always ends in a landing
            self.tasks.insert(self.index + 1, LandTask())

    # ------------------------------------------------------------------ aborts
    def abort(self, kind: Emergency | SafetyRequest, ctx: Context, reason: str) -> None:
        label = kind.value
        if self.aborted == label or (self.aborted == "land" and label != "land"):
            return
        if self.aborted == "rtl" and label == "return":
            return
        self.aborted = label
        self.stack.clear()
        self._log(ctx.t, "abort", f"{label}: {reason}")
        if kind in (Emergency.LAND, SafetyRequest.LAND):
            self.tasks = [*self.tasks[: self.index + 1], LandTask()]
            self._activate(LandB(), ctx)
        elif kind in (Emergency.RETURN, SafetyRequest.RTL):
            self.tasks = [*self.tasks[: self.index + 1], LandTask()]
            if ctx.state.armed and not ctx.state.on_ground:
                self._activate(self._rtl_behaviour(ctx), ctx)
            else:
                self._activate(LandB(), ctx)
        else:  # hold position until the operator or the energy monitor decides
            self.tasks = [*self.tasks[: self.index + 1], ReturnHomeTask()]
            self._activate(HoldB(1e9, name="hold (operator stop)"), ctx)

    # ------------------------------------------------------------------ main step
    def _contacts(self, ctx: Context) -> None:
        for tr in ctx.tracks:
            if tr.hits >= 8 and tr.ident not in self.reported_contacts:
                self.reported_contacts.add(tr.ident)
                p = tr.pos
                self._log(ctx.t, "contact", f"track {tr.ident} at ({p[0]:.0f}, {p[1]:.0f}, {p[2]:.0f})")
        if self.mission.on_contact != "track" or self.aborted or self.current is None:
            return
        if isinstance(self.current, TrackB | TakeoffB | LandB) or self.current.name == "return_home":
            return
        fresh = [
            tr
            for tr in ctx.tracks
            if tr.hits >= 8 and tr.pos_std < 15.0 and tr.ident not in self.handled_contacts
        ]
        if not fresh or ctx.t < self.contact_cooldown_until:
            return
        tr = max(fresh, key=lambda x: x.hits)
        self.handled_contacts.add(tr.ident)
        self.stack.append(self.current)
        b = TrackB(
            self._standoff(self.mission.contact_standoff_m),
            self.mission.contact_track_s,
            self.cfg.safety.max_acceleration * 0.6,
            self.cfg.safety.max_acceleration,
        )
        b.target_id = tr.ident
        self._log(ctx.t, "interrupt", f"{self.current.name} -> track {tr.ident}")
        self._activate(b, ctx)

    def _watch_target(self, ctx: Context) -> Vec | None:
        """Position the camera should watch instead of looking along the path, if any."""
        for tr in ctx.tracks:
            if ctx.t - tr.last_hit_t < 1.0:
                self.last_contact = (ctx.t, tr.pos.copy())
        if self.last_contact is None or isinstance(self.current, TrackB | TakeoffB | LandB):
            return None
        t_seen, pos = self.last_contact
        if ctx.t - t_seen > self.watch_memory_s:
            return None
        if float(np.linalg.norm(pos[0:2] - ctx.state.pos[0:2])) > self.watch_radius_m:
            return None
        return pos

    def _conflict(self, ctx: Context, planned_vel: Vec) -> TrackEstimate | None:
        """A known contact whose predicted closest approach to our planned motion is too close.

        Straight-line prediction for both aircraft over ``YIELD_HORIZON_S``; the same
        closest-point-of-approach test manned aviation uses for traffic advisories.
        """
        need = self.cfg.safety.min_separation + YIELD_MARGIN_M
        worst, worst_d = None, math.inf
        for tr in ctx.tracks:
            if ctx.t - tr.last_hit_t > 2.0:  # only contacts we are actually seeing
                continue
            r = ctx.state.pos - tr.pos
            v = planned_vel - tr.vel
            vv = float(v @ v)
            t_cpa = 0.0 if vv < 1e-6 else float(np.clip(-(r @ v) / vv, 0.0, YIELD_HORIZON_S))
            d = float(np.linalg.norm(r + v * t_cpa))
            if d < need and d < worst_d:
                worst, worst_d = tr, d
        return worst

    def _deconflict(self, ctx: Context, sp: Setpoint | None) -> Setpoint | None:
        """Pause path-following behaviours while they would fly into a known contact."""
        if not isinstance(self.current, PathB) or self.aborted == "land":
            self.yield_since = None
            return sp
        if self.yield_since is None:
            if sp is None:
                return sp
            tr = self._conflict(ctx, np.asarray(sp.vel))
            if tr is None:
                return sp
            self.yield_since = ctx.t
            self.yield_pos = ctx.state.pos.copy()
            self.yield_vel = np.asarray(sp.vel, dtype=float).copy()
            self.yield_clear_since = None
            self._log(ctx.t, "yield", f"{self.current.name} paused for track {tr.ident}")
        # yielding: hold position until the planned motion is conflict-free for a while
        if self._conflict(ctx, self.yield_vel) is None:
            if self.yield_clear_since is None:
                self.yield_clear_since = ctx.t
        else:
            self.yield_clear_since = None
        cleared = self.yield_clear_since is not None and ctx.t - self.yield_clear_since >= YIELD_CLEAR_S
        if cleared or ctx.t - self.yield_since >= YIELD_MAX_S:
            if cleared:
                self._log(ctx.t, "resume", f"{self.current.name} (traffic clear)")
                self.current.resume(ctx)
            else:
                self._log(ctx.t, "skip", f"{self.current.name} blocked for {YIELD_MAX_S:.0f} s")
                self.current = None
                self._advance(ctx)
            self.yield_since = None
            return self.current.step(ctx) if self.current is not None else None
        assert self.yield_pos is not None
        return Setpoint(pos=self.yield_pos, yaw=ctx.state.yaw)

    def step(self, ctx: Context) -> Setpoint | None:
        sp = self._deconflict(ctx, self._step(ctx))
        watch = self._watch_target(ctx)
        if sp is not None and watch is not None:
            d = watch - ctx.state.pos
            sp = sp.with_(yaw=math.atan2(d[1], d[0]), yaw_rate=0.0)
        return sp

    def _step(self, ctx: Context) -> Setpoint | None:
        if self.finished:
            return None
        if self.current is None:
            self._advance(ctx)
        self._contacts(ctx)
        # tracked ids stay handled while their track is alive, so the same aircraft
        # does not interrupt the mission over and over
        live = {tr.ident for tr in ctx.tracks}
        if isinstance(self.current, TrackB) and self.current.target_id is not None:
            self.handled_contacts.add(self.current.target_id)
        self.handled_contacts &= live | {getattr(self.current, "target_id", -1)}
        guard = 0
        while self.current is not None and self.current.done(ctx) and guard < 4:
            self._advance(ctx)
            guard += 1
        if self.current is None:
            return None
        return self.current.step(ctx)

    def gimbal_tilt(self, ctx: Context) -> float | None:
        watch = self._watch_target(ctx)
        if watch is not None:
            d = watch - ctx.state.pos
            return math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))
        return self.current.gimbal_tilt(ctx) if self.current else None

    @property
    def low_altitude_ok(self) -> bool:
        return bool(self.current and self.current.low_altitude_ok)

    def status(self) -> ExecutiveStatus:
        return ExecutiveStatus(
            behaviour=self.current.name if self.current else ("done" if self.finished else "idle"),
            task_index=self.index,
            n_tasks=len(self.tasks),
            aborted=self.aborted,
            finished=self.finished,
        )
