"""The fusion node in the loop: ground sensors and Kestrel feed one picture, and the picture
tasks Kestrel.

Each tick the ground sensors report on the true drones and birds, Kestrel's own SAPIENT
messages (whatever reached the ground over the link) are read, the picture is updated and
scored, and, every ``task_period_s``, the fusion node picks the track that most needs a
closer look and sends Kestrel's camera node a ``LOOK_AT`` task. While Kestrel is on its
way, the same track's newer position is re-sent so the look follows a moving aircraft.
A look ends when Kestrel's camera has reported on that track and the picture can say what
it is (drone or bird), when the track is gone, or after ``look_timeout_s``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from kestrel.config import Scenario
from kestrel.geofence import Geofence
from kestrel.interop.sapient import SapientExporter
from kestrel.rap.fusion import FusionNode, Message
from kestrel.rap.score import PictureScore
from kestrel.rap.sensors import SensorNetwork
from kestrel.rap.truth import Truth
from kestrel.sim.birds import BirdState


@dataclass
class Look:
    track: int
    start_t: float
    last_sent_t: float


@dataclass
class AirPicture:
    scenario: Scenario
    exporter: SapientExporter
    include: set[str] | None = None
    tasking: bool = True
    dt: float = 0.5
    task_period_s: float = 1.0
    refresh_s: float = 3.0
    look_timeout_s: float = 75.0
    altitude_band_m: tuple[float, float] = (0.0, 60.0)
    score: PictureScore = field(default_factory=PictureScore)
    tasks_sent: int = 0
    looks: list[dict[str, Any]] = field(default_factory=list)

    def __post_init__(self) -> None:
        net_cfg = self.scenario.sensors
        assert net_cfg is not None, "scenario has no sensor network"
        self.truth = Truth(self.scenario, flock=None)
        self.net = SensorNetwork(
            net_cfg,
            self.scenario.world,
            self.exporter,
            self.scenario.seed,
            self.include,
            self.altitude_band_m,
        )
        self.fusion = FusionNode(self.exporter, self.scenario.world, net_cfg.whitelist, net_cfg.protected)
        self.fence = Geofence(self.scenario.world.fence)
        self._seen = 0
        self._next_t = 0.0
        self._next_task_t = 0.0
        self.active: Look | None = None
        self.uav_class_t: dict[str, float] = {}  # truth drone -> first time its track was called UAV

    def tick(self, t: float, birds: list[BirdState]) -> list[Message]:
        if t < self._next_t:
            return []
        self._next_t += self.dt
        objects = self.truth.objects(t, birds)
        for msg in self.net.step(t, objects):
            self.fusion.ingest(msg)
        delivered = self.exporter.messages[self._seen :]
        self._seen = len(self.exporter.messages)
        for msg in delivered:
            self.fusion.ingest(msg)
        self.fusion.step(t)
        picture = self.fusion.picture(t)
        self.score.add(t, self.dt, objects, picture, self.fusion, self.fence)
        if not self.tasking or t < self._next_task_t:
            return []
        self._next_task_t += self.task_period_s
        return self._task(t)

    def _task(self, t: float) -> list[Message]:
        live = {tr.ident: tr for tr in self.fusion.tracks}
        look = self.active
        if look is not None:
            tr = live.get(look.track)
            # A look is done when the camera has seen the track and the picture can now say what
            # it is, when the track is gone, or when it has taken too long.
            resolved = (
                tr is not None and tr.camera_t >= look.start_t and self.fusion.class_of(tr) != "Unknown"
            )
            done = tr is None or resolved or t - look.start_t > self.look_timeout_s
            if done:
                outcome = "gone" if tr is None else ("resolved" if resolved else "timeout")
                self.looks.append(
                    {"track": look.track, "start_t": look.start_t, "end_t": t, "outcome": outcome}
                )
                self.active = None
            elif tr is not None and t - look.last_sent_t >= self.refresh_s:
                look.last_sent_t = t
                self.tasks_sent += 1
                return [self.fusion.look_at_task(t, tr)]
            else:
                return []
        cand = self.fusion.most_needed_look(t)
        if cand is None or self.fusion.camera_node is None:
            return []
        self.active = Look(cand.ident, t, t)
        self.tasks_sent += 1
        return [self.fusion.look_at_task(t, cand)]

    def summary(self) -> dict[str, Any]:
        out: dict[str, Any] = dict(self.score.summary())
        out["tasks_sent"] = self.tasks_sent
        out["looks"] = len(self.looks) + (self.active is not None)
        out["looks_resolved"] = sum(lk["outcome"] == "resolved" for lk in self.looks)
        return out
