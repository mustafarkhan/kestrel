"""The fusion node in the loop: ground sensors and Kestrel feed one picture, and the picture
tasks Kestrel.

Each tick the ground sensors report on the true drones and birds, Kestrel's own SAPIENT
messages (whatever reached the ground over the link) are read, the picture is updated and
scored, and, every ``task_period_s``, the fusion node picks the track that most needs a
closer look and sends Kestrel's camera node a ``LOOK_AT`` task. While Kestrel is on its
way, the same track's newer position is re-sent so the look follows a moving aircraft.
A look ends when Kestrel's camera has reported on that track and the picture can say what
it is (drone or bird), when the track is gone, or after ``look_timeout_s``.

The picture is also shared back with Kestrel as **traffic**: every confirmed track the
fusion node holds in three dimensions (radar or fused, with a measured altitude) is sent to
Kestrel as a SAPIENT detection report and enters its separation filter alongside its own
camera tracks. The ground sensors see Kestrel too; the fusion node recognises that track as
its own node from Kestrel's status reports and leaves it out. Kestrel's camera sees only
what is in front of it; without the picture's traffic, a look flown to one aircraft can pass
under another the radar holds and the camera does not. The traffic can only push Kestrel
away from an aircraft, never draw it towards one.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from kestrel.config import Scenario
from kestrel.geofence import Geofence
from kestrel.geometry import Vec
from kestrel.interop.sapient import SapientExporter
from kestrel.rap.fusion import FusionNode, Message
from kestrel.rap.score import PictureScore
from kestrel.rap.sensors import SensorNetwork
from kestrel.rap.tasking import LookTasker
from kestrel.rap.truth import Truth
from kestrel.sim.birds import BirdState


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
        self.tasker = LookTasker(self.fusion, self.refresh_s, self.look_timeout_s)
        self.uav_class_t: dict[str, float] = {}  # truth drone -> first time its track was called UAV

    def tick(self, t: float, birds: list[BirdState], own: tuple[Vec, Vec] | None = None) -> list[Message]:
        """Advance the ground sensors and the fusion node to ``t``.

        Returns the messages the fusion node sends Kestrel: ``LOOK_AT`` tasks, and its 3-D
        tracks as detection reports (traffic for the separation filter). ``own`` is Kestrel's
        true position and velocity, which the ground sensors see like any other aircraft.
        """
        if t < self._next_t:
            return []
        self._next_t += self.dt
        objects = self.truth.objects(t, birds, own)
        for msg in self.net.step(t, objects):
            self.fusion.ingest(msg)
        delivered = self.exporter.messages[self._seen :]
        self._seen = len(self.exporter.messages)
        for msg in delivered:
            self.fusion.ingest(msg)
        self.fusion.step(t)
        node = self.exporter.camera_node_id
        out = self.fusion.traffic_reports(t, node) if node in self.fusion.node_types else []
        picture = self.fusion.picture(t)
        self.score.add(t, self.dt, objects, picture, self.fusion, self.fence)
        if not self.tasking or t < self._next_task_t:
            return out
        self._next_task_t += self.task_period_s
        return out + self.tasker.step(t)

    def summary(self) -> dict[str, Any]:
        out: dict[str, Any] = dict(self.score.summary())
        out["tasks_sent"] = self.tasker.tasks_sent
        out["looks"] = len(self.tasker.looks) + (self.tasker.active is not None)
        out["looks_resolved"] = sum(lk["outcome"] == "resolved" for lk in self.tasker.looks)
        return out
