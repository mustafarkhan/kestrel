"""Ground truth for the picture: every drone and bird, with what sensors could notice of it."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from kestrel.config import IntruderConfig, Scenario
from kestrel.geometry import Vec
from kestrel.sim.birds import BirdState, Flock
from kestrel.sim.intruder import Intruder


@dataclass(frozen=True)
class TruthObject:
    key: str  # "uav-0", "bird-12"
    kind: str  # "uav" or "bird"
    role: str  # "hostile", "friendly", "civil" for drones; "bird"
    pos: Vec
    vel: Vec
    rcs_m2: float
    radar_class_cue: float  # 0..1: how drone-like its micro-Doppler looks
    rf_link: bool = False
    rf_protocol: str = ""
    remote_id: str | None = None
    remote_id_offset: Vec | None = None
    propulsion: str = "none"


def _bird_rcs(ident: int, lo: float, hi: float, seed: int) -> float:
    r = np.random.default_rng(seed * 7919 + ident)
    return float(np.exp(r.uniform(np.log(lo), np.log(hi))))


class Truth:
    """Samples the scenario's drones and birds at any time, independently of the observer."""

    def __init__(self, scenario: Scenario, flock: Flock | None = None) -> None:
        self.scenario = scenario
        self.intruders = [Intruder(c, i) for i, c in enumerate(scenario.intruders)]
        if flock is None and scenario.birds.count:
            flock = Flock(scenario.birds, scenario.world, float(scenario.world.home[2]), scenario.seed)
        self.flock = flock

    def objects(
        self, t: float, birds: list[BirdState] | None = None, own: tuple[Vec, Vec] | None = None
    ) -> list[TruthObject]:
        """Everything in the air at ``t``. ``own`` is our airborne node's position and velocity:
        ground sensors see it like any other small multirotor with a radio link."""
        out = [o for it in self.intruders if (o := self._drone(it, t)) is not None]
        if own is not None and own[0][2] > 1.0:
            out.append(
                TruthObject(
                    "own-0", "uav", "own", own[0], own[1], 0.02, 0.7, True, "c2-link", None, None, "electric"
                )
            )
        if birds is None:
            birds = self.flock.states(t) if self.flock is not None else []
        bc = self.scenario.birds
        for b in birds:
            cue = bc.radar_class_cue if b.flapping else 0.5  # a gliding bird has no wing-beat
            rcs = _bird_rcs(b.ident, bc.rcs_m2[0], bc.rcs_m2[1], self.scenario.seed)
            out.append(TruthObject(f"bird-{b.ident}", "bird", "bird", b.pos, b.vel, rcs, cue))
        return out

    @staticmethod
    def _drone(it: Intruder, t: float) -> TruthObject | None:
        s = it.state(t)
        if s is None:
            return None
        c: IntruderConfig = it.cfg
        return TruthObject(
            key=f"uav-{it.ident}",
            kind="uav",
            role=c.role,
            pos=s[0],
            vel=s[1],
            rcs_m2=c.rcs_m2,
            radar_class_cue=c.radar_class_cue,
            rf_link=c.rf_link,
            rf_protocol=c.rf_protocol,
            remote_id=c.remote_id,
            remote_id_offset=np.array(c.remote_id_offset_m, dtype=float),
            propulsion=c.propulsion,
        )
