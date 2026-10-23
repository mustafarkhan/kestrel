"""Birds: wandering, intermittently flapping flyers that look like small drones at range.

Every survey of counter-UAS sensing names birds as the dominant false-alarm source: at a
few hundred metres a crow and a quadcopter are both a dark blob a few pixels across. The
model gives the renderer what the detector actually sees:

* a silhouette (body plus two wings) whose visible wing extent and vertical wing position
  change through the wing-beat, so the blob's area and darkness flicker;
* intermittent flight: flapping bouts alternate with glides, during which the wings are
  held out and the silhouette is as steady as a drone's;
* wandering paths at bird speeds and altitudes, replaced on the far side of the area when
  a bird leaves it, so the number in view stays roughly constant.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from kestrel.config import BirdConfig, WorldConfig
from kestrel.geometry import Vec

STEP_S = 0.05
MARGIN_M = 120.0  # birds roam this far outside the geofence box


@dataclass
class BirdState:
    ident: int
    pos: Vec
    vel: Vec
    span: float
    spread: float  # visible fraction of the wing (1: fully extended)
    tip_dz: float  # wing-tip height relative to the body, metres
    flapping: bool


@dataclass
class _Bird:
    ident: int
    pos: Vec
    heading: float
    turn_rate: float
    climb: float
    speed: float
    span: float
    flap_hz: float
    phase: float
    flapping: bool
    bout_left: float


class Flock:
    def __init__(self, cfg: BirdConfig, world: WorldConfig, home_alt: float, seed: int) -> None:
        self.cfg = cfg
        self.rng = np.random.default_rng(seed + 4242)
        f = np.asarray(world.fence, dtype=float)
        self.lo = f.min(axis=0) - MARGIN_M
        self.hi = f.max(axis=0) + MARGIN_M
        self.home_alt = home_alt
        self.t = 0.0
        self._next_id = 0
        self.birds = [self._spawn(inside=True) for _ in range(cfg.count)]

    def _spawn(self, inside: bool) -> _Bird:
        c, r = self.cfg, self.rng
        if inside:
            xy = r.uniform(self.lo, self.hi)
            heading = r.uniform(-math.pi, math.pi)
        else:  # enter from a random edge, heading roughly across the area
            side = int(r.integers(4))
            u = r.uniform(0.0, 1.0)
            corners = [(self.lo[0], self.lo[1] + u * (self.hi[1] - self.lo[1])),
                       (self.hi[0], self.lo[1] + u * (self.hi[1] - self.lo[1])),
                       (self.lo[0] + u * (self.hi[0] - self.lo[0]), self.lo[1]),
                       (self.lo[0] + u * (self.hi[0] - self.lo[0]), self.hi[1])]  # fmt: skip
            xy = np.array(corners[side])
            centre = 0.5 * (self.lo + self.hi) + r.normal(0.0, 60.0, 2)
            heading = math.atan2(centre[1] - xy[1], centre[0] - xy[0])
        alt = r.uniform(*c.altitude)
        b = _Bird(
            ident=self._next_id,
            pos=np.array([xy[0], xy[1], self.home_alt + alt]),
            heading=heading,
            turn_rate=0.0,
            climb=0.0,
            speed=r.uniform(*c.speed),
            span=r.uniform(*c.span_m),
            flap_hz=r.uniform(*c.flap_hz),
            phase=r.uniform(0.0, 2 * math.pi),
            flapping=bool(r.random() > c.glide_fraction),
            bout_left=0.0,
        )
        b.bout_left = self._bout(b.flapping)
        self._next_id += 1
        return b

    def _bout(self, flapping: bool) -> float:
        c = self.cfg
        if c.glide_fraction <= 0.0:
            return math.inf if flapping else 0.0
        if c.glide_fraction >= 1.0:
            return 0.0 if flapping else math.inf
        mean = c.mean_bout_s if flapping else c.mean_bout_s * c.glide_fraction / (1.0 - c.glide_fraction)
        return float(self.rng.exponential(mean))

    def advance(self, t: float) -> None:
        while self.t + STEP_S <= t + 1e-9:
            self._step(STEP_S)
            self.t += STEP_S

    def _step(self, dt: float) -> None:
        c, r = self.cfg, self.rng
        lo_z, hi_z = self.home_alt + c.altitude[0], self.home_alt + c.altitude[1]
        for i, b in enumerate(self.birds):
            # wandering: turn rate and climb rate are slow random processes (time constant 4 s)
            b.turn_rate += (-b.turn_rate / 4.0) * dt + 0.25 * math.sqrt(dt) * r.normal()
            b.climb += (-b.climb / 4.0) * dt + 0.5 * math.sqrt(dt) * r.normal()
            b.heading += b.turn_rate * dt
            b.climb = float(np.clip(b.climb, -1.5, 1.5))
            if (b.pos[2] < lo_z and b.climb < 0) or (b.pos[2] > hi_z and b.climb > 0):
                b.climb = -b.climb
            b.pos = b.pos + np.array(
                [b.speed * math.cos(b.heading) * dt, b.speed * math.sin(b.heading) * dt, b.climb * dt]
            )
            if b.flapping:
                b.phase += 2 * math.pi * b.flap_hz * dt
            b.bout_left -= dt
            if b.bout_left <= 0.0:
                b.flapping = not b.flapping
                b.bout_left = self._bout(b.flapping)
            if np.any(b.pos[0:2] < self.lo) or np.any(b.pos[0:2] > self.hi):
                self.birds[i] = self._spawn(inside=False)

    def states(self, t: float) -> list[BirdState]:
        self.advance(t)
        out = []
        for b in self.birds:
            vel = np.array([b.speed * math.cos(b.heading), b.speed * math.sin(b.heading), b.climb])
            if b.flapping:
                spread = 0.55 + 0.45 * math.cos(b.phase)  # wings sweep in and out: 0.1 .. 1.0
                tip_dz = 0.3 * b.span * math.sin(b.phase)
            else:
                spread, tip_dz = 1.0, 0.04 * b.span  # glide: wings out, slight dihedral
            out.append(BirdState(b.ident, b.pos.copy(), vel, b.span, spread, tip_dz, b.flapping))
        return out
