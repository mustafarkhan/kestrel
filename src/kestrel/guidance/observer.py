"""Standoff observation guidance for a tracked aircraft.

Goal: keep a non-cooperative target in the camera at a fixed standoff range,
with three choices that matter in practice:

* **Look up, not down.** The observer holds station slightly *below* the target
  (``look_up_deg``), so the target is seen against sky rather than ground
  clutter. That is where the detector works best.
* **Keep moving.** A monocular sensor measures bearing well and range poorly.
  A slow orbit around the target (``parallax_speed``) creates bearing parallax,
  which is what makes range observable in the tracker.
* **Bounded aggression.** This function only produces the *goal*. The
  executive passes it through a :class:`~kestrel.guidance.trajectory.ReferenceFilter`
  so that a jump in the estimate becomes a smooth, speed- and
  acceleration-limited reference. Separation is enforced again downstream by the
  safety monitor; this module only aims for the standoff.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from kestrel.geometry import Vec
from kestrel.types import Setpoint


@dataclass(frozen=True)
class StandoffParams:
    standoff_m: float = 30.0
    look_up_deg: float = 10.0
    parallax_speed: float = 1.5  # tangential m/s around the target
    max_speed: float = 8.0
    min_altitude: float = 6.0


def standoff_setpoint(
    observer: Vec, target_pos: Vec, target_vel: Vec, p: StandoffParams, direction: float = 1.0
) -> Setpoint:
    rel = observer - target_pos
    horiz = math.hypot(rel[0], rel[1])
    u = np.array([rel[0], rel[1], 0.0]) / horiz if horiz > 1e-3 else np.array([1.0, 0.0, 0.0])
    tangent = direction * np.array([-u[1], u[0], 0.0])
    el = math.radians(p.look_up_deg)
    goal = target_pos + u * p.standoff_m * math.cos(el) - np.array([0.0, 0.0, p.standoff_m * math.sin(el)])
    goal[2] = max(goal[2], p.min_altitude)

    vel = np.array(target_vel, dtype=float) + tangent * p.parallax_speed
    vel[2] = float(np.clip(vel[2], -1.5, 1.5))
    sp_h = math.hypot(vel[0], vel[1])
    if sp_h > p.max_speed:
        vel[0:2] *= p.max_speed / sp_h

    los = target_pos - observer
    yaw = math.atan2(los[1], los[0])
    hd = max(math.hypot(los[0], los[1]), 1.0)
    rel_v = np.asarray(target_vel) - vel
    yaw_rate = float((los[0] * rel_v[1] - los[1] * rel_v[0]) / (hd * hd))
    return Setpoint(pos=goal, vel=vel, yaw=yaw, yaw_rate=float(np.clip(yaw_rate, -1.0, 1.0)))
