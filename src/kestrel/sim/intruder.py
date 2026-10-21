"""Ground-truth motion of non-cooperative small UAS ("intruders")."""

from __future__ import annotations

import math

import numpy as np

from kestrel.config import IntruderConfig
from kestrel.geometry import Vec
from kestrel.guidance.trajectory import Path, PathFollower


class Intruder:
    """Kinematic target following a closed-form or piecewise-linear path.

    ``state(t)`` returns ``None`` before the target appears.
    """

    def __init__(self, cfg: IntruderConfig, ident: int = 0) -> None:
        self.cfg = cfg
        self.ident = ident
        self.center = np.array(cfg.center, dtype=float)
        ax, ay = cfg.size
        if cfg.pattern == "lemniscate":
            # Gerono lemniscate: x = a sin s, y = b sin s cos s; length estimated numerically
            s = np.linspace(0.0, 2 * math.pi, 2001)
            xy = np.stack([ax * np.sin(s), ay * np.sin(s) * np.cos(s)], axis=1)
            length = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
            self._omega = 2 * math.pi * cfg.speed / length
        elif cfg.pattern == "orbit":
            self._omega = cfg.speed / max(ax, 1.0)
        else:
            self._omega = 0.0
        if cfg.pattern == "waypoints":
            if len(cfg.waypoints) < 2:
                raise ValueError("waypoints pattern needs at least two waypoints")
            # Real multirotors cannot reverse velocity instantly: corners are rounded and
            # the speed profile respects a 3 m/s^2 acceleration limit (3 m/s^2 lateral).
            loop = np.array([*cfg.waypoints, cfg.waypoints[0]], dtype=float)
            loop_len = float(np.sum(np.linalg.norm(np.diff(loop, axis=0), axis=1)))
            reps = math.ceil(cfg.speed * 900.0 / max(loop_len, 1.0)) + 1
            pts = np.vstack([loop[:-1]] * reps + [loop[-1:]])
            self._follower = PathFollower(Path(pts, corner_radius=6.0), cfg.speed, 3.0, 3.0)

    def state(self, t: float) -> tuple[Vec, Vec] | None:
        tau = t - self.cfg.appear_at_s
        if tau < 0.0:
            return None
        c = self.cfg
        ax, ay = c.size
        if c.pattern == "lemniscate":
            s = self._omega * tau
            w = self._omega
            pos = self.center + np.array(
                [ax * math.sin(s), ay * math.sin(s) * math.cos(s), 3.0 * math.sin(0.5 * s)]
            )
            vel = w * np.array([ax * math.cos(s), ay * math.cos(2 * s), 1.5 * math.cos(0.5 * s)])
            return pos, vel
        if c.pattern == "orbit":
            s = self._omega * tau
            pos = self.center + np.array([ax * math.cos(s), ax * math.sin(s), 0.0])
            vel = self._omega * np.array([-ax * math.sin(s), ax * math.cos(s), 0.0])
            return pos, vel
        if c.pattern == "straight":
            heading = np.array([ax, ay, 0.0])
            n = float(np.linalg.norm(heading)) or 1.0
            vel = heading / n * c.speed
            return self.center + vel * tau, vel
        # waypoints, looped, with rounded corners and bounded acceleration
        sp, _ = self._follower.sample(tau, 0.0)
        return sp.pos, sp.vel
