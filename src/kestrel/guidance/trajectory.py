"""Path geometry and time-parameterised references.

Paths are dense 3-D polylines (corners filleted so heading changes are
continuous). A :class:`PathFollower` moves a reference along a path with a
curvature-aware speed profile, and returns position, velocity and
acceleration feed-forward that are mutually consistent: velocity is the time
derivative of position and acceleration includes the centripetal term on arcs.
That consistency is what lets the flight controller track tightly without the
jerky corrections you get from position-only waypoints.
"""

from __future__ import annotations

import itertools
import math

import numpy as np

from kestrel.geometry import Vec
from kestrel.types import Setpoint

SAMPLE_M = 0.5


def _densify(points: Vec, step: float = SAMPLE_M) -> Vec:
    out = [points[0]]
    for a, b in itertools.pairwise(points):
        seg = float(np.linalg.norm(b - a))
        n = max(1, math.ceil(seg / step))
        for k in range(1, n + 1):
            out.append(a + (b - a) * (k / n))
    return np.array(out)


def fillet(points: Vec, radius: float) -> Vec:
    """Replace each interior corner with a circular arc of the given radius."""
    if len(points) < 3 or radius <= 0:
        return np.asarray(points, dtype=float)
    out = [points[0]]
    for i in range(1, len(points) - 1):
        p0, p1, p2 = points[i - 1], points[i], points[i + 1]
        a = p0 - p1
        b = p2 - p1
        la, lb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
        if la < 1e-6 or lb < 1e-6:
            continue
        ua, ub = a / la, b / lb
        cosang = float(np.clip(ua @ ub, -1.0, 1.0))
        ang = math.acos(cosang)
        if ang > math.pi - 1e-3:  # straight through
            out.append(p1)
            continue
        cut = min(radius / math.tan(ang / 2), 0.45 * la, 0.45 * lb)
        r_eff = cut * math.tan(ang / 2)
        start, end = p1 + ua * cut, p1 + ub * cut
        bis = ua + ub
        bis /= np.linalg.norm(bis)
        center = p1 + bis * (r_eff / math.sin(ang / 2))
        v0, v1 = start - center, end - center
        n = max(2, math.ceil((math.pi - ang) * r_eff / SAMPLE_M))
        for k in range(n + 1):
            s = k / n
            # slerp between v0 and v1 around the arc
            om = math.acos(float(np.clip((v0 @ v1) / (np.linalg.norm(v0) * np.linalg.norm(v1)), -1, 1)))
            if om < 1e-6:
                out.append(start + (end - start) * s)
            else:
                out.append(center + (math.sin((1 - s) * om) * v0 + math.sin(s * om) * v1) / math.sin(om))
    out.append(points[-1])
    return np.array(out)


class Path:
    def __init__(self, points: Vec, corner_radius: float = 4.0) -> None:
        pts = np.asarray(points, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) < 1:
            raise ValueError("path needs an (N, 3) array")
        if len(pts) == 1:
            pts = np.vstack([pts, pts])
        self.points = _densify(fillet(pts, corner_radius))
        seg = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(seg)])
        self.length = float(self.s[-1])

    def at(self, s: float) -> tuple[Vec, Vec, Vec]:
        """Point, unit tangent and curvature vector at arc length ``s``."""
        if self.length < 1e-6:
            return self.points[0], np.zeros(3), np.zeros(3)
        s = min(max(s, 0.0), self.length)
        i = int(np.clip(np.searchsorted(self.s, s, side="right") - 1, 0, len(self.s) - 2))
        ds = self.s[i + 1] - self.s[i]
        f = (s - self.s[i]) / ds if ds > 1e-9 else 0.0
        p = self.points[i] + f * (self.points[i + 1] - self.points[i])
        t = self._tangent(i)
        j = min(i + 1, len(self.s) - 2)
        dt_ = self._tangent(j) - self._tangent(max(i - 1, 0))
        # tangents of segments i-1 and j sit at their midpoints, s[j] - s[i-1] apart
        span = self.s[j] - self.s[max(i - 1, 0)]
        kappa = dt_ / span if span > 1e-6 else np.zeros(3)
        return p, t, kappa

    def _tangent(self, i: int) -> Vec:
        d = self.points[i + 1] - self.points[i]
        n = float(np.linalg.norm(d))
        return d / n if n > 1e-9 else np.zeros(3)

    def closest_s(self, p: Vec) -> float:
        i = int(np.argmin(np.linalg.norm(self.points - p, axis=1)))
        return float(self.s[i])


class PathFollower:
    """Time-parameterised reference along a :class:`Path`.

    Speed is planned per point along the path: each point gets a limit from the
    lateral-acceleration budget at its curvature, then a forward pass (accelerate)
    and a backward pass (brake in time) make the profile feasible under
    ``a_max``. So the vehicle slows only for the corners that need it, instead of
    the whole path running at the speed of its tightest turn.
    """

    def __init__(
        self, path: Path, v_max: float, a_max: float, v_max_lateral_acc: float = 3.0, v0: float = 0.0
    ) -> None:
        self.path = path
        s = path.s
        n = len(s)
        kappa = np.array([float(np.linalg.norm(path.at(float(si))[2])) for si in s])
        v_lim = np.minimum(v_max, np.sqrt(v_max_lateral_acc / np.maximum(kappa, 1e-9)))
        v = v_lim.copy()
        v[0] = min(v0, v_lim[0])
        v[-1] = 0.0
        ds = np.diff(s)
        for i in range(n - 1):  # forward: cannot accelerate faster than a_max
            v[i + 1] = min(v[i + 1], math.sqrt(v[i] ** 2 + 2 * a_max * ds[i]))
        for i in range(n - 2, -1, -1):  # backward: must be able to brake for what comes next
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_max * ds[i]))
        t = np.zeros(n)
        for i in range(n - 1):
            vm = v[i] + v[i + 1]
            t[i + 1] = t[i] + (2 * ds[i] / vm if vm > 1e-6 else 0.0)
        self.v = v
        self.t = t
        self.duration = float(t[-1])

    def _state(self, t: float) -> tuple[float, float, float]:
        if t <= 0.0 or len(self.t) < 2:
            return 0.0, float(self.v[0]), 0.0
        if t >= self.duration:
            return self.path.length, 0.0, 0.0
        i = int(np.clip(np.searchsorted(self.t, t, side="right") - 1, 0, len(self.t) - 2))
        dt = self.t[i + 1] - self.t[i]
        if dt <= 1e-9:
            return float(self.path.s[i]), float(self.v[i]), 0.0
        acc = (self.v[i + 1] - self.v[i]) / dt
        tau = t - self.t[i]
        vel = self.v[i] + acc * tau
        return float(self.path.s[i] + self.v[i] * tau + 0.5 * acc * tau * tau), float(vel), float(acc)

    def sample(self, t: float, yaw: float) -> tuple[Setpoint, Vec]:
        s, v, a = self._state(t)
        p, tan, kappa = self.path.at(s)
        sp = Setpoint(pos=p, vel=tan * v, acc=tan * a + kappa * v * v, yaw=yaw)
        return sp, tan

    def done(self, t: float) -> bool:
        return t >= self.duration


# ---------------------------------------------------------------------- generators


def orbit_points(center: Vec, radius: float, laps: float, start_angle: float, clockwise: bool) -> Vec:
    n = max(16, int(abs(laps) * 2 * math.pi * radius / SAMPLE_M))
    sign = -1.0 if clockwise else 1.0
    th = start_angle + sign * np.linspace(0, laps * 2 * math.pi, n)
    return np.stack(
        [center[0] + radius * np.cos(th), center[1] + radius * np.sin(th), np.full(n, center[2])], axis=1
    )


def lawnmower(min_xy: tuple[float, float], max_xy: tuple[float, float], alt: float, spacing: float) -> Vec:
    """Boustrophedon lanes along x, stepping in y; first lane at half spacing inside."""
    x0, y0 = min_xy
    x1, y1 = max_xy
    spacing = max(spacing, 1.0)
    ys = np.arange(y0 + spacing / 2, y1, spacing)
    if len(ys) == 0:
        ys = np.array([(y0 + y1) / 2])
    pts = []
    for k, y in enumerate(ys):
        xs = (x0 + spacing / 2, x1 - spacing / 2) if k % 2 == 0 else (x1 - spacing / 2, x0 + spacing / 2)
        pts.append((xs[0], y, alt))
        pts.append((xs[1], y, alt))
    return np.array(pts, dtype=float)


class ReferenceFilter:
    """Second-order reference model that turns a moving goal into feasible setpoints.

    Critically damped tracking of ``(goal, goal_velocity)`` with saturated
    acceleration and speed. Used where the goal is produced by perception (and so
    can jump): the vehicle receives a smooth, dynamically consistent reference
    instead of the raw estimate.
    """

    def __init__(self, pos: Vec, vel: Vec, omega: float, v_max: float, a_max: float, vz_max: float) -> None:
        self.r = np.array(pos, dtype=float)
        self.v = np.array(vel, dtype=float)
        self.omega = omega
        self.v_max = v_max
        self.a_max = a_max
        self.vz_max = vz_max

    def step(self, goal: Vec, goal_vel: Vec, dt: float) -> tuple[Vec, Vec, Vec]:
        w = self.omega
        acc = w * w * (goal - self.r) + 2.0 * w * (goal_vel - self.v)
        an = float(np.linalg.norm(acc))
        if an > self.a_max:
            acc *= self.a_max / an
        v = self.v + acc * dt
        vh = math.hypot(v[0], v[1])
        if vh > self.v_max:
            v[0:2] *= self.v_max / vh
        v[2] = float(np.clip(v[2], -self.vz_max, self.vz_max))
        acc = (v - self.v) / dt
        self.r = self.r + 0.5 * (self.v + v) * dt
        self.v = v
        return self.r.copy(), self.v.copy(), acc
