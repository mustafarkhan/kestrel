"""Multi-target tracker: global-nearest-neighbour association + per-track EKF.

State per track: ``x = [p, v]`` in world ENU with a constant-velocity model
driven by white acceleration noise.

Measurement from one monocular detection, relative to the camera centre ``c``:

    z = [azimuth, elevation, log(range)]

Bearing is precise (pixel centroid + attitude error). Range comes only from
the apparent size of an object of *assumed* size, so it is noisy and biased by
however wrong that assumption is; working in ``log(range)`` makes that error
multiplicative, which matches how it actually behaves. Range then becomes well
determined over time as the observer moves (bearing parallax), which is why the
guidance layer keeps the observer manoeuvring rather than sitting still.

Track management is M-of-N confirmation on frames where the track is predicted
to be inside the field of view, and deletion after a maximum coast time.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import StrEnum

import numpy as np
from scipy.optimize import linear_sum_assignment

from kestrel.config import TrackerParams
from kestrel.geometry import Vec, wrap_pi
from kestrel.perception.camera import Camera, CameraPose
from kestrel.perception.detector import Detection


class TrackStatus(StrEnum):
    TENTATIVE = "tentative"
    CONFIRMED = "confirmed"


@dataclass
class Track:
    ident: int
    x: Vec
    cov: Vec
    t: float
    status: TrackStatus = TrackStatus.TENTATIVE
    history: list[bool] = field(default_factory=list)
    last_hit_t: float = 0.0
    hits: int = 0

    @property
    def pos(self) -> Vec:
        return self.x[0:3]

    @property
    def vel(self) -> Vec:
        return self.x[3:6]

    @property
    def pos_std(self) -> float:
        return math.sqrt(max(float(np.trace(self.cov[0:3, 0:3])), 0.0))


@dataclass(frozen=True)
class TrackEstimate:
    ident: int
    pos: Vec
    vel: Vec
    cov: Vec
    confirmed: bool
    hits: int
    last_hit_t: float

    @property
    def pos_std(self) -> float:
        return math.sqrt(max(float(np.trace(self.cov[0:3, 0:3])), 0.0))


def measurement_model(x: Vec, c: Vec) -> tuple[Vec, Vec]:
    """Return ``h(x)`` and its Jacobian ``H`` (3x6)."""
    d = x[0:3] - c
    hsq = d[0] ** 2 + d[1] ** 2
    h = math.sqrt(max(hsq, 1e-9))
    rho2 = hsq + d[2] ** 2
    rho = math.sqrt(rho2)
    z = np.array([math.atan2(d[1], d[0]), math.atan2(d[2], h), math.log(rho)])
    jac = np.zeros((3, 6))
    jac[0, 0:3] = [-d[1] / max(hsq, 1e-9), d[0] / max(hsq, 1e-9), 0.0]
    jac[1, 0:3] = [-d[2] * d[0] / (rho2 * h), -d[2] * d[1] / (rho2 * h), h / rho2]
    jac[2, 0:3] = d / rho2
    return z, jac


def _residual(z: Vec, zp: Vec) -> Vec:
    r = z - zp
    r[0] = wrap_pi(float(r[0]))
    return r


class MultiTargetTracker:
    def __init__(self, params: TrackerParams, camera: Camera) -> None:
        self.p = params
        self.cam = camera
        sb = math.radians(params.bearing_sigma_deg)
        self.r_meas = np.diag([sb**2, sb**2, params.log_range_sigma**2])
        self.tracks: list[Track] = []
        self._next_id = 1
        self.false_tracks_spawned = 0

    # ------------------------------------------------------------------ helpers
    def measurement(self, det: Detection, pose: CameraPose) -> Vec:
        ray = self.cam.pixel_ray_world(pose, det.u, det.v)
        rng = self.cam.fx * self.p.target_size_m / max(det.size_px, 0.5)
        return np.array([math.atan2(ray[1], ray[0]), math.asin(ray[2]), math.log(rng)])

    def _predict(self, tr: Track, t: float) -> None:
        dt = t - tr.t
        if dt <= 0.0:
            return
        f = np.eye(6)
        f[0:3, 3:6] = np.eye(3) * dt
        q = self.p.accel_noise**2
        qm = np.zeros((6, 6))
        qm[0:3, 0:3] = np.eye(3) * q * dt**3 / 3
        qm[0:3, 3:6] = qm[3:6, 0:3] = np.eye(3) * q * dt**2 / 2
        qm[3:6, 3:6] = np.eye(3) * q * dt
        tr.x = f @ tr.x
        tr.cov = f @ tr.cov @ f.T + qm
        tr.t = t

    def _init_track(self, z: Vec, c: Vec, t: float) -> Track:
        az, el, lr = z
        u = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        rng = math.exp(lr)
        x = np.concatenate([c + rng * u, np.zeros(3)])
        sr = rng * self.p.log_range_sigma
        st = rng * math.radians(self.p.bearing_sigma_deg)
        pos_cov = sr**2 * np.outer(u, u) + st**2 * (np.eye(3) - np.outer(u, u))
        cov = np.zeros((6, 6))
        cov[0:3, 0:3] = pos_cov
        cov[3:6, 3:6] = np.eye(3) * 8.0**2
        tr = Track(self._next_id, x, cov, t, history=[True], last_hit_t=t, hits=1)
        self._next_id += 1
        return tr

    # ------------------------------------------------------------------ main step
    def update(self, t: float, detections: list[Detection], pose: CameraPose) -> None:
        c = pose.position
        zs = [self.measurement(d, pose) for d in detections]
        for tr in self.tracks:
            self._predict(tr, t)

        n_t, n_d = len(self.tracks), len(zs)
        assigned_t: set[int] = set()
        assigned_d: set[int] = set()
        if n_t and n_d:
            cost = np.full((n_t, n_d), 1e6)
            cache: dict[tuple[int, int], tuple[Vec, Vec, Vec]] = {}
            for i, tr in enumerate(self.tracks):
                zp, jac = measurement_model(tr.x, c)
                s = jac @ tr.cov @ jac.T + self.r_meas
                s_inv = np.linalg.inv(s)
                for j, z in enumerate(zs):
                    r = _residual(z, zp)
                    d2 = float(r @ s_inv @ r)
                    if d2 < self.p.gate_chi2:
                        # bearing-dominated cost so a range outlier does not steal the association
                        cost[i, j] = d2 + (0.0 if tr.status is TrackStatus.CONFIRMED else 1.0)
                        cache[(i, j)] = (r, jac, s_inv)
            rows, cols = linear_sum_assignment(cost)
            for i, j in zip(rows, cols, strict=True):
                if cost[i, j] >= 1e6:
                    continue
                r, jac, s_inv = cache[(i, j)]
                tr = self.tracks[i]
                k = tr.cov @ jac.T @ s_inv
                tr.x = tr.x + k @ r
                ikh = np.eye(6) - k @ jac
                tr.cov = ikh @ tr.cov @ ikh.T + k @ self.r_meas @ k.T  # Joseph form
                tr.history.append(True)
                tr.hits += 1
                tr.last_hit_t = t
                assigned_t.add(i)
                assigned_d.add(j)

        for i, tr in enumerate(self.tracks):
            if i not in assigned_t and self.cam.in_view(pose, tr.pos, margin_px=-4):
                tr.history.append(False)
        for j, z in enumerate(zs):
            if j not in assigned_d and detections[j].score >= self.p.init_score:
                self.tracks.append(self._init_track(z, c, t))
        self._manage(t)

    def _manage(self, t: float) -> None:
        p = self.p
        keep: list[Track] = []
        for tr in self.tracks:
            recent = tr.history[-p.confirm_window :]
            if tr.status is TrackStatus.TENTATIVE:
                if sum(recent) >= p.confirm_hits:
                    tr.status = TrackStatus.CONFIRMED
                elif len(recent) >= p.confirm_window or t - tr.last_hit_t > p.max_coast_s:
                    self.false_tracks_spawned += 1
                    continue
            elif t - tr.last_hit_t > p.max_coast_s or tr.pos_std > p.max_position_std:
                continue
            tr.history = tr.history[-p.confirm_window :]
            keep.append(tr)
        # Merge duplicates: a younger track sitting on top of an older confirmed one.
        keep.sort(key=lambda tr: (tr.status is not TrackStatus.CONFIRMED, tr.ident))
        merged: list[Track] = []
        for tr in keep:
            dup = any(
                float(np.linalg.norm(tr.pos - o.pos)) < 2.0 * max(1.0, min(tr.pos_std, o.pos_std))
                for o in merged
            )
            if not dup:
                merged.append(tr)
        self.tracks = merged

    def estimates(self, t: float, confirmed_only: bool = True) -> list[TrackEstimate]:
        out = []
        for tr in self.tracks:
            if confirmed_only and tr.status is not TrackStatus.CONFIRMED:
                continue
            dt = max(0.0, t - tr.t)
            x = tr.x.copy()
            x[0:3] += x[3:6] * dt
            cov = tr.cov.copy()
            cov[0:3, 0:3] += np.eye(3) * (self.p.accel_noise**2) * dt**3 / 3 + tr.cov[3:6, 3:6] * dt**2
            out.append(
                TrackEstimate(
                    tr.ident, x[0:3], x[3:6], cov, tr.status is TrackStatus.CONFIRMED, tr.hits, tr.last_hit_t
                )
            )
        return out
