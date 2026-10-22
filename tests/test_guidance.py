from __future__ import annotations

import math

import numpy as np
from hypothesis import given, settings
from hypothesis import strategies as st

from kestrel.guidance.observer import StandoffParams, standoff_setpoint
from kestrel.guidance.trajectory import Path, PathFollower, ReferenceFilter, lawnmower, orbit_points


def _samples(f: PathFollower, n: int = 400):
    ts = np.linspace(0.0, f.duration, n)
    sps = [f.sample(float(t), 0.0)[0] for t in ts]
    return ts, np.array([s.pos for s in sps]), np.array([s.vel for s in sps]), np.array([s.acc for s in sps])


def test_follower_is_kinematically_consistent() -> None:
    pts = np.vstack([[[0, 0, 20]], lawnmower((-40, 20), (40, 80), 20, 20)])
    f = PathFollower(Path(pts), v_max=6.0, a_max=2.4)
    ts, p, v, _ = _samples(f, 2000)
    dt = ts[1] - ts[0]
    assert np.abs(np.gradient(p, dt, axis=0) - v)[3:-3].max() < 0.25
    assert np.linalg.norm(v, axis=1).max() <= 6.0 + 1e-6
    assert np.allclose(p[0], pts[0]) and np.linalg.norm(p[-1] - pts[-1]) < 1e-6


def test_follower_respects_lateral_acceleration() -> None:
    f = PathFollower(Path(orbit_points(np.array([0, 0, 20.0]), 10.0, 1.0, 0.0, False)), 8.0, 3.0, 3.0)
    _, _, v, a = _samples(f)
    lateral = np.linalg.norm(
        a - (np.sum(a * v, axis=1) / np.maximum(np.sum(v * v, axis=1), 1e-9))[:, None] * v, axis=1
    )
    assert lateral.max() < 3.0 * 1.25
    assert np.linalg.norm(v, axis=1).max() < math.sqrt(3.0 * 10.0) * 1.05


def test_curvature_only_slows_corners() -> None:
    # long straight with one corner: cruise should reach v_max on the straights
    pts = np.array([[0, 0, 20], [100, 0, 20], [100, 100, 20]], dtype=float)
    f = PathFollower(Path(pts, corner_radius=4.0), v_max=8.0, a_max=3.0)
    assert f.v.max() > 7.9
    assert f.v.min() == 0.0


def test_lawnmower_stays_inside_area() -> None:
    pts = lawnmower((-60, 40), (60, 120), 25, 30)
    assert np.all(pts[:, 0] >= -60) and np.all(pts[:, 0] <= 60)
    assert np.all(pts[:, 1] >= 40) and np.all(pts[:, 1] <= 120)
    assert len(pts) == 2 * 3


@settings(max_examples=50, deadline=None)
@given(st.floats(-200, 200), st.floats(-200, 200), st.floats(-30, 30))
def test_reference_filter_limits(gx: float, gy: float, gz: float) -> None:
    rf = ReferenceFilter(np.zeros(3), np.zeros(3), omega=0.8, v_max=6.0, a_max=2.4, vz_max=2.0)
    for _ in range(400):
        _, v, a = rf.step(np.array([gx, gy, gz]), np.zeros(3), 0.05)
        assert math.hypot(v[0], v[1]) <= 6.0 + 1e-9
        assert abs(v[2]) <= 2.0 + 1e-9
        # speed clipping can bend the acceleration slightly within one step
        assert np.linalg.norm(a) <= 2.4 * 1.5 + 1e-6


def test_standoff_goal_geometry() -> None:
    p = StandoffParams(standoff_m=30.0, look_up_deg=10.0)
    sp = standoff_setpoint(np.array([0.0, 0.0, 20.0]), np.array([50.0, 0.0, 30.0]), np.zeros(3), p)
    rel = sp.pos - np.array([50.0, 0.0, 30.0])
    assert math.isclose(float(np.linalg.norm(rel)), 30.0, rel_tol=1e-6)
    assert rel[2] < 0  # below the target, looking up at sky
    assert math.isclose(sp.yaw, 0.0, abs_tol=0.25)  # facing the target
