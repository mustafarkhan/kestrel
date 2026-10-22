from __future__ import annotations

import math

import numpy as np
from hypothesis import given
from hypothesis import strategies as st

from kestrel.geometry import (
    enu_to_ned,
    hat,
    quat_from_euler,
    quat_mul,
    quat_to_euler,
    quat_to_rot,
    rot_to_quat,
    vee,
    wrap_pi,
)
from kestrel.vehicle.mavlink_vehicle import quat_ned_frd_to_enu_flu, yaw_enu_to_ned

angles = st.floats(-3.0, 3.0)
small = st.floats(-1.4, 1.4)


@given(small, small, angles)
def test_euler_roundtrip(r: float, p: float, y: float) -> None:
    rr, pp, yy = quat_to_euler(quat_from_euler(r, p, y))
    assert math.isclose(rr, r, abs_tol=1e-9)
    assert math.isclose(pp, p, abs_tol=1e-9)
    assert math.isclose(wrap_pi(yy - y), 0.0, abs_tol=1e-9)


@given(angles, angles, angles)
def test_rotation_quaternion_roundtrip(r: float, p: float, y: float) -> None:
    q = quat_from_euler(r, p, y)
    rot = quat_to_rot(q)
    assert np.allclose(rot @ rot.T, np.eye(3), atol=1e-12)
    assert np.allclose(quat_to_rot(rot_to_quat(rot)), rot, atol=1e-9)


def test_quat_mul_composes_rotations() -> None:
    a = quat_from_euler(0.1, -0.2, 0.7)
    b = quat_from_euler(-0.3, 0.4, -1.1)
    assert np.allclose(quat_to_rot(quat_mul(a, b)), quat_to_rot(a) @ quat_to_rot(b), atol=1e-12)


def test_hat_vee() -> None:
    v = np.array([0.3, -1.2, 2.0])
    w = np.array([1.0, 0.5, -0.4])
    assert np.allclose(hat(v) @ w, np.cross(v, w))
    assert np.allclose(vee(hat(v)), v)


@given(st.floats(-50, 50), st.floats(-50, 50), st.floats(-50, 50))
def test_enu_ned_is_involution(x: float, y: float, z: float) -> None:
    v = np.array([x, y, z])
    assert np.allclose(enu_to_ned(enu_to_ned(v)), v)


def test_mavlink_attitude_conversion() -> None:
    # Level, nose pointing north in NED (yaw 0) is yaw +90 deg in ENU.
    q = quat_ned_frd_to_enu_flu(np.array([1.0, 0.0, 0.0, 0.0]))
    assert math.isclose(quat_to_euler(q)[2], math.pi / 2, abs_tol=1e-9)
    # Nose east in NED (yaw +90) is yaw 0 in ENU.
    q = quat_ned_frd_to_enu_flu(np.array([math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)]))
    assert math.isclose(quat_to_euler(q)[2], 0.0, abs_tol=1e-9)
    # FRD body z (down) maps to ENU -z when level.
    assert np.allclose(quat_to_rot(q) @ np.array([0, 0, 1.0]), [0, 0, 1.0], atol=1e-9)


@given(angles)
def test_yaw_conversion_matches_frames(yaw: float) -> None:
    # heading vector in ENU, converted to NED, must have the NED yaw we send
    h_enu = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    h_ned = enu_to_ned(h_enu)
    assert math.isclose(wrap_pi(math.atan2(h_ned[1], h_ned[0]) - yaw_enu_to_ned(yaw)), 0.0, abs_tol=1e-9)
