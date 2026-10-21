"""Rotation and frame utilities.

Conventions used throughout the package:

* World frame: ENU (x east, y north, z up), metres.
* Body frame: FLU (x forward, y left, z up).
* Quaternions are Hamilton, scalar-first ``(w, x, y, z)`` and rotate body
  vectors into the world frame: ``v_world = R(q) @ v_body``.

MAVLink uses NED/FRD; the conversion lives in :mod:`kestrel.vehicle.mavlink_vehicle`
and nowhere else, so the rest of the code never has to think about it.
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray

Vec = NDArray[np.float64]

E3: Vec = np.array([0.0, 0.0, 1.0])


def hat(v: Vec) -> Vec:
    """Skew-symmetric matrix such that ``hat(a) @ b == cross(a, b)``."""
    return np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ]
    )


def vee(m: Vec) -> Vec:
    """Inverse of :func:`hat` for a skew-symmetric matrix."""
    return np.array([m[2, 1], m[0, 2], m[1, 0]])


def quat_to_rot(q: Vec) -> Vec:
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def rot_to_quat(r: Vec) -> Vec:
    """Shepperd's method; numerically stable for all rotations."""
    tr = float(np.trace(r))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = np.array([0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s])
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        q = np.array([(r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s])
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        q = np.array([(r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s])
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        q = np.array([(r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s])
    q = q / np.linalg.norm(q)
    out: Vec = q if q[0] >= 0 else -q
    return out


def quat_mul(a: Vec, b: Vec) -> Vec:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def quat_from_euler(roll: float, pitch: float, yaw: float) -> Vec:
    """ZYX (yaw-pitch-roll) Euler angles in radians to a quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def quat_to_euler(q: Vec) -> tuple[float, float, float]:
    """Quaternion to ZYX Euler angles ``(roll, pitch, yaw)`` in radians."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def yaw_of(q: Vec) -> float:
    return quat_to_euler(q)[2]


def wrap_pi(a: float) -> float:
    """Wrap an angle to ``[-pi, pi)``."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def rot_z(yaw: float) -> Vec:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(pitch: float) -> Vec:
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def enu_to_ned(v: Vec) -> Vec:
    """ENU <-> NED is its own inverse: swap x/y, negate z."""
    return np.array([v[1], v[0], -v[2]])


ned_to_enu = enu_to_ned
