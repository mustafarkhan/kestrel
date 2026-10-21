"""Pinhole camera rigidly mounted on the observer, tilted up by a fixed angle.

Camera frame follows the OpenCV convention (x right, y down, z forward) so the
intrinsics and pixel maths match what a real calibrated camera would give.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from kestrel.config import CameraParams
from kestrel.geometry import Vec, quat_to_rot, rot_y, rot_z, yaw_of


@dataclass(frozen=True)
class CameraPose:
    position: Vec  # world ENU
    r_world_cam: Vec  # columns: camera x, y, z axes expressed in world

    def jittered(self, rng: np.random.Generator, sigma_rad: float) -> CameraPose:
        """Small random rotation: residual gimbal stabilisation error."""
        a, b = rng.normal(0.0, sigma_rad, 2)
        return CameraPose(self.position, self.r_world_cam @ rot_y(a) @ _rx(b))


def _rx(a: float) -> Vec:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _cam_axes(tilt_up: float) -> Vec:
    """Columns: camera right, down, forward in a frame whose x is the boresight azimuth."""
    fwd = np.array([math.cos(tilt_up), 0.0, math.sin(tilt_up)])
    right = np.array([0.0, -1.0, 0.0])
    return np.column_stack([right, np.cross(fwd, right), fwd])


class Camera:
    """Pinhole camera on either a rigid mount or a 2-axis stabilised gimbal.

    With the gimbal, camera roll and pitch are decoupled from the airframe: the
    boresight azimuth follows the vehicle yaw and the elevation is commanded
    (``tilt_deg``). Without it, the camera tilts with every manoeuvre, which is
    exactly what makes rigid-mount air-to-air tracking fragile.
    """

    def __init__(self, params: CameraParams, offset_body: tuple[float, float, float]) -> None:
        self.p = params
        self.width = params.width
        self.height = params.height
        self.fx = (params.width / 2) / math.tan(math.radians(params.hfov_deg) / 2)
        self.fy = self.fx
        self.cx = (params.width - 1) / 2
        self.cy = (params.height - 1) / 2
        self.offset = np.array(offset_body, dtype=float)
        self.r_body_cam = _cam_axes(math.radians(params.pitch_up_deg))
        self.vfov = 2 * math.atan((params.height / 2) / self.fy)

    @property
    def intrinsics(self) -> Vec:
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])

    def pose(self, pos: Vec, quat: Vec, tilt_deg: float | None = None) -> CameraPose:
        r_wb = quat_to_rot(quat)
        position = pos + r_wb @ self.offset
        if not self.p.gimbal:
            return CameraPose(position=position, r_world_cam=r_wb @ self.r_body_cam)
        tilt = self.p.pitch_up_deg if tilt_deg is None else tilt_deg
        tilt = min(max(tilt, self.p.gimbal_min_deg), self.p.gimbal_max_deg)
        r = rot_z(yaw_of(quat)) @ _cam_axes(math.radians(tilt))
        return CameraPose(position=position, r_world_cam=r)

    def project(self, pose: CameraPose, point_world: Vec) -> tuple[float, float, float] | None:
        """Return ``(u, v, depth)`` or ``None`` if behind the camera."""
        d = pose.r_world_cam.T @ (point_world - pose.position)
        if d[2] <= 0.1:
            return None
        return self.fx * d[0] / d[2] + self.cx, self.fy * d[1] / d[2] + self.cy, float(d[2])

    def in_view(self, pose: CameraPose, point_world: Vec, margin_px: float = 0.0) -> bool:
        uvz = self.project(pose, point_world)
        if uvz is None:
            return False
        u, v, _ = uvz
        return (
            -margin_px <= u <= self.width - 1 + margin_px and -margin_px <= v <= self.height - 1 + margin_px
        )

    def pixel_ray_world(self, pose: CameraPose, u: float, v: float) -> Vec:
        ray = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        w = pose.r_world_cam @ ray
        out: Vec = w / np.linalg.norm(w)
        return out

    def horizon_row(self, pose: CameraPose, u: float) -> float:
        """Image row of the true horizon (elevation 0) at column ``u``.

        Solves for ``v`` such that the pixel ray has zero world z component.
        """
        r = pose.r_world_cam
        a = (u - self.cx) / self.fx
        # world_z(ray) = r[2,0]*a + r[2,1]*b + r[2,2] = 0  with  b = (v - cy)/fy
        if abs(r[2, 1]) < 1e-9:
            return math.inf if (r[2, 0] * a + r[2, 2]) < 0 else -math.inf
        b = -(r[2, 0] * a + r[2, 2]) / r[2, 1]
        return float(b * self.fy + self.cy)
