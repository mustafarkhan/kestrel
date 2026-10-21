"""Synthetic camera frames for closed-loop perception testing.

Not photorealism, but the properties that make air-to-air detection hard are
there: a target only a few pixels across, a bright structured sky with cloud
edges that look like small dark gaps, textured ground below the horizon,
atmospheric haze, optical blur and sensor noise. The detector therefore runs on
real pixels and its false alarms and misses are emergent, not scripted.
"""

from __future__ import annotations

import math

import numpy as np
from scipy import ndimage

from kestrel.geometry import Vec, rot_z
from kestrel.perception.camera import Camera, CameraPose

_SKY_ZENITH = np.array([92.0, 140.0, 205.0])
_SKY_HORIZON = np.array([200.0, 214.0, 228.0])
_CLOUD = np.array([238.0, 240.0, 244.0])
_GROUND_A = np.array([70.0, 88.0, 52.0])
_GROUND_B = np.array([104.0, 96.0, 70.0])
_TARGET = np.array([42.0, 42.0, 46.0])


def _fractal_noise(rng: np.random.Generator, shape: tuple[int, int], beta: float = 2.4) -> Vec:
    """Fractal noise with a 1/f^beta spectrum, periodic in both axes.

    Built in the frequency domain, so it wraps seamlessly: the cloud map's left and
    right edges are the same azimuth, and a seam there would be a hard vertical edge
    in the sky that the detector then has to cope with.
    """
    h, w = shape
    fy = np.fft.fftfreq(h)[:, None] * h
    fx = np.fft.fftfreq(w)[None, :] * h  # cycles per map height on both axes: isotropic pixels
    k = np.sqrt(fx * fx + fy * fy)
    amplitude = (1.0 + k * k) ** (-beta / 2) * np.exp(-((k / (0.35 * h)) ** 2))
    amplitude[0, 0] = 0.0
    spectrum = np.fft.fft2(rng.standard_normal(shape)) * amplitude
    out = np.real(np.fft.ifft2(spectrum))
    return (out - out.mean()) / (out.std() + 1e-9)


class SkyRenderer:
    def __init__(self, camera: Camera, seed: int = 0, cloud_cover: float = 0.35) -> None:
        self.cam = camera
        rng = np.random.default_rng(seed + 1000)
        # equirectangular cloud map: azimuth [-pi, pi) x elevation [0, pi/2]
        self.cloud_map = _fractal_noise(rng, (256, 1024))
        thr = float(np.quantile(self.cloud_map, 1.0 - cloud_cover))
        self.cloud_alpha = np.clip((self.cloud_map - thr) * 2.5, 0.0, 1.0)
        self.ground_tex = _fractal_noise(rng, (512, 512))
        self.rng = np.random.default_rng(seed + 2000)
        u, v = np.meshgrid(np.arange(camera.width), np.arange(camera.height))
        rays = np.stack(
            [(u - camera.cx) / camera.fx, (v - camera.cy) / camera.fy, np.ones_like(u, dtype=float)],
            axis=-1,
        )
        self.rays = (rays / np.linalg.norm(rays, axis=-1, keepdims=True)).astype(np.float32)
        # sky colour as a lookup on sin(elevation): avoids per-pixel trig and powers
        t = np.linspace(0.0, 1.0, 1024)[:, None] ** 0.6
        self.sky_lut = (_SKY_HORIZON * (1 - t) + _SKY_ZENITH * t).astype(np.float32)
        # sensor noise pool, sampled at a random offset each frame
        n = camera.width * camera.height
        self.noise_pool = self.rng.standard_normal(4 * n).astype(np.float32)
        self._n = n

    def render(
        self, pose: CameraPose, targets: list[tuple[Vec, float, float]], noise_sigma: float
    ) -> np.ndarray:
        """Render an RGB uint8 frame.

        ``targets`` are ``(position, span_m, yaw)`` of aircraft in the world.
        """
        h, w = self.cam.height, self.cam.width
        d = self.rays @ pose.r_world_cam.T.astype(np.float32)  # (H, W, 3) world directions
        dz = d[..., 2]
        sky = dz >= 0.0

        # --- sky: gradient + clouds (equirectangular map, bilinear lookup)
        sin_el = np.clip(dz, 0.0, 1.0)
        img = self.sky_lut[(sin_el * 1023).astype(np.int32)]
        el = np.arcsin(sin_el)
        az = np.arctan2(d[..., 1], d[..., 0])
        ci = el * (255.0 / (math.pi / 2))
        cj = (az + math.pi) * (1024.0 / (2 * math.pi))
        alpha = ndimage.map_coordinates(self.cloud_alpha, [ci.ravel(), cj.ravel()], order=1, mode="grid-wrap")
        a = (alpha.reshape(h, w) * np.clip(el * 12.0, 0.0, 1.0)).astype(np.float32)[..., None]
        img = img + (_CLOUD.astype(np.float32) - img) * a

        # --- ground: textured plane with haze towards the horizon
        if not sky.all():
            hgt = max(float(pose.position[2]), 0.5)
            dzg = np.minimum(dz, -1e-4)
            rng_g = hgt / -dzg
            gx = pose.position[0] + d[..., 0] * rng_g
            gy = pose.position[1] + d[..., 1] * rng_g
            ti = (gx * 2.0).astype(np.int64) % 512
            tj = (gy * 2.0).astype(np.int64) % 512
            tex = np.clip(0.5 + 0.25 * self.ground_tex[ti, tj], 0.0, 1.0).astype(np.float32)[..., None]
            col = _GROUND_A.astype(np.float32) + (_GROUND_B - _GROUND_A).astype(np.float32) * tex
            haze = np.clip(rng_g / 1500.0, 0.0, 0.85).astype(np.float32)[..., None]
            col = col + (_SKY_HORIZON.astype(np.float32) - col) * haze
            img = np.where(sky[..., None], img, col)

        for pos, span, yaw in targets:
            self._draw_aircraft(img, pose, pos, span, yaw)

        if noise_sigma > 0:
            off = int(self.rng.integers(0, 3 * self._n))
            img = img + (noise_sigma * self.noise_pool[off : off + self._n]).reshape(h, w, 1)
        out: np.ndarray = np.clip(img, 0, 255).astype(np.uint8)
        return out

    def _draw_aircraft(self, img: np.ndarray, pose: CameraPose, pos: Vec, span: float, yaw: float) -> None:
        cam = self.cam
        c = cam.project(pose, pos)
        if c is None:
            return
        u0, v0, depth = c
        r_px = cam.fx * (span / 2) / depth
        if r_px < 0.15:
            return
        pad = math.ceil(r_px * 1.6 + 3)
        x0, x1 = int(u0) - pad, int(u0) + pad + 1
        y0, y1 = int(v0) - pad, int(v0) + pad + 1
        if x1 < 0 or y1 < 0 or x0 >= cam.width or y0 >= cam.height:
            return
        # Silhouette: central body + four rotor discs, rasterised with 4x supersampling.
        ss = 4
        # pixel i covers [i - 0.5, i + 0.5), matching the projection's pixel centres
        xs = np.arange(x0 * ss, x1 * ss) / ss + 0.5 / ss - 0.5
        ys = np.arange(y0 * ss, y1 * ss) / ss + 0.5 / ss - 0.5
        gx, gy = np.meshgrid(xs, ys)
        cover = np.zeros_like(gx)
        rz = rot_z(yaw)
        parts = [(np.zeros(3), 0.22)]
        for k in range(4):
            ang = math.pi / 4 + k * math.pi / 2
            parts.append((rz @ np.array([math.cos(ang), math.sin(ang), 0.0]) * span * 0.36, 0.16))
        for off, rad in parts:
            pc = cam.project(pose, pos + off)
            if pc is None:
                continue
            rr = max(cam.fx * rad * span / pc[2], 0.35)
            cover = np.maximum(cover, ((gx - pc[0]) ** 2 + (gy - pc[1]) ** 2 <= rr * rr).astype(float))
        alpha = cover.reshape(y1 - y0, ss, x1 - x0, ss).mean(axis=(1, 3))
        alpha = ndimage.gaussian_filter(alpha, 0.6)  # optics PSF
        alpha *= min(1.0, 0.55 + 0.45 * r_px)  # sub-pixel targets lose contrast
        ya, yb = max(y0, 0), min(y1, cam.height)
        xa, xb = max(x0, 0), min(x1, cam.width)
        a = alpha[ya - y0 : yb - y0, xa - x0 : xb - x0][..., None]
        img[ya:yb, xa:xb] = img[ya:yb, xa:xb] * (1 - a) + _TARGET * a
