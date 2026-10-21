"""Perception tests on rendered frames: geometry, detection rates, clutter, calibration."""

from __future__ import annotations

import math

import numpy as np
import pytest

from kestrel.config import Config, IntruderConfig
from kestrel.geometry import quat_from_euler
from kestrel.perception.camera import Camera
from kestrel.perception.detector import Detection, SkyTargetDetector
from kestrel.perception.render import SkyRenderer
from kestrel.perception.tracker import MultiTargetTracker, measurement_model
from kestrel.sim.intruder import Intruder


@pytest.fixture()
def cam(cfg: Config) -> Camera:
    return Camera(cfg.camera, cfg.vehicle.camera_offset)


def test_projection_and_ray_are_inverse(cam: Camera) -> None:
    pose = cam.pose(np.array([3.0, -2.0, 20.0]), quat_from_euler(0.1, -0.05, 0.8), 12.0)
    p = np.array([40.0, 30.0, 28.0])
    u, v, _ = cam.project(pose, p)  # type: ignore[misc]
    ray = cam.pixel_ray_world(pose, u, v)
    assert np.allclose(pose.position + ray * np.linalg.norm(p - pose.position), p, atol=1e-6)


def test_gimbal_decouples_airframe_tilt(cam: Camera) -> None:
    level = cam.pose(np.zeros(3), quat_from_euler(0.0, 0.0, 0.5), 10.0)
    tilted = cam.pose(np.zeros(3), quat_from_euler(0.4, -0.3, 0.5), 10.0)
    assert np.allclose(level.r_world_cam, tilted.r_world_cam)


@pytest.mark.parametrize("roll,pitch,tilt", [(0, 0, 8), (0.3, -0.2, 8), (0, 0, 35), (0, 0, -20)])
def test_sky_mask_never_covers_ground(
    cfg: Config, cam: Camera, roll: float, pitch: float, tilt: float
) -> None:
    det = SkyTargetDetector(cam, cfg.detector)
    pose = cam.pose(np.array([0, 0, 20.0]), quat_from_euler(roll, pitch, 0.4), tilt)
    mask = det.sky_mask(pose)
    for v in range(0, cam.height, 12):
        for u in range(0, cam.width, 12):
            if mask[v, u]:
                assert cam.pixel_ray_world(pose, u, v)[2] > 0


def _random_frames(cfg: Config, cam: Camera, n: int, seed: int, with_target: bool):
    rng = np.random.default_rng(seed)
    renderer = SkyRenderer(cam, seed=seed)
    for _ in range(n):
        yaw = rng.uniform(-math.pi, math.pi)
        tilt = rng.uniform(0, 20)
        pose = cam.pose(np.array([0, 0, 20.0]), quat_from_euler(0, 0, yaw), tilt)
        rng_m = rng.uniform(12, 55)
        az = yaw + rng.uniform(-0.4, 0.4)
        el = math.radians(tilt + rng.uniform(-10, 12))
        tgt = np.array([0, 0, 20.0]) + rng_m * np.array(
            [math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el)]
        )
        targets = [(tgt, 0.35, 0.3)] if with_target else []
        yield pose, tgt, renderer.render(pose, targets, cfg.camera.noise_sigma)


def test_detection_rate_and_size_calibration(cfg: Config, cam: Camera) -> None:
    det = SkyTargetDetector(cam, cfg.detector)
    hits, total, log_err = 0, 0, []
    for pose, tgt, img in _random_frames(cfg, cam, 60, seed=101, with_target=True):
        u, v, depth = cam.project(pose, tgt)  # type: ignore[misc]
        if not (15 < u < cam.width - 15 and 15 < v < cam.height - 15):
            continue
        if cam.pixel_ray_world(pose, u, v)[2] < math.sin(math.radians(3)):
            continue  # near-horizon targets are excluded by design
        total += 1
        found = [d for d in det.detect(img, pose) if math.hypot(d.u - u, d.v - v) < 3]
        if found:
            hits += 1
            log_err.append(math.log(found[0].size_px / (cam.fx * 0.35 / depth)))
    assert total > 25
    assert hits / total > 0.8
    assert abs(float(np.mean(log_err))) < 0.15 and float(np.std(log_err)) < 0.3


def test_clouds_do_not_produce_false_alarms(cfg: Config, cam: Camera) -> None:
    det = SkyTargetDetector(cam, cfg.detector)
    false = sum(len(det.detect(img, pose)) for pose, _, img in _random_frames(cfg, cam, 60, 202, False))
    assert false / 60 < 0.1


def test_measurement_jacobian_matches_finite_difference() -> None:
    x = np.array([30.0, -12.0, 8.0, 1.0, 2.0, 0.0])
    c = np.array([1.0, 2.0, 3.0])
    _, jac = measurement_model(x, c)
    num = np.zeros((3, 6))
    for i in range(6):
        dx = np.zeros(6)
        dx[i] = 1e-6
        num[:, i] = (measurement_model(x + dx, c)[0] - measurement_model(x - dx, c)[0]) / 2e-6
    assert np.allclose(jac, num, atol=1e-6)


def test_tracker_follows_one_target_through_misses(cfg: Config, cam: Camera) -> None:
    tracker = MultiTargetTracker(cfg.tracker, cam)
    target = Intruder(IntruderConfig(pattern="lemniscate", center=(20, 60, 30), size=(40, 22), speed=4.0))
    rng = np.random.default_rng(3)
    obs = np.array([0, 0, 22.0])
    errs = []
    for k in range(400):
        t = k * 0.1
        p, _ = target.state(t)  # type: ignore[misc]
        los = p - obs
        pose = cam.pose(obs, quat_from_euler(0, 0, math.atan2(los[1], los[0])), 10.0)
        u, v, z = cam.project(pose, p)  # type: ignore[misc]
        size = cam.fx * 0.35 / z * math.exp(rng.normal(0, 0.18))
        dets = [Detection(u + rng.normal(0, 0.3), v + rng.normal(0, 0.3), size, 30.0, (0, 0, 1, 1))]
        # 20 % missed detections plus one random clutter blob every few frames
        if rng.random() < 0.2:
            dets = []
        if rng.random() < 0.3:
            dets.append(Detection(rng.uniform(0, 480), rng.uniform(0, 180), 2.0, 15.0, (0, 0, 1, 1)))
        tracker.update(t, dets, pose)
        est = [e for e in tracker.estimates(t) if np.linalg.norm(e.pos - p) < 15]
        if est:
            errs.append(float(np.linalg.norm(est[0].pos - p)))
    assert len(errs) > 360  # confirmed on the true target almost the whole time
    assert float(np.median(errs)) < 5.0
    assert len({e.ident for e in tracker.estimates(40.0)}) == 1
