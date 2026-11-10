"""Night and thermal: an unlit drone vanishes from a visible camera at night; a thermal camera
sees it, day or night, without raising false alarms on warm clouds."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from kestrel.config import CameraParams, Config, load_config, load_scenario
from kestrel.geometry import quat_from_euler
from kestrel.perception.camera import Camera
from kestrel.perception.detector import SkyTargetDetector
from kestrel.perception.render import SkyRenderer

ROOT = Path(__file__).resolve().parents[1]
THERMAL = CameraParams(band="thermal", width=320, height=256, hfov_deg=50.0)


def _rate(cfg: Config, camp: CameraParams, light: str, range_m: float, n: int = 30) -> float:
    cam = Camera(camp, cfg.vehicle.camera_offset)
    det = SkyTargetDetector(cam, cfg.detector, "bright" if camp.band == "thermal" else "dark")
    ren = SkyRenderer(cam, seed=7, band=camp.band, light=light)
    rng = np.random.default_rng(5)
    hits = total = 0
    for _ in range(n):
        yaw, tilt = rng.uniform(-math.pi, math.pi), rng.uniform(0, 15)
        pose = cam.pose(np.array([0, 0, 20.0]), quat_from_euler(0, 0, yaw), tilt)
        az, el = yaw + rng.uniform(-0.3, 0.3), math.radians(tilt + rng.uniform(-6, 8))
        tgt = np.array([0, 0, 20.0]) + range_m * np.array(
            [math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el)]
        )
        c = cam.project(pose, tgt)
        if c is None or cam.pixel_ray_world(pose, c[0], c[1])[2] < math.sin(math.radians(3)):
            continue
        total += 1
        img = ren.render(pose, [(tgt, 0.35, 0.3)], camp.noise_sigma)
        hits += any(math.hypot(d.u - c[0], d.v - c[1]) < 3 for d in det.detect(img, pose))
    assert total > 15
    return hits / total


def _false_per_frame(cfg: Config, camp: CameraParams, light: str, n: int = 40) -> float:
    cam = Camera(camp, cfg.vehicle.camera_offset)
    det = SkyTargetDetector(cam, cfg.detector, "bright" if camp.band == "thermal" else "dark")
    ren = SkyRenderer(cam, seed=11, band=camp.band, light=light)
    rng = np.random.default_rng(9)
    false = 0
    for _ in range(n):
        pose = cam.pose(
            np.array([0, 0, 20.0]), quat_from_euler(0, 0, rng.uniform(-math.pi, math.pi)), rng.uniform(0, 20)
        )
        false += len(det.detect(ren.render(pose, [], camp.noise_sigma), pose))
    return false / n


def test_an_unlit_drone_is_invisible_to_a_visible_camera_at_night(cfg: Config) -> None:
    assert _rate(cfg, CameraParams(), "day", 30.0) > 0.9
    assert _rate(cfg, CameraParams(), "night", 30.0) == 0.0


@pytest.mark.parametrize("light", ["day", "night"])
def test_a_thermal_camera_sees_it_day_and_night(cfg: Config, light: str) -> None:
    assert _rate(cfg, THERMAL, light, 30.0) > 0.9
    assert _rate(cfg, THERMAL, light, 80.0) < 0.2  # the same angular resolution limits range


@pytest.mark.parametrize("light", ["day", "night"])
def test_warm_clouds_raise_no_thermal_false_alarms(cfg: Config, light: str) -> None:
    assert _false_per_frame(cfg, THERMAL, light) < 0.1


def test_shipped_thermal_config_and_night_scenario() -> None:
    cfg = load_config(ROOT / "configs" / "thermal.yaml")
    assert cfg.camera.band == "thermal"
    assert cfg.camera.hfov_deg / cfg.camera.width == pytest.approx(75.0 / 480, rel=0.01)
    assert load_scenario(ROOT / "scenarios" / "airfield_night.yaml").world.light == "night"
