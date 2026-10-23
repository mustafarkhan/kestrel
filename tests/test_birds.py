"""Birds: the flock model, the flicker classifier, and the classifier on rendered frames."""

from __future__ import annotations

import math

import numpy as np

from kestrel.config import BirdConfig, Config, WorldConfig
from kestrel.perception.camera import Camera
from kestrel.perception.classify import FlickerClassifier
from kestrel.perception.detector import SkyTargetDetector
from kestrel.perception.render import SkyRenderer
from kestrel.sim.birds import BirdState, Flock

WORLD = WorldConfig()


def test_flock_keeps_its_size_and_stays_near_the_area() -> None:
    flock = Flock(BirdConfig(count=5), WORLD, 0.0, seed=1)
    for t in np.arange(0.0, 120.0, 1.0):
        states = flock.states(float(t))
        assert len(states) == 5
        for b in states:
            assert np.all(np.abs(b.pos[0:2]) <= 150 + 120 + 1e-6)
            assert -5.0 < b.pos[2] < 70.0
            assert 8.0 <= float(np.linalg.norm(b.vel[0:2])) <= 13.0
            assert 0.1 - 1e-9 <= b.spread <= 1.0


def test_flock_is_deterministic_per_seed() -> None:
    a = Flock(BirdConfig(count=3), WORLD, 0.0, seed=4).states(30.0)
    b = Flock(BirdConfig(count=3), WORLD, 0.0, seed=4).states(30.0)
    assert all(np.allclose(x.pos, y.pos) for x, y in zip(a, b, strict=True))


def test_intermittent_flight_glides_about_as_often_as_configured() -> None:
    flock = Flock(BirdConfig(count=20, glide_fraction=0.3), WORLD, 0.0, seed=2)
    flapping = [b.flapping for t in np.arange(0.0, 300.0, 0.5) for b in flock.states(float(t))]
    assert 0.6 < float(np.mean(flapping)) < 0.8


def _feed(clf: FlickerClassifier, flux: np.ndarray) -> None:
    for k, f in enumerate(flux):
        clf.add(0.1 * k, float(f))


def test_classifier_needs_evidence_before_it_decides() -> None:
    clf = FlickerClassifier()
    _feed(clf, np.full(5, 300.0))
    assert clf.p_uav() == 0.5


def test_classifier_separates_steady_from_flickering_flux() -> None:
    rng = np.random.default_rng(0)
    for mean in (200.0, 400.0, 1500.0):  # far, middle, near
        noise = 1.57 / math.sqrt(2.0) / math.sqrt(mean)  # the per-frame log noise the model expects
        steady, flapping = FlickerClassifier(), FlickerClassifier()
        _feed(steady, mean * np.exp(rng.normal(0.0, noise, 30)))
        beat = 0.55 + 0.45 * np.cos(2 * math.pi * 4.0 * 0.1 * np.arange(30))
        _feed(flapping, mean * beat * np.exp(rng.normal(0.0, noise, 30)))
        assert steady.p_uav() > 0.8, mean
        assert flapping.p_uav() < 0.2, mean


def test_too_faint_to_classify_gives_no_opinion() -> None:
    clf = FlickerClassifier()
    beat = 0.55 + 0.45 * np.cos(2 * math.pi * 4.0 * 0.1 * np.arange(30))
    _feed(clf, 60.0 * beat)  # flickering, but at a brightness where noise would look the same
    assert clf.p_uav() == 0.5


def test_frame_gaps_do_not_count_as_flicker() -> None:
    clf = FlickerClassifier()
    for k in range(20):  # a steady target seen every other second at very different brightness
        clf.add(1.0 * k, 200.0 if k % 2 else 800.0)
    assert clf.pairs == 0


def test_rendered_flapping_bird_is_rejected_and_drone_kept() -> None:
    """End to end on rendered pixels: renderer -> detector flux -> classifier."""
    cfg = Config()
    cam = Camera(cfg.camera, cfg.vehicle.camera_offset)
    det = SkyTargetDetector(cam, cfg.detector)
    obs = np.array([0.0, 0.0, 30.0])
    pose = cam.pose(obs, np.array([1.0, 0.0, 0.0, 0.0]), cfg.camera.pitch_up_deg)
    results = {}
    for kind in ("drone", "bird"):
        ren = SkyRenderer(cam, seed=11)
        clf = FlickerClassifier()
        # flying towards the camera, so the wings sweep across the line of sight; a bird seen
        # side-on shows its wing-beat mostly as up-and-down motion and flickers less
        p0 = obs + np.array([50.0, -6.0, 6.0])
        vel = np.array([-4.0, 0.0, 0.0])
        for k in range(30):
            t = 0.1 * k
            pos = p0 + vel * t
            if kind == "drone":
                img = ren.render(pose, [(pos, 0.35, 0.3 * t)], cfg.camera.noise_sigma)
            else:
                ph = 2 * math.pi * 4.5 * t
                b = BirdState(0, pos, vel, 0.7, 0.55 + 0.45 * math.cos(ph), 0.21 * math.sin(ph), True)
                img = ren.render(pose, [], cfg.camera.noise_sigma, birds=[b])
            uv = cam.project(pose, pos)
            assert uv is not None
            hits = [d for d in det.detect(img, pose) if math.hypot(d.u - uv[0], d.v - uv[1]) < 4]
            if hits:
                clf.add(t, hits[0].flux)
        assert clf.pairs >= 20, kind
        results[kind] = clf.p_uav()
    assert results["drone"] > 0.6
    assert results["bird"] < 0.4
