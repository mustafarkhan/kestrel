"""Acoustic cueing network: triangulation accuracy, honesty of its covariance, coverage."""

from __future__ import annotations

import numpy as np

from kestrel.config import AcousticParams, WorldConfig
from kestrel.perception.acoustic import AcousticNetwork, node_grid

WORLD = WorldConfig(fence=[(-150, -150), (150, -150), (150, 150), (-150, 150)])


def test_grid_covers_the_area() -> None:
    nodes = node_grid(WORLD, 100.0)
    assert len(nodes) == 9
    assert np.all(np.abs(nodes) <= 150)


def test_fix_accuracy_and_covariance_are_consistent() -> None:
    net = AcousticNetwork(AcousticParams(detection_probability=1.0), WORLD, np.random.default_rng(1))
    truth = np.array([40.0, -30.0, 30.0])
    errs, nis = [], []
    for k in range(400):
        fixes = net.sense(float(k), [(0, truth, np.zeros(3))])
        assert len(fixes) == 1
        e = fixes[0].xy - truth[0:2]
        errs.append(float(np.linalg.norm(e)))
        nis.append(float(e @ np.linalg.solve(fixes[0].cov_xy, e)))
    # several nodes within 250 m at ~2 deg each: a few metres of error, as the geometry predicts
    assert np.median(errs) < 6.0
    # normalised innovation squared for a 2-D Gaussian averages 2: the stated covariance is honest
    assert 1.5 < float(np.mean(nis)) < 2.6


def test_no_fix_from_a_single_node() -> None:
    params = AcousticParams(detection_probability=1.0, range_m=60.0)
    net = AcousticNetwork(params, WORLD, np.random.default_rng(2))
    near_one_node = np.array([-100.0, -100.0, 30.0])  # within 60 m of exactly one grid node
    assert net.sense(0.0, [(0, near_one_node, np.zeros(3))]) == []


def test_out_of_range_is_silent() -> None:
    small = WorldConfig(fence=[(-50, -50), (50, -50), (50, 50), (-50, 50)])
    net = AcousticNetwork(AcousticParams(detection_probability=1.0), small, np.random.default_rng(3))
    assert net.sense(0.0, [(0, np.array([900.0, 900.0, 30.0]), np.zeros(3))]) == []
