"""Site and config files are an operator's input: a typo must stop the flight on the ground."""

from __future__ import annotations

import math
from typing import Any

import pytest
from pydantic import ValidationError

from kestrel.config import CameraParams, SafetyLimits, WorldConfig

SQUARE = [(-100.0, -100.0), (100.0, -100.0), (100.0, 100.0), (-100.0, 100.0)]


@pytest.mark.parametrize(
    ("change", "why"),
    [
        ({"fence": [(0.0, 0.0), (50.0, 50.0)]}, "three vertices"),
        ({"fence": [(0.0, 0.0), (50.0, 50.0), (100.0, 100.0)]}, "square metres"),  # a line
        ({"fence": [(-100.0, -100.0), (100.0, 100.0), (100.0, -100.0), (-100.0, 100.0)]}, "cross"),
        ({"fence": [(math.nan, 0.0), (100.0, 0.0), (100.0, 100.0)]}, "finite"),
        ({"origin_lla": (95.0, 25.0, 20.0)}, "origin_lla"),
        ({"origin_lla": (60.3, 250.0, 20.0)}, "origin_lla"),
        ({"fense": SQUARE}, "Extra inputs"),
    ],
)
def test_a_site_that_cannot_hold_an_aircraft_is_refused(change: dict[str, Any], why: str) -> None:
    with pytest.raises(ValidationError, match=why):
        WorldConfig(**{"fence": SQUARE, **change})


def test_plain_and_concave_fences_are_accepted() -> None:
    WorldConfig(fence=SQUARE)
    WorldConfig(fence=[(0.0, 0.0), (200.0, 0.0), (200.0, 200.0), (100.0, 60.0), (0.0, 200.0)])  # an arrowhead


@pytest.mark.parametrize(
    "change",
    [
        {"min_separation": -5.0},
        {"min_separation": 0.0},
        {"min_altitude": 50.0, "max_altitude": 10.0},
        {"max_horizontal_speed": 0.0},
        {"max_horizontal_speed": 400.0},
        {"energy_reserve_fraction": 1.5},
        {"fence_margin": -1.0},
        {"link_loss_timeout_s": 0.0},
        {"min_seperation": 30.0},
        {"min_separation": math.nan},
    ],
)
def test_a_safety_limit_that_is_not_a_limit_is_refused(change: dict[str, Any]) -> None:
    with pytest.raises(ValidationError):
        SafetyLimits(**change)


@pytest.mark.parametrize("change", [{"hfov_deg": 400.0}, {"width": 0}, {"rate_hz": 0.0}])
def test_a_camera_that_cannot_exist_is_refused(change: dict[str, Any]) -> None:
    with pytest.raises(ValidationError):
        CameraParams(**change)
