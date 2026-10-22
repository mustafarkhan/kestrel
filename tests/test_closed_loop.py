"""End-to-end simulations through the whole stack (camera pixels to autopilot setpoints)."""

from __future__ import annotations

import numpy as np
import pytest

from kestrel.config import Config, IntruderConfig, LinkLossEvent, Scenario
from kestrel.metrics import compute_metrics
from kestrel.runtime import run

pytestmark = pytest.mark.slow


def _quiet(sc: Scenario, **kw: object) -> Scenario:
    return sc.model_copy(update={"intruders": [], **kw})


def test_short_mission_without_perception(cfg: Config, airfield: Scenario) -> None:
    sc = _quiet(airfield, max_duration_s=200.0)
    res = run(
        "take off to 15 m, fly to the hangar, hover for 5 s, then land", cfg, sc, perception_enabled=False
    )
    m = compute_metrics(res)
    assert res.completed and res.landed, m
    hangar = np.array([-30.0, 10.0])
    assert np.linalg.norm(res.log.arrays()["pos"][-1, 0:2] - hangar) < 1.5
    assert abs(m["max_altitude_m"] - 15.0) < 1.5


def test_operator_abort_returns_home(cfg: Config, airfield: Scenario) -> None:
    sc = _quiet(airfield, max_duration_s=300.0)
    res = run(
        "take off to 20 m and orbit the tower 3 times", cfg, sc, operator_commands=[(40.0, "abort")],
        perception_enabled=False,
    )  # fmt: skip
    kinds = [(e.kind, e.detail) for e in res.events]
    assert any(k == "abort" and "operator" in d for k, d in kinds)
    assert res.landed and not res.completed
    assert compute_metrics(res)["final_distance_from_home_m"] < 2.0


def test_link_loss_triggers_return(cfg: Config, airfield: Scenario) -> None:
    sc = _quiet(airfield, max_duration_s=300.0, link_loss=[LinkLossEvent(start_s=30.0, duration_s=60.0)])
    res = run("take off to 20 m and survey the north field", cfg, sc, perception_enabled=False)
    assert any("ground link lost" in r for r in res.safety_reasons)
    assert res.landed and compute_metrics(res)["final_distance_from_home_m"] < 2.0


def test_rejected_plan_never_flies(cfg: Config, airfield: Scenario) -> None:
    res = run("orbit the tower at 90 m", cfg, airfield)
    assert res.report is not None and not res.report.ok
    assert not res.flown


def test_detect_track_and_keep_separation(cfg: Config, airfield: Scenario) -> None:
    sc = airfield.model_copy(
        update={
            "max_duration_s": 220.0,
            "intruders": [IntruderConfig(pattern="orbit", center=(35, 85, 30), size=(20, 20), speed=3.0)],
        }
    )
    res = run(
        "take off to 25 m, search the north field for 60 seconds, and if you see a drone follow it "
        "from 30 m for 40 seconds, then return home",
        cfg,
        sc,
    )
    m = compute_metrics(res)
    assert any(e.kind == "interrupt" for e in res.events), m["events"]
    assert m["min_separation_m"] >= cfg.safety.min_separation, m
    assert m["target_in_fov_fraction"] > 0.8, m
    assert m["track_error_median_m"] < 8.0, m
    assert res.landed
