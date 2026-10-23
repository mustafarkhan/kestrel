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


def _orbiting_intruder(sc: Scenario, **kw: object) -> Scenario:
    intruder = IntruderConfig(pattern="orbit", center=(35, 85, 30), size=(20, 20), speed=3.0)
    return sc.model_copy(update={"intruders": [intruder], **kw})


STANDBY = (
    "take off to 25 m, stand by at the hangar for 2 minutes and investigate any acoustic cues, "
    "then return home"
)
STANDBY_FOLLOW = (
    "take off to 25 m, stand by at the hangar for 2 minutes and investigate any acoustic cues, "
    "and if you see a drone, follow it from 30 m for 60 seconds, then return home"
)


def test_acoustic_cue_is_investigated_and_confirmed(cfg: Config, airfield: Scenario) -> None:
    acoustic = cfg.model_copy(update={"acoustic": cfg.acoustic.model_copy(update={"enabled": True})})
    res = run(STANDBY, acoustic, _orbiting_intruder(airfield, max_duration_s=260.0))
    m = compute_metrics(res)
    kinds = [e.kind for e in res.events]
    assert "investigate" in kinds and "confirmed" in kinds, m["events"]
    assert m["cue_to_confirm_s"] is not None and m["cue_to_confirm_s"] < 60.0, m
    assert m["min_separation_m"] >= cfg.safety.min_separation, m
    assert res.landed


def test_jammed_sensor_node_keeps_working_and_reports_late(cfg: Config, airfield: Scenario) -> None:
    from kestrel.interop.sapient import SapientExporter, SapientOptions

    safety = cfg.safety.model_copy(update={"link_loss_action": "continue"})
    c = cfg.model_copy(
        update={"safety": safety, "acoustic": cfg.acoustic.model_copy(update={"enabled": True})}
    )
    sc = _orbiting_intruder(
        airfield, max_duration_s=260.0, link_loss=[LinkLossEvent(start_s=40.0, duration_s=50.0)]
    )
    sapient = SapientExporter(SapientOptions(origin_lla=sc.world.origin_lla))
    res = run(STANDBY_FOLLOW, c, sc, sapient=sapient)
    m = compute_metrics(res)
    assert not any(e.kind == "abort" for e in res.events), m["events"]
    assert res.reports_buffered > 0
    assert m["observed_while_jammed_s"] > 10.0, m
    assert res.landed and m["min_separation_m"] >= cfg.safety.min_separation, m


def test_two_observers_triangulate_and_keep_apart(cfg: Config, airfield: Scenario) -> None:
    from kestrel.team import run_team

    sc = _orbiting_intruder(airfield, max_duration_s=300.0)
    mission = "take off to 25 m, survey the north field at 25 m with 30 m spacing, then return home"
    res = run_team(mission, cfg, sc, 2)
    alone = compute_metrics(run(mission, cfg, sc))
    m = compute_metrics(res)
    assert res.completed and res.landed, m["events"]
    assert m["team_min_separation_m"] >= cfg.safety.min_separation, m
    assert m["min_separation_m"] >= cfg.safety.min_separation, m
    # two lines of sight fix range far better than one camera's size-based guess
    assert m["track_error_all_median_m"] < 0.8 * alone["track_error_all_median_m"], (m, alone)


def test_two_observers_do_not_both_follow_one_drone(cfg: Config, airfield: Scenario) -> None:
    from kestrel.team import run_team

    sc = _orbiting_intruder(airfield, max_duration_s=200.0)
    res = run_team(
        "take off to 25 m, stand by at the hangar for 90 seconds, and if you see a drone, "
        "follow it from 30 m for 40 seconds, then return home",
        cfg,
        sc,
        2,
    )
    followers = {e.detail.split("]")[0] for e in res.events if e.kind == "interrupt"}
    targets = [e.detail.rsplit("track ", 1)[1] for e in res.events if e.kind == "interrupt"]
    assert len(targets) == len(set(targets)), [e.detail for e in res.events if e.kind == "interrupt"]
    assert followers, "nobody followed the drone"
