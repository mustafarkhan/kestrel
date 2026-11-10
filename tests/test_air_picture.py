"""The recognised air picture: sensor blind spots, fusion, identification, tasking, replay."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from kestrel.config import (
    Config,
    RadarConfig,
    RemoteIdConfig,
    RfConfig,
    Scenario,
    SensorNetConfig,
    WorldConfig,
    load_scenario,
)
from kestrel.interop.sapient import SapientExporter, SapientOptions
from kestrel.mission.executive import Executive, InvestigateB
from kestrel.mission.parser import RuleParser
from kestrel.rap.fusion import FusionNode
from kestrel.rap.picture import flank_scenario, run_picture
from kestrel.rap.replay import fuse_file
from kestrel.rap.sensors import GroundNode, Radar, RemoteIdReceiver, RfNetwork, SensorNetwork
from kestrel.rap.truth import TruthObject

ROOT = Path(__file__).resolve().parents[1]
ORIGIN = (60.30, 25.05, 20.0)
WORLD = WorldConfig(
    fence=[(-800.0, -800.0), (800.0, -800.0), (800.0, 800.0), (-800.0, 800.0)], origin_lla=ORIGIN
)


def geo(seed: int = 0) -> SapientExporter:
    return SapientExporter(SapientOptions(origin_lla=ORIGIN, seed=seed, altitude_band_m=(0.0, 150.0)))


def drone(
    key: str, pos: tuple[float, float, float], vel: tuple[float, float, float] = (0, 0, 0), **kw: object
) -> TruthObject:
    base: dict[str, object] = dict(
        key=key,
        kind="uav",
        role="hostile",
        pos=np.array(pos, float),
        vel=np.array(vel, float),
        rcs_m2=0.01,
        radar_class_cue=0.7,
    )
    base.update(kw)
    return TruthObject(**base)  # type: ignore[arg-type]


# --------------------------------------------------------------------------- sensors


def test_radar_detection_falls_with_range_and_misses_low_targets_far_out() -> None:
    radar = Radar(
        RadarConfig(),
        GroundNode(geo(), "NODE_TYPE_RADAR", "r", "r", []),
        np.zeros(3),
        np.random.default_rng(0),
    )
    near, far = drone("a", (300, 0, 50)), drone("b", (1800, 0, 80))
    assert radar.p_detect(near) > 0.99 > radar.p_detect(far)
    assert radar.p_detect(drone("c", (600, 0, 12))) == 0.0  # low and beyond the clutter range
    assert radar.p_detect(drone("d", (300, 0, 12))) > 0.9  # low but close: seen
    big = drone("e", (1800, 0, 80), rcs_m2=0.1)
    assert radar.p_detect(big) > radar.p_detect(far)


def test_rf_network_cannot_hear_a_radio_silent_drone() -> None:
    out: list[dict[str, object]] = []
    rf = RfNetwork(
        RfConfig(nodes=[(-500, -500), (500, -500), (0, 500)]),
        GroundNode(geo(), "NODE_TYPE_PASSIVE_RF", "rf", "rf", out),
        np.zeros(3),
        np.random.default_rng(1),
    )
    silent = drone("s", (100, 100, 40), rf_link=False)
    loud = drone("l", (-100, 50, 40), rf_link=True, rf_protocol="FHSS")
    for t in range(10):
        rf.sense(float(t), [silent, loud], (0.0, 150.0))
    reports = [m["detection_report"] for m in out]
    assert reports, "the radio-controlled drone should be heard"
    assert len({r["object_id"] for r in reports}) == 1  # one emitter, one fingerprint
    assert all(r["track_info"][0]["value"] == "FHSS" for r in reports)  # type: ignore[index]


def test_remote_id_reports_what_the_drone_claims() -> None:
    out: list[dict[str, object]] = []
    g = geo()
    rid = RemoteIdReceiver(
        RemoteIdConfig(gnss_sigma_m=(0.01, 0.01)),
        GroundNode(g, "NODE_TYPE_PASSIVE_RF", "rid", "rid", out),
        np.zeros(3),
        np.random.default_rng(2),
    )
    spoofer = drone(
        "x", (100, 0, 40), remote_id="KST-FRIEND-0001", remote_id_offset=np.array([200.0, 0.0, 0.0])
    )
    rid.sense(0.0, [spoofer, drone("y", (0, 0, 40))])
    assert len(out) == 1  # the drone without Remote ID says nothing
    pos = g.to_enu(out[0]["detection_report"]["location"])  # type: ignore[index]
    assert np.allclose(pos, [300.0, 0.0, 40.0], atol=0.5)


# --------------------------------------------------------------------------- fusion


def _fuse(
    objects_at: list[tuple[float, list[TruthObject]]], cfg: SensorNetConfig, whitelist: list[str]
) -> FusionNode:
    g = geo(3)
    net = SensorNetwork(cfg, WORLD, g, 3, None, (0.0, 150.0))
    fu = FusionNode(g, WORLD, whitelist, cfg.protected)
    for t, objs in objects_at:
        for m in net.step(t, objs):
            fu.ingest(m)
        fu.step(t)
    return fu


def _moving(key: str, t: float, **kw: object) -> TruthObject:
    return drone(key, (-200 + 5 * t, 100, 50), (5, 0, 0), **kw)


def test_two_sensors_on_one_drone_make_one_friendly_track() -> None:
    cfg = SensorNetConfig(radar=RadarConfig(), remote_id=RemoteIdConfig(), whitelist=["KST-FRIEND-0001"])
    seq = [(float(t), [_moving("f", t, remote_id="KST-FRIEND-0001", role="friendly")]) for t in range(40)]
    fu = _fuse(seq, cfg, cfg.whitelist)
    pic = fu.picture(39.0)
    assert len(pic) == 1
    assert pic[0].identity == "FRIEND"
    assert {"NODE_TYPE_RADAR", "NODE_TYPE_PASSIVE_RF"} <= pic[0].node_types


def test_a_whitelisted_serial_broadcast_from_the_wrong_place_is_not_a_friend() -> None:
    cfg = SensorNetConfig(radar=RadarConfig(), remote_id=RemoteIdConfig(), whitelist=["KST-FRIEND-0001"])
    off = np.array([250.0, 0.0, 0.0])
    seq = [
        (float(t), [_moving("s", t, remote_id="KST-FRIEND-0001", remote_id_offset=off)]) for t in range(40)
    ]
    fu = _fuse(seq, cfg, cfg.whitelist)
    ids = {tr.identity for tr in fu.picture(39.0)}
    assert "FRIEND" not in ids
    assert "ASSUMED_FRIEND" in ids  # the phantom: a serial no independent sensor corroborates


def test_a_cloned_serial_is_never_shown_as_friend() -> None:
    cfg = SensorNetConfig(radar=RadarConfig(), remote_id=RemoteIdConfig(), whitelist=["KST-FRIEND-0001"])
    off = np.array([250.0, 300.0, 0.0])
    seq = [
        (
            float(t),
            [
                drone("f", (300, -300, 60), remote_id="KST-FRIEND-0001", role="friendly"),
                _moving("s", t, remote_id="KST-FRIEND-0001", remote_id_offset=off),
            ],
        )
        for t in range(40)
    ]
    fu = _fuse(seq, cfg, cfg.whitelist)
    by_id = {tr.identity for tr in fu.picture(39.0) if tr.remote_id == "KST-FRIEND-0001"}
    assert by_id == {"ASSUMED_FRIEND", "SUSPECT"}  # neither copy is trusted while both exist


def test_hostile_is_only_ever_a_human_decision() -> None:
    cfg = SensorNetConfig(
        radar=RadarConfig(), remote_id=None, rf=RfConfig(nodes=[(-500, -500), (500, -500), (0, 500)])
    )
    seq = [(float(t), [_moving("h", t)]) for t in range(40)]
    fu = _fuse(seq, cfg, [])
    assert all(tr.identity != "HOSTILE" for tr in fu.tracks)
    tr = fu.picture(39.0)[0]
    fu.declare(tr.ident, "HOSTILE")
    fu.step(40.0)
    assert fu.picture(40.0)[0].identity == "HOSTILE"
    with pytest.raises(ValueError):
        fu.declare(tr.ident, "ENEMY")


def test_one_sensor_cannot_talk_itself_into_certainty() -> None:
    """A radar's correlated micro-Doppler scores are credited as at most three looks."""
    cfg = SensorNetConfig(radar=RadarConfig(class_noise=0.0), remote_id=None)
    seq = [(float(t), [_moving("h", 0.0, radar_class_cue=0.7)]) for t in range(300)]
    fu = _fuse(seq, cfg, [])
    tr = fu.picture(299.0)[0]
    assert 0.5 < tr.p_uav < 0.8
    assert fu.class_of(tr) == "Unknown"


def test_rf_and_radar_together_settle_the_class() -> None:
    cfg = SensorNetConfig(
        radar=RadarConfig(), remote_id=None, rf=RfConfig(nodes=[(-500, -500), (500, -500), (0, 500)])
    )
    seq = [(float(t), [_moving("h", t, rf_link=True)]) for t in range(40)]
    fu = _fuse(seq, cfg, [])
    assert [fu.class_of(tr) for tr in fu.picture(39.0)] == ["UAV"]


def test_suspect_when_an_unidentified_drone_heads_for_a_protected_site() -> None:
    from kestrel.config import CircleSite

    cfg = SensorNetConfig(
        radar=RadarConfig(),
        remote_id=None,
        rf=RfConfig(nodes=[(-500, -500), (500, -500), (0, 500)]),
        protected={"depot": CircleSite(center=(200.0, 100.0), radius=50.0)},
    )
    seq = [(float(t), [_moving("h", t, rf_link=True)]) for t in range(40)]
    fu = _fuse(seq, cfg, [])
    assert fu.picture(39.0)[0].identity == "SUSPECT"


def test_fused_covariance_is_consistent() -> None:
    """Normalised estimation error squared of the fused position averages near 3 (3 dof)."""
    cfg = SensorNetConfig(radar=RadarConfig(), remote_id=None)
    g = geo(4)
    net = SensorNetwork(cfg, WORLD, g, 4, None, (0.0, 150.0))
    fu = FusionNode(g, WORLD)
    nees = []
    for k in range(400):
        t = 0.5 * k
        obj = _moving("h", t)
        for m in net.step(t, [obj]):
            fu.ingest(m)
        fu.step(t)
        pic = fu.picture(t)
        if t > 20 and pic:
            e = pic[0].pos - obj.pos
            nees.append(float(e @ np.linalg.solve(pic[0].p[0:3, 0:3], e)))
    assert 1.5 < float(np.mean(nees)) < 6.0


def test_duplicate_tracks_from_two_radars_are_merged() -> None:
    g = geo(5)
    out: list[dict[str, object]] = []
    r1 = Radar(
        RadarConfig(),
        GroundNode(g, "NODE_TYPE_RADAR", "r1", "r1", out),
        np.zeros(3),
        np.random.default_rng(5),
    )
    r2 = Radar(
        RadarConfig(xyz=(600.0, 0.0, 6.0)),
        GroundNode(g, "NODE_TYPE_RADAR", "r2", "r2", out),
        np.zeros(3),
        np.random.default_rng(6),
    )
    fu = FusionNode(g, WORLD)
    for n in (r1.node, r2.node):
        n.register(0.0)
    for t in range(60):
        r1.scan(float(t), [_moving("h", t)])
        r2.scan(float(t), [_moving("h", t)])
        for m in out:
            fu.ingest(m)
        out.clear()
        fu.step(float(t))
    assert len(fu.picture(59.0)) == 1


def test_most_needed_look_prefers_the_unresolved_track_nearest_harm() -> None:
    from kestrel.config import CircleSite

    cfg = SensorNetConfig(
        radar=RadarConfig(), remote_id=None, protected={"depot": CircleSite(center=(0.0, 0.0), radius=50.0)}
    )
    seq = [
        (float(t), [drone("far", (600, 600, 50)), drone("near", (150, 0, 50), radar_class_cue=0.6)])
        for t in range(20)
    ]
    fu = _fuse(seq, cfg, [])
    fu.camera_node = "kestrel-eo"
    tr = fu.most_needed_look(19.0)
    assert tr is not None and float(np.linalg.norm(tr.pos[0:2] - [150, 0])) < 30
    task = fu.look_at_task(19.0, tr)
    assert task["destination_id"] == "kestrel-eo"
    assert fu.most_needed_look(20.0) is not tr  # not re-tasked at once


# --------------------------------------------------------------------------- scenario, replay


def test_flank_picture_fused_beats_any_single_sensor() -> None:
    sc = flank_scenario(7, spoofer=False).model_copy(update={"max_duration_s": 240.0})
    fused = run_picture(sc).score.summary()
    for single in ({"radar"}, {"rf"}, {"acoustic"}):
        alone = run_picture(sc, single).score.summary()
        assert fused["drone_coverage"] >= alone["drone_coverage"]  # type: ignore[operator]
    assert fused["friend_identified"] and fused["friend_identified"] > 0.9  # type: ignore[operator]
    assert fused["hostile_as_friend_s"] == 0.0


def test_recorded_sapient_traffic_fuses_to_the_same_picture(tmp_path: Path) -> None:
    sc = flank_scenario(8, spoofer=False).model_copy(update={"max_duration_s": 120.0})
    run = run_picture(sc, keep_messages=True)
    src = tmp_path / "sapient.jsonl"
    src.write_text("\n".join(json.dumps(m) for m in run.messages))
    out = tmp_path / "rap.jsonl"
    n = fuse_file(src, out, sc.sensors.whitelist if sc.sensors else [])
    assert n == run.fusion.reports
    last = json.loads(out.read_text().splitlines()[-1])
    ids = {tr["identity"] for tr in last["tracks"]}
    assert "FRIEND" in ids
    assert all(-90 <= tr["lat"] <= 90 and tr["sigma_m"] >= 0 for tr in last["tracks"])


def test_example_picture_scenario_loads() -> None:
    sc = load_scenario(ROOT / "scenarios" / "airfield_picture.yaml")
    assert sc.sensors is not None and sc.sensors.whitelist == ["KST-FRIEND-0001"]
    roles = [i.role for i in sc.intruders]
    assert roles == ["hostile", "friendly"]


# --------------------------------------------------------------------------- tasking reaches Kestrel


def _look(g: SapientExporter, task_id: str, sigma: np.ndarray) -> dict[str, object]:
    loc = g.location(np.array([50.0, 80.0, 30.0]), sigma)
    return {"task": {"task_id": task_id, "command": {"look_at": {"location_list": {"locations": [loc]}}}}}


def test_look_at_with_altitude_lets_the_observer_close_in() -> None:
    g = geo()
    task = _look(g, "T1", np.array([3.0, 3.0, 4.0]))
    detail = g.look_at_detail(task)
    assert detail is not None and detail[2] is True
    column = _look(g, "T2", np.array([9.0, 9.0, 30.0]))
    detail = g.look_at_detail(column)
    assert detail is not None and detail[2] is False


def test_a_resent_tasking_updates_the_look_instead_of_queueing(cfg: Config, airfield: Scenario) -> None:
    rules = RuleParser(list(airfield.world.sites))
    plan = rules.parse("take off to 25 m, stand by at the hangar for 4 minutes, then return home")
    ex = Executive(plan, cfg, airfield.world)
    look = InvestigateB(np.array([0.0, 50.0]), "C2", 30.0, 60.0, 6.0, 3.0)
    ex.current = look
    ex.request_investigate(np.array([10.0, 60.0]), 1.0, "C2")
    assert np.allclose(look.xy, [10.0, 60.0]) and not ex.requests
    ex.request_investigate(np.array([500.0, 60.0]), 2.0, "C2")  # far: a new look, queued
    ex.request_investigate(np.array([520.0, 60.0]), 3.0, "C2")  # supersedes the queued one
    assert len(ex.requests) == 1 and math.isclose(float(ex.requests[0][1][0]), 520.0)
