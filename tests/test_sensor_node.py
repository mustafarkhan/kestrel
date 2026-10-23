"""Sensor-node features: standby and cue investigation, C2 tasking over SAPIENT,
store-and-forward reporting, the link-loss policy, the inverse UTM and the sea surface."""

from __future__ import annotations

import math

import numpy as np
import pytest

from kestrel.config import Config, SafetyLimits, VehicleParams, WorldConfig
from kestrel.interop.sapient import SapientExporter, SapientOptions
from kestrel.interop.utm import enu_to_lla, from_utm, lla_to_enu, to_utm
from kestrel.mission.parser import RuleParser, parse_investigate
from kestrel.mission.schema import StandbyTask
from kestrel.mission.validator import validate
from kestrel.perception.camera import Camera
from kestrel.perception.detector import SkyTargetDetector
from kestrel.perception.render import SkyRenderer
from kestrel.safety.monitor import SafetyMonitor, SafetyRequest
from kestrel.types import FlightMode, VehicleState

ORIGIN = (60.1595, 24.9560, 0.0)
SITES = ["north_field", "tower", "lake", "hangar"]


def test_standby_and_cue_policy_parse() -> None:
    m = RuleParser(SITES).parse(
        "take off to 25 m, stand by at the hangar for 4 minutes and investigate any acoustic cues, "
        "then return home"
    )
    assert m.on_cue == "investigate"
    standby = [t for t in m.tasks if isinstance(t, StandbyTask)]
    assert standby and standby[0].at == "hangar" and standby[0].duration_s == 240.0


def test_investigate_commands() -> None:
    p = RuleParser(SITES)
    assert parse_investigate("investigate (40, 90)", p) == (40.0, 90.0)
    assert parse_investigate("check the tower", p) == "tower"
    assert parse_investigate("survey the lake", p) is None


def test_standby_mission_validates_with_an_investigation_allowance() -> None:
    from kestrel.config import load_scenario

    world = load_scenario("scenarios/airfield.yaml").world
    p = RuleParser(list(world.sites))
    quiet = validate(p.parse("take off to 25 m, stand by at the hangar for 4 minutes"), Config(), world)
    alert = validate(
        p.parse("take off to 25 m, stand by at the hangar for 4 minutes and investigate any cues"),
        Config(),
        world,
    )
    assert quiet.ok and alert.ok
    assert alert.est_energy_nominal_wh > quiet.est_energy_nominal_wh


def test_inverse_utm_round_trip() -> None:
    for lat, lon in [(60.1595, 24.956), (-35.363261, 149.16523), (54.6872, 25.2797)]:
        e, n, z, north = to_utm(lat, lon)
        la, lo = from_utm(e, n, z, north)
        assert la == pytest.approx(lat, abs=1e-8) and lo == pytest.approx(lon, abs=1e-8)
    lat, lon, alt = enu_to_lla(ORIGIN, 120.0, -80.0, 25.0)
    assert lla_to_enu(ORIGIN, lat, lon, alt) == pytest.approx((120.0, -80.0, 25.0), abs=1e-6)


def _exporter() -> SapientExporter:
    return SapientExporter(SapientOptions(origin_lla=ORIGIN, seed=1))


def test_registration_offers_look_at_tasking() -> None:
    ex = _exporter()
    ex.step(0.0, np.zeros(3), "standby", [], [], acoustic_fitted=False)
    reg = ex.messages[0]["registration"]
    task_defs = reg["mode_definition"][0]["task"]
    assert task_defs[0]["command"][0]["type"] == "COMMAND_TYPE_LOOK_AT"
    assert task_defs[0]["region_definition"]["region_type"] == ["REGION_TYPE_AREA_OF_INTEREST"]


def test_look_at_task_round_trip() -> None:
    ex = _exporter()
    corners = [ex.location(np.array([x, y, 30.0])) for x, y in [(30, 80), (50, 80), (50, 100), (30, 100)]]
    msg = {"node_id": "c2", "task": {"task_id": "T1", "command": {"look_at": {"locations": corners}}}}
    look = ex.look_at(msg)
    assert look is not None
    task_id, xy = look
    assert task_id == "T1"
    assert xy[0:2] == pytest.approx([40.0, 90.0], abs=0.05)
    ex.task_ack(1.0, task_id, True)
    assert ex.messages[-1]["task_ack"] == {"task_id": "T1", "task_status": "TASK_STATUS_ACCEPTED"}
    assert ex.look_at({"status_report": {}}) is None


def test_reports_are_stored_and_forwarded_through_a_jammed_link() -> None:
    from kestrel.perception.tracker import TrackEstimate

    ex = _exporter()
    tr = TrackEstimate(3, np.array([30.0, 60.0, 30.0]), np.zeros(3), np.eye(6), True, 10, 0.0, 0.9, 0.9)
    for k in range(40):
        t = 0.25 * k
        ex.set_link(t, not (2.0 <= t < 7.0))
        ex.step(t, np.zeros(3), "track", [tr], [], acoustic_fitted=False)
        if 2.0 <= t < 7.0:
            delivered_during = len(ex.messages)
    assert ex.late_deliveries > 0
    stamps = [m["timestamp"] for m in ex.messages]
    assert stamps == sorted(stamps)  # measurement times kept, delivered in order
    held = [m for m in ex.messages[delivered_during:] if "detection_report" in m]
    assert len(held) >= 9  # 2 Hz over the 5 s outage
    status_held = [
        m
        for m in ex.messages[delivered_during : delivered_during + ex.late_deliveries]
        if "status_report" in m
    ]
    assert len(status_held) == 1  # only the latest status is worth sending late


def _state(t: float) -> VehicleState:
    return VehicleState(
        t=t, pos=np.array([0.0, 0.0, 20.0]), vel=np.zeros(3), quat=np.array([1.0, 0, 0, 0]),
        omega=np.zeros(3), armed=True, on_ground=False, mode=FlightMode.GUIDED, battery_wh=70.0,
        battery_capacity_wh=77.0,
    )  # fmt: skip


@pytest.mark.parametrize(("action", "latched_at"), [("rtl", 5.1), ("continue", 120.1)])
def test_link_loss_policy(action: str, latched_at: float) -> None:
    lim = SafetyLimits(link_loss_action=action)  # type: ignore[arg-type]
    mon = SafetyMonitor(lim, VehicleParams(), WorldConfig())
    mon.check_vehicle(_state(0.0), True)
    first = None
    for k in range(1, 1300):
        t = 0.1 * k
        mon.check_vehicle(_state(t), False)
        if mon.latched is SafetyRequest.RTL and first is None:
            first = t
    assert first is not None and math.isclose(first, latched_at, abs_tol=0.2)


def test_sea_surface_adds_no_false_alarms() -> None:
    cfg = Config()
    cam = Camera(cfg.camera, cfg.vehicle.camera_offset)
    det = SkyTargetDetector(cam, cfg.detector)
    rng = np.random.default_rng(5)
    false = 0
    for s in range(25):
        ren = SkyRenderer(cam, seed=s, surface="sea")
        pose = cam.pose(np.array([0.0, 0.0, 25.0]), np.array([1.0, 0, 0, 0]), float(rng.uniform(-6.0, 8.0)))
        false += len(det.detect(ren.render(pose, [], cfg.camera.noise_sigma), pose))
    assert false == 0
