"""The real flight loop on a bench: camera frames, SAPIENT link, recorder and fault handling.

``BenchVehicle`` stands in for the autopilot and the sky; everything else is the code that
flies: ``live.run_live``, the detector and tracker on camera frames, a real TCP link to a
real fusion service. Stepped deterministically (no sleeping), so these run in seconds.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from kestrel.config import Config, Scenario, load_config, load_scenario
from kestrel.interop import wire
from kestrel.interop.sapient import SapientExporter, SapientOptions
from kestrel.live import LiveOptions, LiveResult, run_live
from kestrel.mission.parser import RuleParser
from kestrel.rap.service import FusionService, ServiceOptions
from kestrel.vehicle.bench_vehicle import BenchVehicle

ROOT = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.slow
MISSION = "take off to 20 m, hover for 25 seconds, then land"


@pytest.fixture()
def cfg() -> Config:
    return load_config(None)


@pytest.fixture()
def scenario() -> Scenario:
    return load_scenario(ROOT / "scenarios" / "airfield.yaml")


def bench(cfg: Config, sc: Scenario) -> BenchVehicle:
    return BenchVehicle(cfg, sc, step_dt=1.0 / 20.0)


def fly(cfg: Config, sc: Scenario, veh: BenchVehicle, mission: str = MISSION, **kw: Any) -> LiveResult:
    plan = RuleParser(list(sc.world.sites)).parse(mission)
    opts = LiveOptions(clock=veh.clock, realtime=False, max_duration_s=240.0, **kw)
    return run_live(plan, cfg, sc.world, veh, options=opts)


def kinds(res: LiveResult) -> str:
    return " | ".join(f"{e.kind}: {e.detail}" for e in res.events)


def wait(cond: Any, timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(0.01)
    return bool(cond())


def test_a_flight_with_camera_fusion_link_and_recorder(
    cfg: Config, scenario: Scenario, tmp_path: Path
) -> None:
    veh = bench(cfg, scenario)
    epoch = time.time()
    svc = FusionService(
        scenario.world, opts=ServiceOptions(port=0, tasking=False), clock=lambda: epoch + veh.world.t
    ).start()
    client = wire.SapientClient("127.0.0.1", svc.server.port).start()
    try:
        assert wait(lambda: client.connected)
        res = fly(cfg, scenario, veh, frames=veh.camera_source(), sapient=client, log_dir=tmp_path / "flight")
        assert res.completed, kinds(res)
        assert res.frames > 150, "the camera path ran at about 10 Hz for the whole flight"
        assert wait(lambda: client.pending() == 0 and svc.server.accepted > 20)
        picture = svc.tick()
        node = picture["nodes"][0]
        assert node["type"] == "NODE_TYPE_CAMERA" and svc.server.rejected == 0 and not svc.rejected
        assert svc.fusion.mobile_nodes and svc.fusion.camera_node is not None
        # the recorder: metadata, every event, and a state record about five times a second
        lines = [json.loads(x) for x in (tmp_path / "flight" / "flight.jsonl").read_text().splitlines()]
        events = [x for x in lines if x["k"] == "event"]
        ticks = [x for x in lines if x["k"] == "tick"]
        assert len(events) == len(res.events) and len(ticks) > 4 * (res.times[-1] - res.times[0]) * 0.9
        assert {"pos", "sp", "behaviour", "interventions", "link", "battery_wh"} <= set(ticks[5])
        meta = json.loads((tmp_path / "flight" / "meta.json").read_text())
        assert meta["camera"] and meta["sapient"] and meta["mission"]
    finally:
        client.close()
        svc.close()


def test_tracks_from_camera_frames_reach_the_fusion_node(cfg: Config, scenario: Scenario) -> None:
    """A drone crossing in front of the hovering observer is detected in the frames, tracked,
    and reported over the link, where it becomes a picture track."""
    from kestrel.config import IntruderConfig

    ahead = IntruderConfig(pattern="lemniscate", center=(45.0, 0.0, 30.0), size=(14.0, 10.0), speed=3.0)
    scenario = scenario.model_copy(update={"intruders": [ahead]})
    veh = bench(cfg, scenario)
    epoch = time.time()
    svc = FusionService(
        scenario.world, opts=ServiceOptions(port=0, tasking=False), clock=lambda: epoch + veh.world.t
    ).start()
    client = wire.SapientClient("127.0.0.1", svc.server.port).start()
    try:
        mission = "take off to 25 m, hover for 40 seconds, then land"
        res = fly(cfg, scenario, veh, mission, frames=veh.camera_source(), sapient=client)
        assert res.tracks_reported > 0, kinds(res)
        assert wait(lambda: client.pending() == 0)
        assert wait(lambda: svc.fusion.reports > 0), "detection reports arrived and were fused"
    finally:
        client.close()
        svc.close()


def test_a_pilot_taking_control_ends_the_mission_loop(cfg: Config, scenario: Scenario) -> None:
    veh = bench(cfg, scenario)
    state = veh.state

    def state_then_takeover() -> Any:
        st = state()
        if st.t > 12.0:
            veh.pilot = True
        return st

    veh.state = state_then_takeover  # type: ignore[method-assign]
    res = fly(cfg, scenario, veh)
    assert res.pilot_took_control and not res.completed
    assert "handover" in kinds(res) and "LOITER" in kinds(res)
    sent = veh.setpoints
    assert res.times[-1] < 13.0 and veh.land_requests == 0, "it stops at once and commands nothing more"
    assert veh.setpoints == sent


def test_a_silent_autopilot_gets_no_setpoints_then_one_landing_request(
    cfg: Config, scenario: Scenario
) -> None:
    veh = bench(cfg, scenario)
    state = veh.state

    def state_then_silence() -> Any:
        st = state()
        if st.t > 12.0:
            veh.silent = True
        return st

    veh.state = state_then_silence  # type: ignore[method-assign]
    plan = RuleParser(list(scenario.world.sites)).parse(MISSION)
    opts = LiveOptions(clock=veh.clock, realtime=False, max_duration_s=40.0, telemetry_land_s=5.0)
    res = run_live(plan, cfg, scenario.world, veh, options=opts)
    assert "no position from the autopilot" in kinds(res) and "landing requested" in kinds(res)
    assert veh.land_requests == 1 and not res.completed
    assert veh.setpoints < 12.5 * 20, "nothing was sent after the autopilot went quiet"


def test_a_dead_camera_is_a_failed_sensor_not_an_empty_sky(cfg: Config, scenario: Scenario) -> None:
    veh = bench(cfg, scenario)
    src = veh.camera_source()
    render = src.render
    src.render = lambda now: None if veh.world.t > 10.0 else render(now)
    status: list[dict[str, Any]] = []
    client = wire.SapientClient("127.0.0.1", 9)  # never started: we only read what it would send
    client.send = lambda m: status.append(m)  # type: ignore[method-assign]
    client._connected = True
    res = fly(cfg, scenario, veh, frames=src, sapient=client)
    assert "camera stopped delivering frames" in kinds(res) and not res.completed
    systems = [m["status_report"]["system"] for m in status if "status_report" in m]
    assert "SYSTEM_ERROR" in systems and systems[-1] == "SYSTEM_GOODBYE"
    assert all(not wire.violations(m) for m in status)
    assert veh.world.quad.on_ground, "it came home and landed"


def test_frames_of_the_wrong_size_are_refused(cfg: Config, scenario: Scenario) -> None:
    from kestrel.perception.source import CallableSource

    veh = bench(cfg, scenario)
    wrong = CallableSource(lambda now: np.zeros((240, 320, 3), np.uint8), 10.0, clock=veh.clock)
    with pytest.raises(ValueError, match="bearings would be wrong"):
        fly(cfg, scenario, veh, frames=wrong)


def test_losing_the_fusion_link_brings_it_home(cfg: Config, scenario: Scenario) -> None:
    veh = bench(cfg, scenario)
    client = wire.SapientClient("127.0.0.1", 9)  # never connects
    res = fly(cfg, scenario, veh, sapient=client)
    assert "ground link lost" in kinds(res) and not res.completed
    assert client.pending() > 0, "its reports are waiting for the link"


def _task(
    ex: SapientExporter, task_id: str, xy: tuple[float, float], control: str = "CONTROL_START"
) -> dict[str, Any]:
    loc = ex.location(np.array([xy[0], xy[1], 30.0]), np.array([3.0, 3.0, 4.0]))
    return {
        "timestamp": ex.timestamp(0.0),
        "node_id": "fusion",
        "task": {
            "task_id": task_id,
            "control": control,
            "command": {"look_at": {"location_list": {"locations": [loc]}}},
        },
    }


def test_tasks_and_traffic_from_the_fusion_node(cfg: Config, scenario: Scenario) -> None:
    veh = bench(cfg, scenario)
    geo = SapientExporter(SapientOptions(origin_lla=scenario.world.origin_lla))
    sent: list[dict[str, Any]] = []
    client = wire.SapientClient("127.0.0.1", 9)
    client.send = lambda m: sent.append(m)  # type: ignore[method-assign]
    client._connected = True
    fed = False

    def poll() -> list[dict[str, Any]]:
        nonlocal fed
        if fed or veh.world.t < 9.0:
            return []
        fed = True
        far = geo.location(np.array([60.0, 0.0, 22.0]), np.array([3.0, 3.0, 4.0]))
        stamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        report = {"timestamp": stamp, "node_id": "fusion", "detection_report": {
            "report_id": "R", "object_id": "O", "location": far,
            "enu_velocity": {"east_rate": 0.0, "north_rate": 0.0}}}  # fmt: skip
        column = {**report, "detection_report": {**report["detection_report"], "location": {
            k: v for k, v in far.items() if k != "z_error"}}}  # fmt: skip
        return [
            _task(geo, "inside", (40.0, 30.0)),
            _task(geo, "outside", (5000.0, 0.0)),
            _task(geo, "stop", (40.0, 30.0), "CONTROL_STOP"),
            {**_task(geo, "not-mine", (40.0, 30.0)), "destination_id": "someone-else"},
            report,
            column,
        ]

    client.poll = poll  # type: ignore[method-assign]
    res = fly(cfg, scenario, veh, sapient=client)
    acks = {m["task_ack"]["task_id"]: m["task_ack"] for m in sent if "task_ack" in m}
    assert acks["inside"]["task_status"] == "TASK_STATUS_ACCEPTED"
    assert acks["outside"]["task_status"] == "TASK_STATUS_REJECTED"
    assert "outside the operating area" in acks["outside"]["reason"][0]
    assert acks["stop"]["task_status"] == "TASK_STATUS_REJECTED" and "not-mine" not in acks
    assert res.tasks_accepted == 1 and "C2 tasking (SAPIENT): investigate (40, 30)" in kinds(res)
    assert res.traffic_received == 1, "the 3-D report is traffic; the column without height is not"
    assert all(not wire.violations(m) for m in sent)


def test_the_site_frame_follows_the_vehicle_not_the_other_way_round(cfg: Config, scenario: Scenario) -> None:
    from kestrel.interop.utm import enu_to_lla

    veh = bench(cfg, scenario)
    veh.global_lla = enu_to_lla(scenario.world.origin_lla, 40.0, -30.0, 0.0)  # parked 50 m from the origin
    mission = "take off to 15 m, hover for 5 seconds, then return home"
    res = fly(cfg, scenario, veh, mission, align_site=True)
    assert res.completed, kinds(res)
    path = np.array(res.positions)
    assert float(np.linalg.norm(path[0, 0:2] - [40.0, -30.0])) < 1.5, "positions are in the site's frame"
    assert float(np.linalg.norm(path[-1, 0:2] - [40.0, -30.0])) < 2.5, "home is where it took off"

    lost = bench(cfg, scenario)
    lost.global_lla = enu_to_lla(scenario.world.origin_lla, 4000.0, 0.0, 0.0)  # the wrong site file
    with pytest.raises(ValueError, match="leaves the geofence|not inside the site's fence"):
        fly(cfg, scenario, lost, mission, align_site=True)
