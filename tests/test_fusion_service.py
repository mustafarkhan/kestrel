"""The fusion node as a live service, end to end over real TCP connections."""

from __future__ import annotations

import copy
import math
import time
from collections import defaultdict
from typing import Any

import numpy as np
import pytest

from kestrel.interop import wire
from kestrel.interop.sapient import SapientExporter, SapientOptions
from kestrel.rap.fusion import FusionNode, FusionOptions
from kestrel.rap.picture import PictureRun, flank_scenario, run_picture
from kestrel.rap.service import FusionService, ServiceOptions, stream_log, summary_line

EPOCH = SapientOptions(origin_lla=(0.0, 0.0, 0.0)).epoch_s
Message = dict[str, Any]


class FakeClock:
    def __init__(self) -> None:
        self.t = EPOCH

    def __call__(self) -> float:
        return self.t


def wait(cond: Any, timeout: float = 10.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(0.01)
    return bool(cond())


@pytest.fixture(scope="module")
def recorded() -> PictureRun:
    return run_picture(flank_scenario(5, spoofer=True), keep_messages=True)


def test_live_service_builds_the_same_picture_as_the_benchmark(recorded: PictureRun) -> None:
    horizon = 90.0
    sc = recorded.scenario
    assert sc.sensors is not None
    geo = recorded.fusion.geo
    msgs = [m for m in recorded.messages if geo.sim_time(m["timestamp"]) < horizon]
    clock = FakeClock()
    svc = FusionService(
        sc.world, sc.sensors.whitelist, sc.sensors.protected, ServiceOptions(port=0, tasking=False), clock
    ).start()
    clients: dict[str, wire.SapientClient] = {}
    by_second: dict[int, list[Message]] = defaultdict(list)
    for m in msgs:
        by_second[int(geo.sim_time(m["timestamp"]))].append(m)
    try:
        sent = 0
        picture: dict[str, Any] = {}
        for sec in range(int(horizon)):
            clock.t = EPOCH + sec + 0.999
            for m in by_second[sec]:
                node = m["node_id"]
                if node not in clients:
                    clients[node] = wire.SapientClient("127.0.0.1", svc.server.port).start()
                clients[node].send(m)
                sent += 1
            assert wait(lambda n=sent: svc.server.accepted + svc.server.rejected >= n), sec
            picture = svc.tick()
        assert svc.server.rejected == 0 and not svc.rejected
        assert {n["state"] for n in picture["nodes"]} == {"OK"} and len(picture["nodes"]) == len(clients)

        # the same fusion code on the same reports, fed directly: the reference
        ref = FusionService(
            sc.world, sc.sensors.whitelist, sc.sensors.protected, ServiceOptions(port=0), FakeClock()
        )
        for sec in range(int(horizon)):
            for m in by_second[sec]:
                ref.fusion.ingest(m)
            ref.fusion.step(sec + 0.999)
        want = ref.fusion.picture(horizon - 0.001)
        ref.server.close()
        got = picture["tracks"]
        assert abs(len(got) - len(want)) <= 1 and len(got) >= 4
        assert sorted(t["identity"] for t in got if t["remote_id"]) == sorted(
            t.identity for t in want if t.remote_id
        )
        assert "FRIEND" in {t["identity"] for t in got} and "HOSTILE" not in {t["identity"] for t in got}
        assert "tracks=" in summary_line(picture)

        # ---- what the service refuses
        late = dict(msgs[-1])  # a replayed report: its timestamp is in the past
        clock.t += 30.0
        clients[late["node_id"]].send(late)
        det = next(m for m in reversed(msgs) if "detection_report" in m)
        far = {**det, "timestamp": geo.timestamp(clock.t - EPOCH)}
        far["detection_report"] = {
            **det["detection_report"],
            "location": geo.location(np.array([90_000.0, 0.0, 50.0]), np.ones(3)),
        }
        clients[far["node_id"]].send(far)
        assert wait(lambda: svc.rejected["clock skew"] == 1 and svc.rejected["outside the site"] == 1)

        # ---- a silent sensor is shown as stale, and its tracks age out
        clock.t += 60.0
        picture = svc.tick()
        assert picture["tracks"] == []
        assert {n["state"] for n in picture["nodes"]} == {"STALE"}
    finally:
        for c in clients.values():
            c.close()
        svc.close()


def test_a_mobile_node_gets_traffic_and_tasks_and_never_itself(recorded: PictureRun) -> None:
    sc = recorded.scenario
    assert sc.sensors is not None
    clock = FakeClock()
    svc = FusionService(sc.world, sc.sensors.whitelist, sc.sensors.protected, ServiceOptions(port=0), clock)
    svc.start()
    kestrel = SapientExporter(SapientOptions(origin_lla=sc.world.origin_lla, seed=4))
    radar = SapientExporter(SapientOptions(origin_lla=sc.world.origin_lla, seed=5))
    from kestrel.rap.sensors import GroundNode

    sent_by_radar: list[Message] = []
    node = GroundNode(radar, "NODE_TYPE_RADAR", "test radar", "RADAR", sent_by_radar, tracking=True)
    kc = wire.SapientClient("127.0.0.1", svc.server.port).start()
    rc = wire.SapientClient("127.0.0.1", svc.server.port).start()
    kestrel.sink = kc.send
    seen = 0

    def flush() -> None:
        nonlocal seen
        for m in sent_by_radar[seen:]:
            rc.send(m)
        seen = len(sent_by_radar)

    inbox: list[Message] = []
    try:
        node.register(0.0)
        for k in range(40):
            t = 0.5 * k
            clock.t = EPOCH + t
            own = np.array([10.0 + 2.0 * t, 0.0, 25.0])  # Kestrel, which the radar also sees
            other = np.array([120.0 - 3.0 * t, 40.0, 40.0])  # an unknown aircraft inside the area
            kestrel.step(t, own, "observe", [], [], False)
            node.detect(
                t, "own-ship", own, np.array([2.0, 2.0, 3.0]), np.array([2.0, 0.0, 0.0]), np.ones(3), 0.6
            )
            node.detect(
                t, "unknown", other, np.array([2.0, 2.0, 3.0]), np.array([-3.0, 0.0, 0.0]), np.ones(3), 0.6
            )
            flush()
            total = len(kestrel.messages) + len(sent_by_radar)
            assert wait(lambda n=total: svc.server.accepted >= n)
            svc.tick()
            time.sleep(0.01)
            inbox += kc.poll()
        assert wait(lambda: bool(inbox.extend(kc.poll()) or any("task" in m for m in inbox)))
        traffic = [kestrel.traffic_estimate(m, -1) for m in inbox if "detection_report" in m]
        assert len(traffic) > 10 and all(est is not None for est in traffic)
        own_now = np.array([10.0 + 2.0 * 19.5, 0.0, 25.0])
        for est in traffic:
            assert est is not None and abs(est.pos[1] - 40.0) < 8.0, "only the other aircraft, never Kestrel"
        look = next(kestrel.look_at(m) for m in inbox if "task" in m)
        assert look is not None and float(np.linalg.norm(look[1][0:2] - own_now[0:2])) > 20.0
        own_track = [t for t in svc.tick()["tracks"] if t["own_node"]]
        assert len(own_track) == 1 and own_track[0]["identity"] == "FRIEND"
        assert all(not wire.violations(m) for m in inbox)
    finally:
        kc.close()
        rc.close()
        svc.close()


def test_a_recorded_log_can_be_streamed_to_a_live_fusion_node(recorded: PictureRun, tmp_path: Any) -> None:
    import json

    geo = recorded.fusion.geo
    sc = recorded.scenario
    assert sc.sensors is not None
    path = tmp_path / "log.jsonl"
    msgs = [m for m in recorded.messages if geo.sim_time(m["timestamp"]) < 6.0]
    path.write_text("\n".join(json.dumps(m) for m in msgs), encoding="utf-8")
    svc = FusionService(sc.world, sc.sensors.whitelist, sc.sensors.protected, ServiceOptions(port=0)).start()
    try:
        sent = stream_log(path, "127.0.0.1", svc.server.port, speed=20.0)
        assert sent == len(msgs)
        assert wait(lambda: svc.server.accepted == sent)
        assert not svc.rejected, "re-stamped to now, so nothing is dropped for its clock"
        assert svc.fusion.reports > 0
    finally:
        svc.close()


def test_only_an_operator_makes_a_track_hostile_and_displays_are_told(recorded: PictureRun) -> None:
    import socket
    import xml.etree.ElementTree as ET

    from kestrel.interop.cot import CotSender, events

    sc = recorded.scenario
    assert sc.sensors is not None
    geo = recorded.fusion.geo
    clock = FakeClock()
    svc = FusionService(sc.world, sc.sensors.whitelist, sc.sensors.protected, ServiceOptions(port=0), clock)
    try:
        for m in recorded.messages:
            if geo.sim_time(m["timestamp"]) < 60.0:
                svc.fusion.ingest(m)
        clock.t = EPOCH + 60.0
        picture = svc.tick()
        assert picture["tracks"] and all(t["identity"] != "HOSTILE" for t in picture["tracks"])
        suspect = next(t for t in picture["tracks"] if t["identity"] in ("UNKNOWN", "SUSPECT"))
        assert "commands:" in svc.operator_command("shoot it")
        assert "refused" in svc.operator_command("declare 99999 HOSTILE")
        assert "refused" in svc.operator_command(f"declare {suspect['id']} ENEMY")
        assert "declared HOSTILE" in svc.operator_command(f"declare {suspect['id']} hostile")
        picture = svc.tick()
        hostile = [t for t in picture["tracks"] if t["identity"] == "HOSTILE"]
        assert [t["id"] for t in hostile] == [suspect["id"]]
        assert picture["declared_by_operator"][0]["track"] == suspect["id"]

        # Cursor on Target: one well-formed event per track that is not a bird
        xml = events(picture)
        assert len(xml) == sum(t["class"] != "Bird" for t in picture["tracks"])
        by_uid = {ET.fromstring(x).get("uid"): ET.fromstring(x) for x in xml}
        ev = by_uid[f"KESTREL-{suspect['id']}"]
        assert str(ev.get("type")).startswith("a-h-A")
        point = ev.find("point")
        assert point is not None and abs(float(str(point.get("lat"))) - suspect["lat"]) < 1e-6
        friend = next(t for t in picture["tracks"] if t["identity"] == "FRIEND")
        assert str(by_uid[f"KESTREL-{friend['id']}"].get("type")).startswith("a-f-A")
        assert ev.get("stale") > ev.get("time")  # type: ignore[operator]

        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("127.0.0.1", 0))
        rx.settimeout(2.0)
        sender = CotSender("127.0.0.1", rx.getsockname()[1])
        sender.send(picture)
        assert ET.fromstring(rx.recv(65535)).tag == "event" and sender.sent == len(xml)
        sender.close()
        rx.close()
    finally:
        svc.server.close()


def _fused(recorded: PictureRun, msgs: list[Message], max_tracks: int = 2000) -> FusionNode:
    sc = recorded.scenario
    assert sc.sensors is not None
    geo = SapientExporter(SapientOptions(origin_lla=sc.world.origin_lla, seed=9))
    node = FusionNode(
        geo, sc.world, sc.sensors.whitelist, sc.sensors.protected, FusionOptions(max_tracks=max_tracks)
    )
    for sec in range(60):
        for m in msgs:
            if sec <= geo.sim_time(m["timestamp"]) < sec + 1:
                node.ingest(m)
        node.step(sec + 0.999)
    return node


def test_one_sensor_sending_nonsense_cannot_corrupt_the_picture(recorded: PictureRun) -> None:
    """Protobuf carries not-a-number and infinity like any other double. One such coordinate,
    fused, spreads through association to every track; so nothing unchecked reaches a track."""
    geo = recorded.fusion.geo
    msgs = [m for m in recorded.messages if geo.sim_time(m["timestamp"]) < 60.0]
    clean = _fused(recorded, msgs)
    assert len(clean.tracks) >= 4 and not clean.rejected

    nonsense = [
        (("location", "x"), math.nan),
        (("location", "y"), math.inf),
        (("location", "z"), -math.inf),
        (("location", "x_error"), math.nan),
        (("location", "y_error"), -4.0),
        (("location", "z_error"), 1e300),
        (("location", "x"), "north"),
        (("location", "coordinate_system"), "LOCATION_COORDINATE_SYSTEM_UNSPECIFIED"),
        (("classification",), [{"type": "UAV", "confidence": math.nan}]),
        (("enu_velocity",), {"east_rate": math.nan, "north_rate": 0.0}),
        (("enu_velocity",), {"east_rate": 9e9, "north_rate": 0.0}),
        (("enu_velocity",), {"north_rate": 1.0}),
    ]
    dirty: list[Message] = []
    injected = 0
    for m in msgs:
        dirty.append(m)
        if "detection_report" in m:  # every report is followed by a corrupted copy of itself
            path, value = nonsense[injected % len(nonsense)]
            det = copy.deepcopy(m["detection_report"])
            target = det
            for key in path[:-1]:
                target = target[key]
            target[path[-1]] = value
            dirty.append({**m, "detection_report": det})
            injected += 1
    got = _fused(recorded, dirty)
    assert sum(got.rejected.values()) == injected, got.rejected
    assert all(np.all(np.isfinite(tr.x)) and np.all(np.isfinite(tr.p)) for tr in got.tracks)
    assert len(got.tracks) == len(clean.tracks)
    for a, b in zip(got.tracks, clean.tracks, strict=True):
        assert np.allclose(a.x, b.x) and a.identity == b.identity

    # a flood of new objects stops at the limit instead of stalling the picture
    capped = _fused(recorded, msgs, max_tracks=3)
    assert len(capped.tracks) <= 3 and capped.rejected["track limit reached"] > 0
