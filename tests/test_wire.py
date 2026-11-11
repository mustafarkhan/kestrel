"""SAPIENT on the wire: encoding, mandatory fields, framing, and the TCP endpoints."""

from __future__ import annotations

import shutil
import socket
import struct
import subprocess
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from kestrel.interop import wire
from kestrel.interop.sapient import SapientExporter, SapientOptions
from kestrel.perception.tracker import TrackEstimate

ROOT = Path(__file__).resolve().parents[1]
Message = dict[str, Any]


def exporter(seed: int = 3) -> SapientExporter:
    return SapientExporter(SapientOptions(origin_lla=(60.3, 25.05, 20.0), seed=seed))


def node_messages(ex: SapientExporter, n: int = 3) -> list[Message]:
    track = TrackEstimate(
        1, np.array([40.0, 10.0, 30.0]), np.array([1.0, 0.0, 0.0]), np.eye(6), True, 9, 0.0, p_uav=0.9
    )
    for k in range(n):
        ex.step(float(k), np.array([0.0, 0.0, 25.0]), "observe", [track], [], False)
    return list(ex.messages)


def wait(cond: Callable[[], bool], timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(0.02)
    return cond()


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


# --------------------------------------------------------------------------- encoding


def test_every_message_survives_the_binary_round_trip() -> None:
    ex = exporter()
    msgs = node_messages(ex)
    assert {wire.content_kind(m) for m in msgs} == {"registration", "status_report", "detection_report"}
    for m in msgs:
        back = wire.decode(wire.encode(m))
        assert wire.content_kind(back) == wire.content_kind(m)
        assert back["node_id"] == m["node_id"]
        assert abs(ex.sim_time(back["timestamp"]) - ex.sim_time(m["timestamp"])) < 1e-6
        if "detection_report" in m:  # positions are doubles on the wire: centimetres survive
            a, b = m["detection_report"]["location"], back["detection_report"]["location"]
            assert abs(a["x"] - b["x"]) < 1e-6 and abs(a["y"] - b["y"]) < 1e-6


def test_messages_carry_the_mandatory_fields_of_the_standard() -> None:
    """Protobuf accepts a message without them; a SAPIENT receiver rejects it."""
    for m in node_messages(exporter()):
        assert wire.violations(m) == [], wire.content_kind(m)


def test_missing_mandatory_fields_are_named() -> None:
    reg = next(m for m in node_messages(exporter()) if "registration" in m)
    bad = {**reg, "registration": {k: v for k, v in reg["registration"].items() if k != "config_data"}}
    assert "registration.config_data" in wire.violations(bad)
    assert any("content" in v for v in wire.violations({"timestamp": reg["timestamp"], "node_id": "n"}))
    with pytest.raises(wire.WireError):
        wire.encode({"node_id": "n", "status_report": {"no_such_field": 1}})
    with pytest.raises(wire.WireError):
        wire.decode(b"\xff\xff\xff\xff\xff")


def test_the_shipped_descriptor_set_matches_the_vendored_schema(tmp_path: Path) -> None:
    pytest.importorskip("grpc_tools")
    import importlib.util

    from google.protobuf import descriptor_pb2

    spec = importlib.util.spec_from_file_location("gen", ROOT / "scripts" / "gen_sapient_pb.py")
    assert spec is not None and spec.loader is not None
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    gen.generate(tmp_path / "fresh.desc")

    def own_files(path: Path) -> dict[str, bytes]:
        fds = descriptor_pb2.FileDescriptorSet()
        fds.ParseFromString(path.read_bytes())
        return {f.name: f.SerializeToString(deterministic=True) for f in fds.file if "sapient" in f.name}

    assert own_files(gen.DESC) == own_files(tmp_path / "fresh.desc"), "run scripts/gen_sapient_pb.py"


# --------------------------------------------------------------------------- framing


@settings(max_examples=60, deadline=None)
@given(
    st.lists(st.binary(min_size=0, max_size=300), min_size=1, max_size=8),
    st.lists(st.integers(1, 64), min_size=1, max_size=20),
)
def test_frames_reassemble_from_any_chunking(payloads: list[bytes], cuts: list[int]) -> None:
    stream = b"".join(wire.frame(p) for p in payloads)
    reader = wire.FrameReader()
    out: list[bytes] = []
    i, k = 0, 0
    while i < len(stream):
        step = cuts[k % len(cuts)]
        out += reader.feed(stream[i : i + step])
        i, k = i + step, k + 1
    assert out == payloads


def test_an_oversized_frame_is_refused_before_it_is_allocated() -> None:
    reader = wire.FrameReader(max_frame=1000)
    with pytest.raises(wire.WireError):
        reader.feed(struct.pack("<I", 2_000_000_000))
    with pytest.raises(wire.WireError):
        wire.frame(b"x" * (wire.MAX_FRAME + 1))


# --------------------------------------------------------------------------- endpoints


class Harness:
    def __init__(self, port: int = 0, tls: Any = None) -> None:
        self.got: list[Message] = []
        self.server = wire.SapientServer("127.0.0.1", port, self.got.append, "fusion-node", tls=tls).start()

    def close(self) -> None:
        self.server.close()


def test_a_node_registers_reports_and_is_tasked_over_tcp() -> None:
    h = Harness()
    ex = exporter()
    client = wire.SapientClient("127.0.0.1", h.server.port).start()
    ex.sink = client.send
    try:
        msgs = node_messages(ex, n=4)
        assert wait(lambda: len(h.got) == len(msgs))
        assert [wire.content_kind(m) for m in h.got] == [wire.content_kind(m) for m in msgs]
        assert wire.content_kind(h.got[0]) == "registration"
        # the registration is acknowledged, and a task addressed to the node reaches it
        task = {
            "timestamp": ex.timestamp(5.0),
            "node_id": "fusion-node",
            "destination_id": ex.camera_node_id,
            "task": {
                "task_id": "T1",
                "control": "CONTROL_START",
                "command": {
                    "look_at": {"location_list": {"locations": [ex.location(np.array([5.0, 6.0, 30.0]))]}}
                },
            },
        }
        assert h.server.send(task)
        assert not h.server.send({**task, "destination_id": "nobody"})
        inbox: list[Message] = []
        assert wait(lambda: bool(inbox.extend(client.poll()) or len(inbox) >= 2))
        assert inbox[0]["registration_ack"]["acceptance"] is True
        look = ex.look_at(inbox[1])
        assert (
            look is not None and look[0] == "T1" and float(np.linalg.norm(look[1][0:2] - [5.0, 6.0])) < 0.05
        )
    finally:
        client.close()
        h.close()


def _raw(port: int) -> socket.socket:
    s = socket.create_connection(("127.0.0.1", port), timeout=2.0)
    s.settimeout(2.0)
    return s


def _read(sock: socket.socket) -> Message:
    reader = wire.FrameReader()
    while True:
        frames = reader.feed(sock.recv(65536))
        if frames:
            return wire.decode(frames[0])


def test_the_server_refuses_what_would_corrupt_the_picture() -> None:
    h = Harness()
    try:
        msgs = node_messages(exporter(1))
        reg = next(m for m in msgs if "registration" in m)
        status = next(m for m in msgs if "status_report" in m)
        det = next(m for m in msgs if "detection_report" in m)

        a = _raw(h.server.port)
        a.sendall(wire.frame(wire.encode(det)))  # before any registration: discarded
        a.sendall(wire.frame(wire.encode(reg)))
        assert _read(a)["registration_ack"]["acceptance"] is True
        a.sendall(wire.frame(wire.encode(status)))
        assert wait(lambda: len(h.got) == 2)

        b = _raw(h.server.port)  # a second connection cannot take the node id over ...
        b.sendall(wire.frame(wire.encode(reg)))
        ack = _read(b)["registration_ack"]
        assert ack["acceptance"] is False and "another connection" in ack["ack_response_reason"][0]
        b.sendall(wire.frame(wire.encode(det)))  # ... nor report in its name
        other = next(m for m in node_messages(exporter(2)) if "registration" in m)
        incomplete = {
            **other,
            "registration": {k: v for k, v in other["registration"].items() if k != "capabilities"},
        }
        b.sendall(wire.frame(wire.encode(incomplete)))  # a registration without mandatory fields
        ack = _read(b)["registration_ack"]
        assert ack["acceptance"] is False and "capabilities" in ack["ack_response_reason"][0]

        no_info = {
            **status,
            "status_report": {k: v for k, v in status["status_report"].items() if k != "info"},
        }
        a.sendall(wire.frame(wire.encode(no_info)))  # a report without mandatory fields: an Error back
        assert "status_report.info" in _read(a)["error"]["error_message"][0]

        c = _raw(h.server.port)  # garbage closes that connection only
        c.sendall(struct.pack("<I", 3) + b"\xff\xff\xff")
        d = _raw(h.server.port)
        d.sendall(struct.pack("<I", 2_000_000_000))  # so does an absurd length
        assert wait(lambda: c.recv(16) == b"" and d.recv(16) == b"")
        a.sendall(wire.frame(wire.encode(det)))
        assert wait(lambda: len(h.got) == 3), "the first node is still served"
        assert h.server.rejected >= 5
        for s in (a, b, c, d):
            s.close()
        assert wait(lambda: h.server.nodes() == []), "a closed connection frees its node id"
    finally:
        h.close()


def test_reports_wait_for_the_link_and_the_node_registers_again() -> None:
    port = free_port()
    ex = exporter()
    client = wire.SapientClient("127.0.0.1", port, reconnect_s=(0.05, 0.2)).start()
    ex.sink = client.send
    try:
        before = node_messages(ex, n=3)  # nothing is listening yet
        assert not client.connected and client.pending() == len(before) - 1  # the registration is held apart
        h = Harness(port)
        assert wait(lambda: len(h.got) == len(before))
        assert wire.content_kind(h.got[0]) == "registration"
        stamps = [m["timestamp"] for m in h.got[1:]]
        assert stamps == sorted(stamps), "held reports arrive in order with their measurement times"
        h.close()
        assert wait(lambda: not client.connected)
        ex.step(10.0, np.array([0.0, 0.0, 25.0]), "observe", [], [], False)
        h2 = Harness(port)
        assert wait(lambda: len(h2.got) >= 2)
        assert wire.content_kind(h2.got[0]) == "registration", "a new connection starts with a registration"
        assert client.connections == 2
        h2.close()
    finally:
        client.close()


def test_a_full_queue_drops_stale_status_first() -> None:
    ex = exporter()
    client = wire.SapientClient("127.0.0.1", free_port(), queue_max=6)  # never started: nothing drains it
    ex.sink = client.send
    node_messages(ex, n=8)
    assert client.pending() == 6 and client.dropped > 0
    kinds = [wire.content_kind(m) for m in client._out]
    assert kinds.count("detection_report") >= 4


@pytest.mark.skipif(shutil.which("openssl") is None, reason="needs the openssl command")
def test_mutual_tls_admits_only_certificates_from_our_ca(tmp_path: Path) -> None:
    def run(*args: str) -> None:
        subprocess.run(["openssl", *args], check=True, capture_output=True, cwd=tmp_path)

    def issue(ca: str, name: str) -> None:
        run(
            "req",
            "-newkey",
            "rsa:2048",
            "-nodes",
            "-keyout",
            f"{name}.key",
            "-out",
            f"{name}.csr",
            "-subj",
            f"/CN={name}",
        )
        sign = ["-CA", f"{ca}.crt", "-CAkey", f"{ca}.key", "-CAcreateserial"]
        run("x509", "-req", "-in", f"{name}.csr", *sign, "-out", f"{name}.crt", "-days", "2")

    for ca in ("ca", "rogue"):
        run("req", "-x509", "-newkey", "rsa:2048", "-nodes", "-keyout", f"{ca}.key", "-out", f"{ca}.crt",
            "-days", "2", "-subj", f"/CN={ca}")  # fmt: skip
    issue("ca", "fusion")
    issue("ca", "node")
    issue("rogue", "intruder")

    def ctx(ca: str, name: str, server: bool) -> Any:
        return wire.tls_context(
            str(tmp_path / f"{ca}.crt"), str(tmp_path / f"{name}.crt"), str(tmp_path / f"{name}.key"), server
        )

    h = Harness(tls=ctx("ca", "fusion", True))
    good = wire.SapientClient("127.0.0.1", h.server.port, tls=ctx("ca", "node", False)).start()
    bad = wire.SapientClient(
        "127.0.0.1", h.server.port, tls=ctx("rogue", "intruder", False), reconnect_s=(0.05, 0.1)
    ).start()
    plain = _raw(h.server.port)
    try:
        ex = exporter()
        ex.sink = good.send
        msgs = node_messages(ex, n=2)
        assert wait(lambda: len(h.got) == len(msgs))
        ex2 = exporter(8)
        ex2.sink = bad.send
        node_messages(ex2, n=2)
        plain.sendall(wire.frame(wire.encode(msgs[0])))
        time.sleep(0.6)
        assert len(h.got) == len(msgs), "neither a foreign certificate nor plain TCP gets a message in"
        assert h.server.nodes() == [ex.camera_node_id]
    finally:
        good.close()
        bad.close()
        plain.close()
        h.close()
