"""Every SAPIENT message Kestrel writes parses against Dstl's published protobuf schema.

The schema (BSI Flex 335 v1.0 and v2.0) is vendored under ``third_party/sapient-protos`` and
compiled here with ``grpc_tools``; ``json_format.ParseDict`` rejects unknown fields, wrong
enum names and wrong types, so a passing test means the JSON converts to the binary wire
format as is. Skipped when ``grpcio-tools`` is not installed.
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np
import pytest

from kestrel.interop.sapient import SapientExporter, SapientOptions
from kestrel.perception.acoustic import AcousticFix
from kestrel.perception.tracker import TrackEstimate
from kestrel.rap.picture import flank_scenario, run_picture

PROTOS = Path(__file__).resolve().parents[1] / "third_party" / "sapient-protos"
grpc_tools = pytest.importorskip("grpc_tools")
json_format = pytest.importorskip("google.protobuf.json_format")


@pytest.fixture(scope="module")
def schemas(tmp_path_factory: pytest.TempPathFactory) -> dict[str, ModuleType]:
    from grpc_tools import protoc

    out = tmp_path_factory.mktemp("pb")
    files = sorted(str(p.relative_to(PROTOS)) for p in PROTOS.rglob("*.proto"))
    include = str(Path(grpc_tools.__file__).parent / "_proto")
    assert protoc.main(["protoc", f"-I{PROTOS}", f"-I{include}", f"--python_out={out}", *files]) == 0
    for d in [out / "sapient_msg", *(out / "sapient_msg").iterdir()]:
        if d.is_dir():
            (d / "__init__.py").touch()
    sys.path.insert(0, str(out))
    try:
        return {
            v: importlib.import_module(f"sapient_msg.bsi_flex_335_{v}_0.sapient_message_pb2")
            for v in ("v1", "v2")
        }
    finally:
        sys.path.remove(str(out))


def _parse(schema: ModuleType, msg: dict[str, Any]) -> None:
    json_format.ParseDict(msg, schema.SapientMessage())


def _kinds(msgs: list[dict[str, Any]]) -> set[str]:
    return {next(k for k in m if k not in ("timestamp", "node_id", "destination_id")) for m in msgs}


def _kestrel_messages() -> list[dict[str, Any]]:
    ex = SapientExporter(SapientOptions(origin_lla=(60.3, 25.05, 20.0), seed=9))
    cov = np.diag([4.0, 9.0, 1.0, 1.0, 1.0, 1.0])
    track = TrackEstimate(
        3, np.array([30.0, 60.0, 30.0]), np.array([1.0, -2.0, 0.0]), cov, True, 10, 0.0, 0.9, 0.8
    )
    fix = AcousticFix(1.0, np.array([20.0, 30.0]), np.diag([16.0, 25.0]), 3)
    for k in range(10):
        ex.step(0.5 * k, np.array([0.0, 0.0, 25.0]), "standby", [track], [fix] if k == 2 else [], True)
    ex.task_ack(5.0, "01J0000000000000000000000T", True)
    ex.task_ack(5.5, "01J0000000000000000000000U", False, "mission aborted")
    return ex.messages


def test_kestrel_node_messages_parse_against_v2(schemas: dict[str, ModuleType]) -> None:
    msgs = _kestrel_messages()
    assert _kinds(msgs) == {"registration", "status_report", "detection_report", "task_ack"}
    for m in msgs:
        _parse(schemas["v2"], m)


def _v1_differs(m: dict[str, Any]) -> bool:
    """Messages whose v2.0 layout is not valid v1.0, by design of the newer standard.

    Registration: ``detection_definition`` became a list and ``task`` a single definition.
    TaskAck: ``reason`` became a list. DetectionReport: the object ``id`` field is new.
    """
    return "registration" in m or "reason" in m.get("task_ack", {}) or "id" in m.get("detection_report", {})


def test_status_detection_and_ack_messages_also_parse_against_v1(schemas: dict[str, ModuleType]) -> None:
    msgs = [m for m in _kestrel_messages() if not _v1_differs(m)]
    assert _kinds(msgs) == {"status_report", "detection_report", "task_ack"}
    for m in msgs:
        _parse(schemas["v1"], m)


def test_ground_sensors_and_fusion_tasks_parse(schemas: dict[str, ModuleType]) -> None:
    sc = flank_scenario(11).model_copy(update={"max_duration_s": 30.0})
    run = run_picture(sc, keep_messages=True)
    fusion = run.fusion
    fusion.camera_node = "6f1c0d7e-2a8b-4c55-9b7e-0d2f3c4a5b6c"
    tracks = fusion.picture(30.0)
    assert tracks
    msgs = [*run.messages, fusion.look_at_task(30.0, tracks[0])]
    assert _kinds(msgs) == {"registration", "status_report", "detection_report", "task"}
    for m in msgs:
        _parse(schemas["v2"], m)
        if not _v1_differs(m):
            _parse(schemas["v1"], m)
