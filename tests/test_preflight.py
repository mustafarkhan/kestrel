"""Pre-flight checks, and the command line end to end on the bench."""

from __future__ import annotations

import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, ClassVar

import numpy as np
import pytest

from kestrel.config import load_config, load_scenario
from kestrel.interop.utm import enu_to_lla
from kestrel.mission.parser import RuleParser
from kestrel.perception.source import CallableSource
from kestrel.preflight import FAIL, PASS, WARN, Check, cleared, preflight
from kestrel.vehicle.bench_vehicle import BenchVehicle

ROOT = Path(__file__).resolve().parents[1]
MISSION = "take off to 15 m, hover for 5 seconds, then land"


class Autopilot(BenchVehicle):
    """The bench vehicle with the extra things a real autopilot reports."""

    fix, sats, hb = 3, 12, 0.3
    params: ClassVar[dict[str, float | None]] = {
        "FENCE_ENABLE": 1.0, "FS_THR_ENABLE": 1.0, "BATT_FS_LOW_ACT": 2.0, "FS_EKF_ACTION": 1.0
    }  # fmt: skip

    def gps(self) -> tuple[int, int]:
        return self.fix, self.sats

    def heartbeat_age_s(self) -> float:
        return self.hb

    def param(self, name: str) -> float | None:
        return self.params.get(name)


def setup(**kw: Any) -> tuple[Any, Any, Any, Autopilot]:
    cfg = load_config(None)
    sc = load_scenario(ROOT / "scenarios" / "airfield.yaml")
    veh = Autopilot(cfg, sc, step_dt=0.05)
    veh.global_lla = enu_to_lla(sc.world.origin_lla, 10.0, 5.0, 0.0)
    for k, v in kw.items():
        setattr(veh, k, v)
    return cfg, sc, RuleParser(list(sc.world.sites)).parse(MISSION), veh


def by_name(checks: list[Check]) -> dict[str, Check]:
    return {c.name: c for c in checks}


def test_a_ready_aircraft_is_cleared(tmp_path: Path) -> None:
    cfg, sc, mission, veh = setup()
    frames = CallableSource(lambda now: np.full((360, 480, 3), 120, np.uint8), 30.0)
    checks = preflight(mission, cfg, sc.world, veh, frames, None, True, tmp_path, camera_wait_s=0.5)
    assert cleared(checks), [c.line() for c in checks]
    got = by_name(checks)
    assert {got[n].status for n in ("autopilot", "navigation", "GNSS", "site", "mission", "camera")} == {PASS}
    assert "10 m east, 5 m north" in got["site"].detail and got["FENCE_ENABLE"].status == PASS


def test_each_way_of_not_being_ready_is_caught(tmp_path: Path) -> None:
    cfg, sc, mission, veh = setup(fix=1, sats=4, hb=9.0)
    veh.global_lla = enu_to_lla(sc.world.origin_lla, 900.0, 0.0, 0.0)  # the wrong site file
    veh.params = {**veh.params, "FENCE_ENABLE": 0.0, "FS_EKF_ACTION": None}
    black = CallableSource(lambda now: np.zeros((360, 480, 3), np.uint8), 30.0)
    got = by_name(preflight(mission, cfg, sc.world, veh, black, None, True, tmp_path, camera_wait_s=0.3))
    assert got["autopilot"].status == FAIL and got["GNSS"].status == FAIL
    assert got["site"].status == FAIL and "OUTSIDE the fence" in got["site"].detail
    assert got["camera"].status == FAIL and "black" in got["camera"].detail
    assert got["FENCE_ENABLE"].status == WARN and "OFF" in got["FENCE_ENABLE"].detail
    assert got["FS_EKF_ACTION"].status == WARN and "did not answer" in got["FS_EKF_ACTION"].detail

    cfg, sc, mission, veh = setup()
    small = CallableSource(lambda now: np.full((240, 320, 3), 99, np.uint8), 30.0)
    got = by_name(preflight(mission, cfg, sc.world, veh, small, None, False, None, camera_wait_s=0.3))
    assert got["camera"].status == FAIL and "320x240" in got["camera"].detail
    assert got["flight log"].status == WARN and "site" not in got

    silent = CallableSource(lambda now: None, 30.0)
    got = by_name(preflight(mission, cfg, sc.world, veh, silent, None, False, None, camera_wait_s=0.3))
    assert got["camera"].status == FAIL and "no frame" in got["camera"].detail
    got = by_name(preflight(mission, cfg, sc.world, veh, None, None, False, None))
    assert got["camera"].status == WARN and "cannot see or avoid" in got["camera"].detail

    far = RuleParser(list(sc.world.sites)).parse("take off to 15 m, fly to (4000, 0), then land")
    got = by_name(preflight(far, cfg, sc.world, veh))
    assert got["mission"].status == FAIL and not cleared(list(got.values()))


def test_an_unreachable_fusion_node_fails_the_check() -> None:
    from kestrel.interop.wire import SapientClient

    cfg, sc, mission, veh = setup()
    client = SapientClient("127.0.0.1", 9, reconnect_s=(0.05, 0.1), connect_timeout_s=0.2).start()
    try:
        got = by_name(preflight(mission, cfg, sc.world, veh, sapient=client))
        assert got["fusion link"].status == FAIL and "cannot reach" in got["fusion link"].detail
    finally:
        client.close()


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


@pytest.mark.slow
def test_fusion_service_and_bench_flight_from_the_command_line(tmp_path: Path) -> None:
    """``kestrel fusion`` in one process, ``kestrel fly --connect bench`` in another, in real time."""
    port = _free_port()
    site = str(ROOT / "scenarios" / "airfield_picture.yaml")
    env = {**os.environ, "PYTHONUNBUFFERED": "1"}
    kestrel = [sys.executable, "-m", "kestrel.cli"]
    fusion = subprocess.Popen(
        [*kestrel, "fusion", "--site", site, "--listen", f"127.0.0.1:{port}", "--no-tasking",
         "--out", str(tmp_path / "rap.jsonl")],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env, cwd=tmp_path,
    )  # fmt: skip
    try:
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            with socket.socket() as s:
                if s.connect_ex(("127.0.0.1", port)) == 0:
                    break
            time.sleep(0.2)
        fly = subprocess.run(
            [*kestrel, "fly", MISSION, "--connect", "bench", "--camera", "bench", "--scenario", site,
             "--sapient", f"127.0.0.1:{port}", "--log-dir", str(tmp_path / "flight"), "--yes"],
            capture_output=True, text=True, timeout=240, env=env, cwd=tmp_path,
        )  # fmt: skip
        assert fly.returncode == 0, fly.stdout + fly.stderr
        assert "preflight: CLEARED" in fly.stdout and "mission: complete" in fly.stdout
        assert "[PASS] fusion link" in fly.stdout
        time.sleep(1.5)
    finally:
        fusion.send_signal(signal.SIGINT)
        out, _ = fusion.communicate(timeout=15)
    pictures = [json.loads(x) for x in (tmp_path / "rap.jsonl").read_text().splitlines()]
    seen = [n for p in pictures for n in p["nodes"]]
    assert any(n["type"] == "NODE_TYPE_CAMERA" and n["state"] == "OK" for n in seen), out
    assert all(sum(p["rejected"].values()) == 0 for p in pictures), pictures[-1]["rejected"]
    lines = [json.loads(x) for x in (tmp_path / "flight" / "flight.jsonl").read_text().splitlines()]
    assert any(x["k"] == "event" and x["detail"] == "complete" for x in lines)
    assert all(x["link"] for x in lines if x["k"] == "tick")
