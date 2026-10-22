"""Integration test against a real ArduCopter SITL binary.

Runs only when ``ARDUPILOT_SITL`` points at a built ``arducopter`` and
``ARDUPILOT_DEFAULTS`` at ``Tools/autotest/default_params/copter.parm``
(CI builds both and caches them). Flies a real mission through GUIDED mode.
"""

from __future__ import annotations

import os
import socket
import subprocess
import tempfile
import time
from collections.abc import Iterator

import numpy as np
import pytest

from kestrel.config import Config, PointSite, WorldConfig
from kestrel.live import run_live
from kestrel.mission.parser import RuleParser
from kestrel.vehicle.mavlink_vehicle import MavlinkVehicle

pytestmark = pytest.mark.sitl

BIN = os.environ.get("ARDUPILOT_SITL")
DEFAULTS = os.environ.get("ARDUPILOT_DEFAULTS")


def _wait_port(port: int, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with socket.socket() as s:
            if s.connect_ex(("127.0.0.1", port)) == 0:
                return
        time.sleep(0.2)
    raise TimeoutError(f"SITL did not open port {port}")


@pytest.fixture()
def sitl() -> Iterator[str]:
    if not BIN or not DEFAULTS:
        pytest.skip("set ARDUPILOT_SITL and ARDUPILOT_DEFAULTS to run SITL tests")
    workdir = tempfile.mkdtemp(prefix="kestrel-sitl-")
    proc = subprocess.Popen(
        [
            BIN,
            "--model",
            "quad",
            "--speedup",
            "1",
            "--defaults",
            DEFAULTS,
            "-I0",
            "--home",
            "-35.363261,149.165230,584,353",
        ],
        cwd=workdir,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        _wait_port(5760, 30)
        yield "tcp:127.0.0.1:5760"
    finally:
        proc.terminate()
        proc.wait(timeout=10)


def test_mission_on_arducopter_sitl(sitl: str) -> None:
    cfg = Config()
    world = WorldConfig(
        fence=[(-80, -80), (80, -80), (80, 80), (-80, 80)], sites={"tower": PointSite(xy=(30, 20))}
    )
    mission = RuleParser(["tower"]).parse(
        "take off to 15 m, fly to the tower, orbit the tower with a radius of 15 m, then return home"
    )
    vehicle = MavlinkVehicle(sitl, battery_capacity_wh=cfg.vehicle.battery_wh)
    try:
        vehicle.wait_ready(120)
        res = run_live(mission, cfg, world, vehicle, max_duration_s=300)
    finally:
        vehicle.close()
    path = np.array(res.positions)
    assert res.completed, [f"{e.t:.1f} {e.kind} {e.detail}" for e in res.events]
    assert abs(path[:, 2].max() - 15.0) < 1.5
    tower = np.array([30.0, 20.0])
    radii = np.linalg.norm(path[:, 0:2] - tower, axis=1)
    assert np.any(np.abs(radii - 15.0) < 1.5)  # flew the orbit
    assert np.linalg.norm(path[-1, 0:2]) < 2.0  # landed at home
