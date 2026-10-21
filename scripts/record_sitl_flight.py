"""Fly the SITL mission and save its trajectory for the demo video.

Usage (needs a built ArduCopter SITL):

    python scripts/record_sitl_flight.py ARDUCOPTER_BINARY COPTER_PARM docs/results/sitl_flight.json
"""

from __future__ import annotations

import json
import socket
import subprocess
import sys
import tempfile
import time

from kestrel.config import Config, PointSite, WorldConfig
from kestrel.live import run_live
from kestrel.mission.parser import RuleParser
from kestrel.vehicle.mavlink_vehicle import MavlinkVehicle

INSTRUCTION = "take off to 15 m, fly to the tower, orbit the tower with a radius of 15 m, then return home"


def main(binary: str, defaults: str, out: str) -> None:
    workdir = tempfile.mkdtemp(prefix="kestrel-sitl-")
    home = "-35.363261,149.165230,584,353"
    cmd = [binary, "--model", "quad", "--speedup", "1", "--defaults", defaults, "-I0", "--home", home]
    proc = subprocess.Popen(cmd, cwd=workdir, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(150):
            with socket.socket() as s:
                if s.connect_ex(("127.0.0.1", 5760)) == 0:
                    break
            time.sleep(0.2)
        cfg = Config()
        world = WorldConfig(
            fence=[(-80, -80), (80, -80), (80, 80), (-80, 80)], sites={"tower": PointSite(xy=(30, 20))}
        )
        mission = RuleParser(["tower"]).parse(INSTRUCTION)
        vehicle = MavlinkVehicle("tcp:127.0.0.1:5760", battery_capacity_wh=cfg.vehicle.battery_wh)
        try:
            vehicle.wait_ready(120)
            res = run_live(mission, cfg, world, vehicle, max_duration_s=300)
            status = list(vehicle.statustext)
        finally:
            vehicle.close()
    finally:
        proc.terminate()
        proc.wait(timeout=10)
    t0 = res.times[0]
    data = {
        "autopilot": "ArduCopter 4.6.3 SITL",
        "instruction": INSTRUCTION,
        "completed": res.completed,
        "t": [round(t - t0, 3) for t in res.times],
        "pos": [[round(float(v), 3) for v in p] for p in res.positions],
        "events": [{"t": round(e.t - t0, 2), "kind": e.kind, "detail": e.detail} for e in res.events],
        "autopilot_messages": status,
        "tower": [30.0, 20.0],
        "fence": world.fence,
    }
    with open(out, "w", encoding="utf-8") as fh:
        json.dump(data, fh)
    print(f"saved {len(data['t'])} samples, completed={res.completed}")


if __name__ == "__main__":
    main(*sys.argv[1:4])
