"""Checks made on the ground before the flight loop is allowed to arm.

Each check answers one question whose wrong answer has ended real flights: is the autopilot
talking, does it know where it is, is this the right site, will the battery cover the plan
with what is in it *now*, is the camera delivering the frames the configuration describes,
is the fusion link up, are the autopilot's own failsafes armed. A ``FAIL`` stops the flight;
a ``WARN`` is shown and left to the operator.

The autopilot's own failsafes matter because Kestrel runs on a companion computer: its
geofence and energy rules stop working the moment that computer or its link fails. The
autopilot's fence, battery and radio failsafes are the layer underneath, and these checks
say when they are switched off.
"""

from __future__ import annotations

import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from kestrel.config import Config, WorldConfig
from kestrel.geofence import Geofence
from kestrel.interop.utm import lla_to_enu
from kestrel.interop.wire import SapientClient
from kestrel.mission.schema import Mission
from kestrel.mission.validator import validate
from kestrel.perception.source import FrameSource

PASS, WARN, FAIL = "PASS", "WARN", "FAIL"


@dataclass(frozen=True)
class Check:
    name: str
    status: str
    detail: str

    def line(self) -> str:
        return f"[{self.status}] {self.name}: {self.detail}"


# ArduCopter parameters that are the autopilot's own protection, and the value that means "off"
AUTOPILOT_FAILSAFES = (
    ("FENCE_ENABLE", "the autopilot's own geofence", 0.0),
    ("FS_THR_ENABLE", "radio-control failsafe", 0.0),
    ("BATT_FS_LOW_ACT", "low-battery failsafe action", 0.0),
    ("FS_EKF_ACTION", "navigation (EKF) failsafe", 0.0),
)


def _vehicle_checks(vehicle: Any, cfg: Config, world: WorldConfig, align_site: bool) -> list[Check]:
    out: list[Check] = []
    hb = getattr(vehicle, "heartbeat_age_s", None)
    if hb is not None:
        age = hb()
        out.append(Check("autopilot", PASS if age < 2.5 else FAIL, f"last heartbeat {age:.1f} s ago"))
    try:
        st = vehicle.state()
    except Exception as e:
        return [*out, Check("navigation", FAIL, f"no position estimate ({e})")]
    out.append(Check("navigation", PASS, "local position and attitude are streaming"))
    grounded = st.on_ground and not st.armed
    out.append(
        Check(
            "on the ground",
            PASS if grounded else FAIL,
            "disarmed, landed" if grounded else "armed or airborne",
        )
    )
    gps = getattr(vehicle, "gps", None)
    if gps is not None:
        fix, sats = gps()
        status = PASS if fix >= 3 and sats >= 8 else WARN if fix >= 3 else FAIL
        out.append(Check("GNSS", status, f"fix type {fix}, {sats} satellites (want a 3-D fix and 8 or more)"))
    frac = st.battery_fraction
    out.append(
        Check(
            "battery charge",
            PASS if frac >= 0.9 else WARN if frac >= 0.6 else FAIL,
            f"{100 * frac:.0f}% reported (missions are planned for a full pack)",
        )
    )
    if align_site:
        lla = getattr(vehicle, "global_position", lambda: None)()
        if lla is None:
            out.append(Check("site", FAIL, "no global position: cannot place the vehicle on the site"))
        else:
            east, north, _ = lla_to_enu(world.origin_lla, lla[0], lla[1], world.origin_lla[2])
            inside = Geofence(world.fence).signed_distance(np.array([east, north]))
            ok = inside >= cfg.safety.fence_margin
            where = f"vehicle is {east:.0f} m east, {north:.0f} m north of the site origin"
            detail = (
                f"{where}, {inside:.0f} m inside the fence" if inside > 0 else f"{where}: OUTSIDE the fence"
            )
            out.append(Check("site", PASS if ok else FAIL, detail))
    param = getattr(vehicle, "param", None)
    if param is not None:
        for name, what, off in AUTOPILOT_FAILSAFES:
            value = param(name)
            if value is None:
                out.append(Check(name, WARN, f"{what}: the autopilot did not answer"))
            else:
                state = "OFF" if value == off else f"on ({value:g})"
                out.append(Check(name, WARN if value == off else PASS, f"{what} is {state}"))
    return out


def _camera_check(frames: FrameSource, cfg: Config, wait_s: float) -> Check:
    deadline = time.monotonic() + wait_s
    got: list[np.ndarray] = []
    first = last = 0.0
    while time.monotonic() < deadline:
        f = frames.read()
        if f is not None:
            last = time.monotonic()
            first = first or last
            got.append(f[1])
        time.sleep(0.01)
    if not got:
        return Check("camera", FAIL, f"no frame in {wait_s:.0f} s")
    h, w = got[-1].shape[:2]
    if (w, h) != (cfg.camera.width, cfg.camera.height):
        return Check(
            "camera", FAIL, f"frames are {w}x{h}, the config describes {cfg.camera.width}x{cfg.camera.height}"
        )
    mean = float(got[-1].mean())
    if mean < 3.0 or mean > 252.0:
        return Check(
            "camera", FAIL, f"frame is {'black' if mean < 3 else 'saturated'} (mean level {mean:.0f})"
        )
    rate = (len(got) - 1) / (last - first) if len(got) > 1 and last > first else 0.0
    slow = rate < 0.6 * cfg.camera.rate_hz
    return Check(
        "camera",
        WARN if slow else PASS,
        f"{w}x{h} at {rate:.0f} Hz (config {cfg.camera.rate_hz:g} Hz), {cfg.camera.band}",
    )


def _clock_check() -> Check:
    """Reports are fused by timestamp: a node whose clock is off has its reports dropped."""
    if shutil.which("timedatectl") is None:
        return Check("clock", WARN, "cannot tell whether this computer's clock is synchronised")
    try:
        out = subprocess.run(
            ["timedatectl", "show", "-p", "NTPSynchronized", "--value"],
            capture_output=True, text=True, timeout=3, check=False,
        ).stdout.strip()  # fmt: skip
    except (OSError, subprocess.SubprocessError):
        out = ""
    if out == "yes":
        return Check("clock", PASS, "synchronised")
    return Check("clock", WARN, "this computer's clock is not reported as synchronised (GNSS or NTP)")


def preflight(
    mission: Mission,
    cfg: Config,
    world: WorldConfig,
    vehicle: Any,
    frames: FrameSource | None = None,
    sapient: SapientClient | None = None,
    align_site: bool = False,
    log_dir: Path | None = None,
    camera_wait_s: float = 3.0,
) -> list[Check]:
    checks = _vehicle_checks(vehicle, cfg, world, align_site)
    report = validate(mission, cfg, world)
    minutes, need, usable = report.est_duration_s / 60, report.est_energy_wh, report.usable_energy_wh
    detail = "; ".join(report.errors) or f"{minutes:.1f} min, {need:.1f} of {usable:.1f} Wh usable"
    checks.append(Check("mission", PASS if report.ok else FAIL, detail))
    for w in report.warnings:
        checks.append(Check("mission", WARN, w))
    if frames is not None:
        checks.append(_camera_check(frames, cfg, camera_wait_s))
    else:
        checks.append(Check("camera", WARN, "no camera: this flight cannot see or avoid other aircraft"))
    if sapient is not None:
        deadline = time.monotonic() + 5.0
        while not sapient.connected and time.monotonic() < deadline:
            time.sleep(0.05)
        up = sapient.connected
        action = cfg.safety.link_loss_action
        checks.append(
            Check(
                "fusion link",
                PASS if up else FAIL,
                f"connected to {sapient.host}:{sapient.port}" + ("" if sapient.tls else " (no TLS)")
                if up
                else f"cannot reach {sapient.host}:{sapient.port}; link loss means '{action}'",
            )
        )
        checks.append(_clock_check())
    if log_dir is not None:
        log_dir.mkdir(parents=True, exist_ok=True)
        free = shutil.disk_usage(log_dir).free / 1e6
        checks.append(Check("flight log", PASS if free > 200 else FAIL, f"{free:.0f} MB free at {log_dir}"))
    else:
        checks.append(Check("flight log", WARN, "no log directory: this flight will leave no record"))
    return checks


def cleared(checks: list[Check]) -> bool:
    return all(c.status != FAIL for c in checks)
