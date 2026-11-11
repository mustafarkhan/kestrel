"""Real-time autonomy loop for a vehicle reached over MAVLink.

The same executive, tracker and safety monitor as the simulator; what differs is where
things come from. Position comes from the autopilot, frames from a camera
(``perception/source.py``), tasks and traffic from a SAPIENT fusion node over TCP
(``interop/wire.py``), and time from the autopilot's clock, so the loop is correct under
SITL ``--speedup`` too.

What the loop does that the simulator never needed, each a way a real flight goes wrong:

* **A pilot takes the aircraft.** If the autopilot leaves GUIDED while armed (a safety pilot's
  switch, or one of its own failsafes), the loop stops sending setpoints at once and only
  watches until the aircraft is disarmed. It never competes for control.
* **The autopilot goes quiet.** Without a fresh position no setpoint is sent (the autopilot
  holds, then applies its own failsafe); after ``telemetry_land_s`` the loop asks for a
  landing once, in case the link is one-way.
* **The camera fails.** A camera that stops delivering frames is a failed sensor, not an
  empty sky: the node reports ``SYSTEM_ERROR`` and returns home, because it can no longer
  see traffic.
* **The fusion link drops.** Reports wait in the client's queue and are sent when it
  returns. The link-loss policy of the safety monitor (return home, or carry on for a
  bounded time) applies exactly as in simulation.
* **The site file is for somewhere else.** With ``align_site`` the vehicle's own position
  places it on the site before arming; home becomes the take-off point.
* **Nothing is left only in memory.** Events, a 5 Hz state record and the run's metadata are
  written to disk as they happen, so a crash or a power cut still leaves the flight log.
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from collections import Counter, deque
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import IO, Any

import numpy as np

from kestrel.config import Config, WorldConfig
from kestrel.geofence import Geofence
from kestrel.geometry import Vec
from kestrel.interop.sapient import SapientExporter, SapientOptions, software_version
from kestrel.interop.wire import SapientClient
from kestrel.mission.executive import Context, Event, Executive
from kestrel.mission.schema import Mission
from kestrel.mission.validator import ValidationReport, validate
from kestrel.perception.camera import Camera
from kestrel.perception.detector import SkyTargetDetector
from kestrel.perception.source import FrameSource
from kestrel.perception.tracker import MultiTargetTracker, TrackEstimate
from kestrel.safety.monitor import SafetyMonitor, SafetyRequest
from kestrel.types import VehicleState
from kestrel.vehicle.base import Vehicle

log = logging.getLogger("kestrel.live")
CLASSIFY_STANDOFF_M = 30.0  # as the simulator: close to this on a location with a measured altitude
TRAFFIC_MAX_AGE_S = 5.0


class MissionRejected(ValueError):
    def __init__(self, report: ValidationReport) -> None:
        super().__init__(report.summary())
        self.report = report


@dataclass
class LiveOptions:
    rate_hz: float = 20.0
    max_duration_s: float = 900.0
    frames: FrameSource | None = None
    camera_timeout_s: float = 2.0  # no frame for this long in the air: the camera has failed
    sapient: SapientClient | None = None  # link to the fusion node (already started)
    log_dir: Path | None = None
    telemetry_timeout_s: float = 1.5  # no position for this long: send nothing
    telemetry_land_s: float = 10.0  # ... and after this long, ask once for a landing
    align_site: bool = False  # place the vehicle on the site from its own global position
    stop: threading.Event | None = None  # set by the operator's console: return home
    clock: Callable[[], float] = time.monotonic  # what "now" is (a bench steps simulated time)
    realtime: bool = True  # sleep between ticks; a stepped bench does not


@dataclass
class LiveResult:
    events: list[Event]
    positions: list[Vec] = field(default_factory=list)
    times: list[float] = field(default_factory=list)
    safety_counts: Counter[str] = field(default_factory=Counter)
    completed: bool = False
    pilot_took_control: bool = False
    frames: int = 0
    tracks_reported: int = 0
    tasks_accepted: int = 0
    traffic_received: int = 0
    log_dir: Path | None = None


class FlightRecorder:
    """Appends the flight to disk as it happens: ``flight.jsonl`` and ``meta.json``."""

    def __init__(self, directory: Path | None, meta: dict[str, Any]) -> None:
        self._fh: IO[str] | None = None
        self._last_tick = -math.inf
        if directory is None:
            return
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "meta.json").write_text(json.dumps(meta, indent=1, default=str), encoding="utf-8")
        self._fh = open(directory / "flight.jsonl", "a", encoding="utf-8")  # noqa: SIM115

    def _write(self, record: dict[str, Any], sync: bool) -> None:
        if self._fh is None:
            return
        self._fh.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._fh.flush()
        if sync:
            os.fsync(self._fh.fileno())

    def event(self, t: float, kind: str, detail: str) -> None:
        self._write({"k": "event", "t": round(t, 2), "kind": kind, "detail": detail}, sync=True)

    def tick(self, t: float, record: dict[str, Any], period_s: float = 0.2) -> None:
        if t - self._last_tick >= period_s:
            self._last_tick = t
            self._write({"k": "tick", "t": round(t, 2)} | record, sync=False)

    def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None


def _round(v: Vec) -> list[float]:
    return [round(float(x), 2) for x in v]


def run_live(
    mission: Mission,
    cfg: Config,
    world: WorldConfig,
    vehicle: Vehicle,
    rate_hz: float = 20.0,
    max_duration_s: float = 900.0,
    options: LiveOptions | None = None,
) -> LiveResult:
    opts = options or LiveOptions(rate_hz=rate_hz, max_duration_s=max_duration_s)
    if opts.align_site:
        takeoff = vehicle.align_to_site(world.origin_lla)  # type: ignore[attr-defined]
        world = world.model_copy(update={"home": (float(takeoff[0]), float(takeoff[1]), 0.0)})
    report = validate(mission, cfg, world)
    if not report.ok:
        raise MissionRejected(report)
    fence = Geofence(world.fence)
    if fence.signed_distance(np.array(world.home)) < cfg.safety.fence_margin:
        raise ValueError(
            f"the take-off point ({world.home[0]:.0f}, {world.home[1]:.0f}) m is not inside the site's "
            "fence: wrong site file, or the vehicle is in the wrong place"
        )
    executive = Executive(report.mission, cfg, world)
    monitor = SafetyMonitor(cfg.safety, cfg.vehicle, world)
    res = LiveResult(events=executive.events, log_dir=opts.log_dir)
    st = vehicle.state()
    t_start = st.t

    camera = Camera(cfg.camera, cfg.vehicle.camera_offset)
    detector = SkyTargetDetector(camera, cfg.detector, "bright" if cfg.camera.band == "thermal" else "dark")
    tracker = MultiTargetTracker(cfg.tracker, camera)
    history: deque[VehicleState] = deque(maxlen=int(2.0 * opts.rate_hz) + 2)
    camera_failed = False

    exporter: SapientExporter | None = None
    if opts.sapient is not None:
        exporter = SapientExporter(
            SapientOptions(
                origin_lla=world.origin_lla,
                epoch_s=time.time() - st.t,  # the autopilot clock mapped onto UTC
                altitude_band_m=(0.0, cfg.safety.max_altitude),
                camera_band=cfg.camera.band,
                seed=int.from_bytes(os.urandom(8), "big"),
            ),
            sink=opts.sapient.send,
            keep=False,
        )
    traffic: list[TrackEstimate] = []

    rec = FlightRecorder(
        opts.log_dir,
        {
            "software": software_version(),
            "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "mission": report.mission.model_dump(mode="json"),
            "config": cfg.model_dump(mode="json"),
            "world": world.model_dump(mode="json"),
            "camera": opts.frames is not None,
            "sapient": opts.sapient is not None,
        },
    )
    logged = 0

    def note(t: float, kind: str, detail: str) -> None:
        executive.events.append(Event(t, kind, detail))

    age_of = getattr(vehicle, "telemetry_age_s", lambda: 0.0)
    pilot = getattr(vehicle, "pilot_has_control", lambda: False)
    period = 1.0 / opts.rate_hz
    clock = opts.clock
    next_tick = clock()
    silent_since: float | None = None
    land_sent = False
    try:
        while True:
            # ------------------------------------------------------ is the autopilot there?
            if age_of() > opts.telemetry_timeout_s:
                now = clock()
                if silent_since is None:
                    silent_since = now
                    note(st.t, "fault", "no position from the autopilot: sending nothing")
                if not land_sent and now - silent_since > opts.telemetry_land_s:
                    land_sent = True
                    vehicle.land()
                    note(st.t, "fault", "autopilot silent: landing requested")
                if now - silent_since > opts.max_duration_s:
                    break
            else:
                if silent_since is not None:
                    silent_since = None
                    note(st.t, "info", "autopilot position restored")
                st = vehicle.state()
                history.append(st)

                # -------------------------------------------------- has someone else got it?
                if pilot():
                    mode = getattr(vehicle, "mode_name", lambda: "another mode")()
                    note(st.t, "handover", f"autopilot left GUIDED ({mode}): mission loop stands down")
                    res.pilot_took_control = True
                    break

                tracks = _perceive(opts, st, history, camera, detector, tracker, cfg, res)
                if (
                    opts.frames is not None
                    and not camera_failed
                    and st.armed
                    and not st.on_ground
                    and opts.frames.age_s() > opts.camera_timeout_s
                ):
                    camera_failed = True
                    if exporter is not None:
                        exporter.system = "SYSTEM_ERROR"
                ctx = Context(t=st.t, state=st, vehicle=vehicle, tracks=tracks)
                if camera_failed:
                    executive.abort(SafetyRequest.RTL, ctx, "camera stopped delivering frames")
                if opts.stop is not None and opts.stop.is_set():
                    executive.abort(SafetyRequest.RTL, ctx, "operator stop")
                if st.t - t_start > opts.max_duration_s:
                    executive.abort(SafetyRequest.LAND, ctx, "max duration reached")

                # -------------------------------------------------- the fusion node
                link_ok = True
                if opts.sapient is not None and exporter is not None:
                    link_ok = opts.sapient.connected
                    fresh = _from_fusion(opts.sapient, exporter, executive, st, res, fence)
                    if fresh:
                        traffic = fresh
                    traffic = [tr for tr in traffic if st.t - tr.last_hit_t < TRAFFIC_MAX_AGE_S]

                monitor.check_vehicle(st, link_ok)
                if monitor.latched is not SafetyRequest.NONE:
                    executive.abort(monitor.latched, ctx, monitor.reasons[-1])
                sp = executive.step(ctx)
                dec = monitor.filter(st, sp, tracks + traffic, executive.low_altitude_ok)
                if dec.setpoint is not None:
                    vehicle.set_setpoint(dec.setpoint)
                behaviour = executive.current.name if executive.current else "idle"
                if exporter is not None:
                    exporter.step(st.t, st.pos, behaviour, tracks, [], False, world.home[2])
                    res.tracks_reported += len(tracks)

                res.positions.append(np.array(st.pos))
                res.times.append(st.t)
                rec.tick(
                    st.t,
                    {
                        "pos": _round(st.pos),
                        "vel": _round(st.vel),
                        "sp": _round(dec.setpoint.pos) if dec.setpoint is not None else None,
                        "behaviour": behaviour,
                        "interventions": list(dec.interventions),
                        "tracks": [[tr.ident, *_round(tr.pos), tr.p_uav] for tr in tracks],
                        "traffic": len(traffic),
                        "link": link_ok,
                        "battery_wh": round(st.battery_wh, 2),
                    },
                )
                if executive.finished and not st.armed:
                    break
            for e in executive.events[logged:]:
                rec.event(e.t, e.kind, e.detail)
                log.info("%7.1fs %s: %s", e.t, e.kind, e.detail)
            logged = len(executive.events)
            next_tick += period
            if opts.realtime:
                time.sleep(max(0.0, next_tick - clock()))
    finally:
        if exporter is not None:
            exporter.goodbye(st.t, st.pos)
        for e in executive.events[logged:]:
            rec.event(e.t, e.kind, e.detail)
        rec.close()
    res.safety_counts = monitor.counts
    res.completed = executive.finished and executive.aborted is None and not res.pilot_took_control
    return res


def _perceive(
    opts: LiveOptions,
    st: VehicleState,
    history: deque[VehicleState],
    camera: Camera,
    detector: SkyTargetDetector,
    tracker: MultiTargetTracker,
    cfg: Config,
    res: LiveResult,
) -> list[TrackEstimate]:
    """Run the detector on the newest frame, with the attitude the vehicle had when it was taken."""
    if opts.frames is None:
        return []
    frame = opts.frames.read()
    if frame is not None:
        taken, rgb = frame
        if rgb.shape[0] != camera.height or rgb.shape[1] != camera.width:
            raise ValueError(
                f"camera delivers {rgb.shape[1]}x{rgb.shape[0]} frames but the config describes "
                f"{camera.width}x{camera.height}: bearings would be wrong"
            )
        # frame time on the autopilot's clock: now minus how long ago it was taken
        t_frame = st.t - (opts.clock() - taken) - cfg.camera.latency_s
        then = min(history, key=lambda s: abs(s.t - t_frame))
        pose = camera.pose(then.pos, then.quat)
        tracker.update(t_frame, detector.detect(rgb, pose), pose)
        res.frames += 1
    return tracker.estimates(st.t)


def _from_fusion(
    client: SapientClient,
    exporter: SapientExporter,
    executive: Executive,
    st: VehicleState,
    res: LiveResult,
    fence: Geofence,
) -> list[TrackEstimate]:
    """Handle what the fusion node sent: tasks are acknowledged and acted on; its detection
    reports are traffic for the separation filter. Returns the new traffic, if any arrived."""
    traffic: list[TrackEstimate] = []
    for msg in client.poll():
        if msg.get("destination_id") not in (None, "", exporter.camera_node_id):
            continue  # addressed to another node
        if "registration_ack" in msg and not msg["registration_ack"].get("acceptance", False):
            reasons = "; ".join(msg["registration_ack"].get("ack_response_reason") or ["no reason given"])
            executive.events.append(Event(st.t, "fault", f"fusion node refused registration: {reasons}"))
        elif "task" in msg:
            task = msg["task"]
            task_id = str(task.get("task_id", ""))
            try:
                look = exporter.look_at_detail(msg)
            except (KeyError, ValueError) as e:
                look = None
                exporter.task_ack(st.t, task_id, False, f"unreadable location: {e}")
                continue
            if task.get("control") != "CONTROL_START" or look is None:
                exporter.task_ack(st.t, task_id, False, "only CONTROL_START of a LOOK_AT task is supported")
            elif executive.aborted:
                exporter.task_ack(st.t, task_id, False, "mission aborted")
            elif not np.all(np.isfinite(look[1])) or fence.signed_distance(look[1]) <= 0.0:
                exporter.task_ack(st.t, task_id, False, "location is outside the operating area")
            else:
                _, xy, z_known = look
                exporter.task_ack(st.t, task_id, True)
                executive.request_investigate(
                    xy, st.t, "C2 tasking (SAPIENT)", CLASSIFY_STANDOFF_M if z_known else None
                )
                res.tasks_accepted += 1
        elif "detection_report" in msg:
            try:
                est = exporter.traffic_estimate(msg, -1000 - len(traffic))
            except (KeyError, ValueError):
                est = None
            if est is not None and np.all(np.isfinite(est.pos)) and np.all(np.isfinite(est.cov)):
                traffic.append(est)
                res.traffic_received += 1
    return traffic
