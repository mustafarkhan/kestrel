"""Real-time autonomy loop for a vehicle reached over MAVLink.

Same executive and safety monitor as the simulator; only the clock and the
vehicle differ. The mission clock is the autopilot's boot time, so the loop is
correct with SITL ``--speedup`` too. Camera perception is not wired here (no
frame source on the bench), so ``on_contact`` behaviours have no tracks to act on.
"""

from __future__ import annotations

import time
from collections import Counter
from dataclasses import dataclass, field

import numpy as np

from kestrel.config import Config, WorldConfig
from kestrel.geometry import Vec
from kestrel.mission.executive import Context, Event, Executive
from kestrel.mission.schema import Mission
from kestrel.mission.validator import ValidationReport, validate
from kestrel.safety.monitor import SafetyMonitor, SafetyRequest
from kestrel.vehicle.base import Vehicle


class MissionRejected(ValueError):
    def __init__(self, report: ValidationReport) -> None:
        super().__init__(report.summary())
        self.report = report


@dataclass
class LiveResult:
    events: list[Event]
    positions: list[Vec] = field(default_factory=list)
    times: list[float] = field(default_factory=list)
    safety_counts: Counter[str] = field(default_factory=Counter)
    completed: bool = False


def run_live(
    mission: Mission,
    cfg: Config,
    world: WorldConfig,
    vehicle: Vehicle,
    rate_hz: float = 20.0,
    max_duration_s: float = 900.0,
) -> LiveResult:
    report = validate(mission, cfg, world)
    if not report.ok:
        raise MissionRejected(report)
    executive = Executive(report.mission, cfg, world)
    monitor = SafetyMonitor(cfg.safety, cfg.vehicle, world)
    res = LiveResult(events=executive.events)
    t_start = vehicle.state().t
    period = 1.0 / rate_hz
    next_tick = time.monotonic()
    while True:
        st = vehicle.state()
        if st.t - t_start > max_duration_s:
            executive.abort(SafetyRequest.LAND, Context(st.t, st, vehicle, []), "max duration reached")
        ctx = Context(t=st.t, state=st, vehicle=vehicle, tracks=[])
        monitor.check_vehicle(st, link_ok=True)
        if monitor.latched is not SafetyRequest.NONE:
            executive.abort(monitor.latched, ctx, monitor.reasons[-1])
        sp = executive.step(ctx)
        dec = monitor.filter(st, sp, [], executive.low_altitude_ok)
        if dec.setpoint is not None:
            vehicle.set_setpoint(dec.setpoint)
        res.positions.append(np.array(st.pos))
        res.times.append(st.t)
        if executive.finished and not st.armed:
            break
        next_tick += period
        time.sleep(max(0.0, next_tick - time.monotonic()))
    res.safety_counts = monitor.counts
    res.completed = executive.finished and executive.aborted is None
    return res
