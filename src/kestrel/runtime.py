"""Closed-loop simulation runner.

One autonomy tick (default 20 Hz):

    navigation state -> safety pre-checks -> (abort?) -> executive -> safety filter -> autopilot

Perception runs at camera rate on rendered frames. Frames are rendered from the
*true* pose but processed with the *estimated* pose, so navigation error turns
into bearing error exactly as it would on a real aircraft. Detections are
delivered after the configured latency and fused at their capture time.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass, field

import numpy as np

from kestrel.config import Config, Scenario
from kestrel.geometry import Vec
from kestrel.mission.executive import Context, Event, Executive
from kestrel.mission.llm import ChatBackend, MissionPlanner, PlanResult
from kestrel.mission.parser import parse_emergency
from kestrel.mission.schema import Mission
from kestrel.mission.validator import ValidationReport, validate
from kestrel.perception.acoustic import AcousticFix, AcousticNetwork
from kestrel.perception.camera import Camera, CameraPose
from kestrel.perception.detector import Detection, SkyTargetDetector
from kestrel.perception.render import SkyRenderer
from kestrel.perception.tracker import MultiTargetTracker, TrackEstimate
from kestrel.safety.monitor import SafetyMonitor, SafetyRequest
from kestrel.sim.world import SimWorld
from kestrel.vehicle.sim_vehicle import SimVehicle


@dataclass
class Frame:
    """A recorded camera frame with overlays, for replay and the demo GIF."""

    t: float
    image: np.ndarray
    detections: list[Detection]
    tracks_px: list[tuple[int, float, float, bool]]  # id, u, v, confirmed
    truth_px: list[tuple[float, float]]
    behaviour: str


@dataclass
class FlightLog:
    t: list[float] = field(default_factory=list)
    pos: list[Vec] = field(default_factory=list)  # truth
    vel: list[Vec] = field(default_factory=list)
    yaw: list[float] = field(default_factory=list)
    sp: list[Vec] = field(default_factory=list)
    battery_wh: list[float] = field(default_factory=list)
    behaviour: list[str] = field(default_factory=list)
    interventions: list[str] = field(default_factory=list)
    target: list[Vec] = field(default_factory=list)  # first intruder truth (nan if absent)
    target_in_fov: list[bool] = field(default_factory=list)
    track: list[Vec] = field(default_factory=list)  # followed / best track estimate (nan if none)
    track_std: list[float] = field(default_factory=list)
    n_confirmed: list[int] = field(default_factory=list)
    frames: list[Frame] = field(default_factory=list)

    def arrays(self) -> dict[str, np.ndarray]:
        return {
            "t": np.array(self.t),
            "pos": np.array(self.pos),
            "vel": np.array(self.vel),
            "yaw": np.array(self.yaw),
            "sp": np.array(self.sp),
            "battery_wh": np.array(self.battery_wh),
            "target": np.array(self.target),
            "target_in_fov": np.array(self.target_in_fov),
            "track": np.array(self.track),
            "track_std": np.array(self.track_std),
            "n_confirmed": np.array(self.n_confirmed),
        }


@dataclass
class RunResult:
    plan: PlanResult
    report: ValidationReport | None
    log: FlightLog
    events: list[Event]
    safety_counts: Counter[str]
    safety_reasons: list[str]
    false_tracks: int
    energy_used_wh: float
    landed: bool
    completed: bool
    scenario: Scenario
    config: Config
    false_confirmed: int = 0  # confirmed tracks that never matched a real aircraft (false alarms)

    @property
    def flown(self) -> bool:
        return len(self.log.t) > 0


class Perception:
    def __init__(self, cfg: Config, seed: int) -> None:
        self.camera = Camera(cfg.camera, cfg.vehicle.camera_offset)
        self.renderer = SkyRenderer(self.camera, seed=seed)
        self.detector = SkyTargetDetector(self.camera, cfg.detector)
        self.tracker = MultiTargetTracker(cfg.tracker, self.camera)


def _cue_estimate(fix: AcousticFix, observer_alt: float, ident: int) -> TrackEstimate:
    """An acoustic fix as a contact: a vertical column at our altitude (height is unknown)."""
    cov = np.zeros((6, 6))
    cov[0:2, 0:2] = fix.cov_xy
    cov[2, 2] = 1.0
    cov[3:6, 3:6] = np.eye(3) * 25.0
    pos = np.array([fix.xy[0], fix.xy[1], observer_alt])
    return TrackEstimate(ident, pos, np.zeros(3), cov, True, 0, fix.t)


def _span_yaw(vel: Vec) -> float:
    return math.atan2(vel[1], vel[0]) if float(np.linalg.norm(vel[0:2])) > 0.2 else 0.0


def plan_and_validate(
    command: str | Mission, cfg: Config, scenario: Scenario, backend: ChatBackend | None = None
) -> tuple[PlanResult, ValidationReport | None]:
    if isinstance(command, Mission):
        plan = PlanResult(command, "rules")
    else:
        plan = MissionPlanner(list(scenario.world.sites), backend).plan(command)
    if plan.mission is None:
        return plan, None
    return plan, validate(plan.mission, cfg, scenario.world)


def run(
    command: str | Mission,
    cfg: Config,
    scenario: Scenario,
    backend: ChatBackend | None = None,
    operator_commands: list[tuple[float, str]] | None = None,
    record_every: int = 0,
    perception_enabled: bool = True,
    record_downsample: int = 2,
    avoidance: bool = True,
) -> RunResult:
    """Fly one closed-loop mission.

    ``avoidance=False`` is the unmitigated baseline that detect-and-avoid standards compare
    against: same mission and traffic, but no contact handling, no yielding and no
    separation filtering. The ratio of separation losses with and without avoidance is the
    risk ratio.
    """
    plan, report = plan_and_validate(command, cfg, scenario, backend)
    world = SimWorld(cfg, scenario)
    log = FlightLog()
    if report is None or not report.ok:
        return RunResult(plan, report, log, [], Counter(), [], 0, 0.0, True, False, scenario, cfg)

    vehicle = SimVehicle(world, cfg.state_noise, seed=scenario.seed)
    perc = Perception(cfg, scenario.seed)
    mission = report.mission if avoidance else report.mission.model_copy(update={"on_contact": "report"})
    executive = Executive(mission, cfg, scenario.world, avoidance=avoidance)
    monitor = SafetyMonitor(cfg.safety, cfg.vehicle, scenario.world, separation=avoidance)
    matched_ids: set[int] = set()
    confirmed_ids: set[int] = set()
    acoustic = (
        AcousticNetwork(cfg.acoustic, scenario.world, np.random.default_rng(scenario.seed + 91))
        if cfg.acoustic.enabled and perception_enabled
        else None
    )
    next_acoustic = 0.0
    acoustic_pending: list[tuple[float, AcousticFix]] = []
    acoustic_fixes: list[AcousticFix] = []
    cmds = sorted(operator_commands or [])

    substeps = max(1, round(1.0 / (cfg.autonomy_rate_hz * cfg.physics_dt)))
    cam_period = 1.0 / cfg.camera.rate_hz
    next_cam = 0.0
    pending: list[tuple[float, float, list[Detection], CameraPose]] = []
    frame_count = 0
    tilt = cfg.camera.pitch_up_deg
    gimbal_rng = np.random.default_rng(scenario.seed + 77)
    jitter = math.radians(cfg.camera.gimbal_jitter_deg)
    ctx_prev: Context | None = None
    span = {i: c.span_m for i, c in enumerate(scenario.intruders)}

    while world.t < scenario.max_duration_s:
        for _ in range(substeps):
            world.step(cfg.physics_dt)
        t = world.t
        st = vehicle.state()
        intruders = world.intruder_states()

        # ---------------------------------------------------------- perception
        # gimbal: slew towards the executive's request at a bounded rate
        want = executive.gimbal_tilt(ctx_prev) if ctx_prev is not None else None
        want = cfg.camera.pitch_up_deg if want is None else want
        step = cfg.camera.gimbal_rate_deg_s / cfg.autonomy_rate_hz
        tilt += float(np.clip(want - tilt, -step, step))
        tilt = float(np.clip(tilt, cfg.camera.gimbal_min_deg, cfg.camera.gimbal_max_deg))

        if perception_enabled and t >= next_cam:
            next_cam += cam_period
            s = world.quad.state
            targets = [(p, span[i], _span_yaw(v)) for i, p, v in intruders]
            for off in cfg.camera.yaw_offsets_deg:
                pose_c = perc.camera.pose(s.pos, s.quat, tilt, off).jittered(gimbal_rng, jitter)
                est_c = perc.camera.pose(st.pos, st.quat, tilt, off)
                img_c = perc.renderer.render(pose_c, targets, cfg.camera.noise_sigma)
                dets_c = perc.detector.detect(img_c, est_c)
                pending.append((t + cfg.camera.latency_s, t, dets_c, est_c))
                if off == cfg.camera.yaw_offsets_deg[0]:
                    true_pose, img, dets = pose_c, img_c, dets_c
            frame_count += 1
            if record_every and frame_count % record_every == 0:
                ests = perc.tracker.estimates(t, confirmed_only=False)
                tp = []
                for e in ests:
                    uv = perc.camera.project(true_pose, e.pos)
                    if uv:
                        tp.append((e.ident, uv[0], uv[1], e.confirmed))
                truth = [uv[:2] for _, p, _ in intruders if (uv := perc.camera.project(true_pose, p))]
                b = executive.current.name if executive.current else "idle"
                log.frames.append(
                    Frame(t, img[::record_downsample, ::record_downsample].copy(), dets, tp, truth, b)
                )
        while pending and pending[0][0] <= t:
            _, stamp, dets, pose = pending.pop(0)
            perc.tracker.update(stamp, dets, pose)
        tracks: list[TrackEstimate] = perc.tracker.estimates(t)

        # ---------------------------------------------------------- acoustic cueing
        if acoustic is not None and t >= next_acoustic:
            next_acoustic += 1.0 / cfg.acoustic.rate_hz
            acoustic_pending += [(t + cfg.acoustic.latency_s, f) for f in acoustic.sense(t, intruders)]
        while acoustic_pending and acoustic_pending[0][0] <= t:
            acoustic_fixes.append(acoustic_pending.pop(0)[1])
        acoustic_fixes = [f for f in acoustic_fixes if t - f.t < cfg.acoustic.cue_memory_s]
        cues = [_cue_estimate(f, st.pos[2], -(k + 1)) for k, f in enumerate(acoustic_fixes)]

        # ---------------------------------------------------------- decide
        ctx = Context(t=t, state=st, vehicle=vehicle, tracks=tracks, cues=cues)
        ctx_prev = ctx
        while cmds and cmds[0][0] <= t:
            _, text = cmds.pop(0)
            emergency = parse_emergency(text)
            if emergency is not None:
                executive.abort(emergency, ctx, f"operator: '{text}'")
        link_ok = not any(e.start_s <= t < e.start_s + e.duration_s for e in scenario.link_loss)
        monitor.check_vehicle(st, link_ok)
        if monitor.latched is not SafetyRequest.NONE:
            executive.abort(monitor.latched, ctx, monitor.reasons[-1])
        sp = executive.step(ctx)
        dec = monitor.filter(st, sp, tracks + cues, executive.low_altitude_ok)
        if dec.setpoint is not None:
            vehicle.set_setpoint(dec.setpoint)

        # ---------------------------------------------------------- log
        s = world.quad.state
        log.t.append(t)
        log.pos.append(s.pos.copy())
        log.vel.append(s.vel.copy())
        log.yaw.append(st.yaw)
        log.sp.append(np.array(world.setpoint.pos, dtype=float))
        log.battery_wh.append(world.battery_wh)
        log.behaviour.append(executive.current.name if executive.current else "done")
        log.interventions.append(",".join(dec.interventions))
        if intruders:
            tpos = intruders[0][1]
            log.target.append(tpos.copy())
            log.target_in_fov.append(
                any(
                    perc.camera.in_view(perc.camera.pose(s.pos, s.quat, tilt, off), tpos)
                    for off in cfg.camera.yaw_offsets_deg
                )
            )
            follow = getattr(executive.current, "target_id", None)
            chosen = [tr for tr in tracks if tr.ident == follow] or sorted(
                tracks, key=lambda tr: float(np.linalg.norm(tr.pos - tpos))
            )
        else:
            log.target.append(np.full(3, np.nan))
            log.target_in_fov.append(False)
            chosen = tracks
        if chosen:
            log.track.append(chosen[0].pos.copy())
            log.track_std.append(chosen[0].pos_std)
        else:
            log.track.append(np.full(3, np.nan))
            log.track_std.append(float("nan"))
        log.n_confirmed.append(len(tracks))
        for tr in tracks:
            confirmed_ids.add(tr.ident)
            if any(float(np.linalg.norm(tr.pos - p)) < 15.0 for _, p, _ in intruders):
                matched_ids.add(tr.ident)

        if executive.finished and not world.armed:
            break

    return RunResult(
        plan=plan,
        report=report,
        log=log,
        events=executive.events,
        safety_counts=monitor.counts,
        safety_reasons=monitor.reasons,
        false_tracks=perc.tracker.false_tracks_spawned,
        false_confirmed=len(confirmed_ids - matched_ids),
        energy_used_wh=world.energy_used_wh,
        landed=not world.armed and world.quad.on_ground,
        completed=executive.finished and executive.aborted is None,
        scenario=scenario,
        config=cfg,
    )
