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
from kestrel.geofence import Geofence
from kestrel.geometry import Vec
from kestrel.interop.sapient import SapientExporter
from kestrel.mission.executive import Context, Event, Executive, declared_drone
from kestrel.mission.geometry import resolve_xy
from kestrel.mission.llm import ChatBackend, MissionPlanner, PlanResult
from kestrel.mission.parser import RuleParser, parse_emergency, parse_investigate
from kestrel.mission.schema import Mission
from kestrel.mission.validator import ValidationReport, validate
from kestrel.perception.acoustic import AcousticFix, AcousticNetwork
from kestrel.perception.camera import Camera, CameraPose
from kestrel.perception.detector import Detection, SkyTargetDetector
from kestrel.perception.render import SkyRenderer
from kestrel.perception.tracker import MultiTargetTracker, TrackEstimate
from kestrel.safety.monitor import SafetyMonitor, SafetyRequest
from kestrel.sim.birds import BirdState
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
    birds_px: list[tuple[float, float]] = field(default_factory=list)
    p_uav: dict[int, float] = field(default_factory=dict)  # classifier output per track id


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
    bird_dist: list[float] = field(default_factory=list)  # to the nearest bird (inf if none)
    intruder_dist: list[float] = field(default_factory=list)  # to the nearest intruder (nan if none)
    nearest_in_fov: list[bool] = field(default_factory=list)  # nearest intruder inside a camera's view
    nearest_tracked: list[bool] = field(default_factory=list)  # a confirmed track within 15 m of it
    on_intruder: list[bool] = field(default_factory=list)  # a track reported as a drone is on an intruder
    team_pos: list[Vec] = field(default_factory=list)  # every observer's position (n x 3), team runs
    fused: list[Vec] = field(default_factory=list)  # positions of tracks on intruders (n x 3), team runs
    link_ok: list[bool] = field(default_factory=list)
    birds: list[Vec] = field(default_factory=list)  # bird positions (n x 3) per tick, for replay
    cues: list[Vec] = field(default_factory=list)  # live acoustic fixes (n x 2) per tick, for replay
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
            "bird_dist": np.array(self.bird_dist),
            "intruder_dist": np.array(self.intruder_dist),
            "nearest_in_fov": np.array(self.nearest_in_fov),
            "nearest_tracked": np.array(self.nearest_tracked),
            "on_intruder": np.array(self.on_intruder),
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
    false_confirmed: int = 0  # tracks declared drones that never matched a real aircraft (false alarms)
    first_entry_t: float | None = None  # first time an intruder was inside the geofence
    first_cue_t: float | None = None  # first ground-sensor report of an intruder that reached the observer
    first_track_t: float | None = None  # first track on an intruder (8+ hits) not called a bird
    first_confirm_t: float | None = None  # first track on an intruder reported as a drone (8+ hits)
    reports_buffered: int = 0  # SAPIENT messages held during link loss and sent afterwards
    observers: int = 1
    team_min_separation_m: float | None = None  # closest two observers came to each other
    track_errors: list[float] = field(default_factory=list)  # position error of tracks on intruders
    bird_tracks: int = 0  # confirmed tracks on birds
    birds_declared_drone: int = 0  # of those, reported as a drone at some point
    drones_declared: int = 0  # confirmed tracks on intruders that were reported as a drone
    drones_matched: int = 0  # confirmed tracks on intruders
    track_truth: dict[int, str] = field(
        default_factory=dict
    )  # "drone" / "bird" / "false" per confirmed track
    track_p_uav: dict[int, list[tuple[float, float]]] = field(default_factory=dict)  # classifier history

    @property
    def flown(self) -> bool:
        return len(self.log.t) > 0


class Perception:
    def __init__(self, cfg: Config, seed: int, surface: str = "land") -> None:
        self.camera = Camera(cfg.camera, cfg.vehicle.camera_offset)
        self.renderer = SkyRenderer(self.camera, seed=seed, surface=surface)
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


def same_bearing(observer: Vec, a: Vec, b: Vec, max_deg: float = 3.0) -> bool:
    """Do ``a`` and ``b`` lie in the same direction from ``observer``?"""
    d, e = a - observer, b - observer
    nd, ne = float(np.linalg.norm(d)), float(np.linalg.norm(e))
    if nd < 1e-6 or ne < 1e-6:
        return False
    return float(d @ e) / (nd * ne) > math.cos(math.radians(max_deg))


def _on_bird(observer: Vec, track_pos: Vec, birds: list[BirdState], max_deg: float = 3.0) -> bool:
    """Is the track looking at a bird? Matched by bearing from the observer, not position:
    the range of a bird is estimated from an assumed drone size, so it is wrong by the ratio
    of the two sizes, while its bearing is right."""
    return any(same_bearing(observer, track_pos, b.pos, max_deg) for b in birds)


def _explained(fix: AcousticFix, tracks: list[TrackEstimate], t: float, gate: float = 9.21) -> bool:
    """Is this acoustic fix the aircraft the camera is already tracking?

    A cue is for aircraft the camera has *not* got. Once a fresh visual track lies within the
    fix's horizontal uncertainty (chi-square gate, 2 DoF, p = 0.99), the track supersedes it:
    it carries altitude and velocity, which sound cannot, so keeping the cue as well would
    only add an altitude-unknown column on top of an aircraft whose height is known.
    """
    for tr in tracks:
        if t - tr.last_hit_t > 1.0:
            continue
        d = tr.pos[0:2] - fix.xy
        s = fix.cov_xy + tr.cov[0:2, 0:2]
        if float(d @ np.linalg.solve(s, d)) < gate:
            return True
    return False


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
    sapient: SapientExporter | None = None,
    sapient_tasks: list[tuple[float, dict[str, object]]] | None = None,
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
    perc = Perception(cfg, scenario.seed, scenario.world.surface)
    mission = report.mission if avoidance else report.mission.model_copy(update={"on_contact": "report"})
    executive = Executive(mission, cfg, scenario.world, avoidance=avoidance)
    monitor = SafetyMonitor(cfg.safety, cfg.vehicle, scenario.world, separation=avoidance)
    matched_ids: set[int] = set()
    confirmed_ids: set[int] = set()
    declared_ids: set[int] = set()
    bird_ids: set[int] = set()
    p_hist: dict[int, list[tuple[float, float]]] = {}
    track_errors: list[float] = []
    acoustic = (
        AcousticNetwork(cfg.acoustic, scenario.world, np.random.default_rng(scenario.seed + 91))
        if cfg.acoustic.enabled and perception_enabled
        else None
    )
    next_acoustic = 0.0
    acoustic_pending: list[tuple[float, AcousticFix]] = []
    acoustic_fixes: list[AcousticFix] = []
    cmds = sorted(operator_commands or [])
    c2_tasks = sorted(sapient_tasks or [], key=lambda x: x[0])
    rules = RuleParser(list(scenario.world.sites))
    first_cue_t: float | None = None
    first_confirm_t: float | None = None
    first_track_t: float | None = None
    first_entry_t: float | None = None
    fence = Geofence(scenario.world.fence)

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
        birds = world.bird_states()
        if first_entry_t is None and any(fence.signed_distance(p) > 0.0 for _, p, _ in intruders):
            first_entry_t = t

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
                img_c = perc.renderer.render(pose_c, targets, cfg.camera.noise_sigma, birds)
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
                bpx = [uv[:2] for bs in birds if (uv := perc.camera.project(true_pose, bs.pos))]
                b = executive.current.name if executive.current else "idle"
                pu = {e.ident: e.p_uav for e in ests if e.p_uav is not None}
                log.frames.append(
                    Frame(
                        t, img[::record_downsample, ::record_downsample].copy(), dets, tp, truth, b, bpx, pu
                    )
                )
        while pending and pending[0][0] <= t:
            _, stamp, dets, pose = pending.pop(0)
            perc.tracker.update(stamp, dets, pose)
        tracks: list[TrackEstimate] = perc.tracker.estimates(t)

        # ---------------------------------------------------------- acoustic cueing
        if acoustic is not None and t >= next_acoustic:
            next_acoustic += 1.0 / cfg.acoustic.rate_hz
            acoustic_pending += [(t + cfg.acoustic.latency_s, f) for f in acoustic.sense(t, intruders)]
        released: list[AcousticFix] = []
        while acoustic_pending and acoustic_pending[0][0] <= t:
            released.append(acoustic_pending.pop(0)[1])
        link_ok = not any(e.start_s <= t < e.start_s + e.duration_s for e in scenario.link_loss)
        if link_ok:  # ground-sensor cues reach the observer over the same link
            acoustic_fixes += released
            if released and first_cue_t is None:
                first_cue_t = t
        acoustic_fixes = [f for f in acoustic_fixes if t - f.t < cfg.acoustic.cue_memory_s]
        cues = [
            _cue_estimate(f, st.pos[2], -(k + 1))
            for k, f in enumerate(acoustic_fixes)
            if not _explained(f, tracks, t)
        ]

        # ---------------------------------------------------------- decide
        ctx = Context(t=t, state=st, vehicle=vehicle, tracks=tracks, cues=cues)
        ctx_prev = ctx
        while link_ok and cmds and cmds[0][0] <= t:  # commands wait for the link
            _, text = cmds.pop(0)
            emergency = parse_emergency(text)
            if emergency is not None:
                executive.abort(emergency, ctx, f"operator: '{text}'")
                continue
            loc = parse_investigate(text, rules)
            if loc is not None:
                executive.request_investigate(resolve_xy(loc, scenario.world), t, "operator")
        while link_ok and c2_tasks and c2_tasks[0][0] <= t:
            _, msg = c2_tasks.pop(0)
            look = sapient.look_at(msg) if sapient is not None else None
            if look is not None and sapient is not None:
                task_id, xy = look
                ok = executive.mission is not None and not executive.aborted
                sapient.task_ack(t, task_id, ok, "" if ok else "mission aborted")
                if ok:
                    executive.request_investigate(xy, t, "C2 tasking (SAPIENT)")
        monitor.check_vehicle(st, link_ok)
        if monitor.latched is not SafetyRequest.NONE:
            executive.abort(monitor.latched, ctx, monitor.reasons[-1])
        sp = executive.step(ctx)
        dec = monitor.filter(st, sp, tracks + cues, executive.low_altitude_ok)
        if dec.setpoint is not None:
            vehicle.set_setpoint(dec.setpoint)
        if sapient is not None:
            sapient.set_link(t, link_ok)
            mode = executive.current.name if executive.current else "idle"
            sapient.step(t, st.pos, mode, tracks, released, acoustic is not None, scenario.world.home[2])

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
        log.birds.append(np.array([b.pos for b in birds]).reshape(-1, 3))
        log.cues.append(np.array([f.xy for f in acoustic_fixes]).reshape(-1, 2))
        log.intruder_dist.append(
            min((float(np.linalg.norm(p - s.pos)) for _, p, _ in intruders), default=math.nan)
        )
        if intruders:
            npos = min((p for _, p, _ in intruders), key=lambda p: float(np.linalg.norm(p - s.pos)))
            log.nearest_in_fov.append(
                any(
                    perc.camera.in_view(perc.camera.pose(s.pos, s.quat, tilt, off), npos)
                    for off in cfg.camera.yaw_offsets_deg
                )
            )
            log.nearest_tracked.append(any(float(np.linalg.norm(tr.pos - npos)) < 15.0 for tr in tracks))
        else:
            log.nearest_in_fov.append(False)
            log.nearest_tracked.append(False)
        log.bird_dist.append(min((float(np.linalg.norm(b.pos - s.pos)) for b in birds), default=math.inf))
        log.link_ok.append(link_ok)
        log.on_intruder.append(
            any(
                declared_drone(tr) and any(float(np.linalg.norm(tr.pos - p)) < 15.0 for _, p, _ in intruders)
                for tr in tracks
            )
        )
        for tr in tracks:
            confirmed_ids.add(tr.ident)
            if tr.p_uav is not None:
                p_hist.setdefault(tr.ident, []).append((t, tr.p_uav))
            if declared_drone(tr):
                declared_ids.add(tr.ident)
            if any(float(np.linalg.norm(tr.pos - p)) < 15.0 for _, p, _ in intruders):
                matched_ids.add(tr.ident)
                if first_confirm_t is None and tr.hits >= 8 and declared_drone(tr):
                    first_confirm_t = t
                if first_track_t is None and tr.hits >= 8 and (tr.p_uav is None or tr.p_uav > 0.4):
                    first_track_t = t
            elif birds and _on_bird(st.pos, tr.pos, birds):
                bird_ids.add(tr.ident)
        for _, p, _ in intruders:  # how well the observer knows where each intruder is
            on_it = [tr for tr in tracks if same_bearing(st.pos, tr.pos, p)]
            if on_it:
                track_errors.append(min(float(np.linalg.norm(tr.pos - p)) for tr in on_it))

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
        false_confirmed=len((confirmed_ids & declared_ids) - matched_ids),
        bird_tracks=len(bird_ids - matched_ids),
        birds_declared_drone=len((bird_ids & declared_ids) - matched_ids),
        drones_declared=len(matched_ids & declared_ids),
        drones_matched=len(matched_ids),
        track_truth={
            i: "drone" if i in matched_ids else "bird" if i in bird_ids else "false" for i in confirmed_ids
        },
        track_p_uav=p_hist,
        first_cue_t=first_cue_t,
        first_entry_t=first_entry_t,
        first_confirm_t=first_confirm_t,
        first_track_t=first_track_t,
        reports_buffered=sapient.late_deliveries if sapient is not None else 0,
        track_errors=track_errors,
        energy_used_wh=world.energy_used_wh,
        landed=not world.armed and world.quad.on_ground,
        completed=executive.finished and executive.aborted is None,
        scenario=scenario,
        config=cfg,
    )
