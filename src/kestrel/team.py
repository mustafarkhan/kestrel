"""Several observers flying one mission together, with one fused air picture.

What changes with a second observer, and why it matters for a sensor node:

* **Range.** One camera measures bearing well and range poorly (it guesses range from an
  assumed target size). Two cameras looking at the same aircraft from different places
  give range by triangulation. Here both observers' detections feed a single tracker at
  a fusion node, and the tracker's filter does the triangulation: each detection is a
  bearing from a different place, so the track's range error collapses wherever the
  two lines of sight cross at a useful angle.
* **Coverage.** The work is split (a survey area into strips, standby stations spread
  out), so the team covers more sky, and contacts are shared: one observer follows a
  drone while the other keeps its own sector.
* **Each other.** Teammates are cooperative traffic. Their positions are known over the
  datalink, so the safety monitor keeps them apart like any other aircraft, and tracks
  that sit on a teammate are recognised as friendly and not reported as intruders.

Everything else (executive, safety monitor, detector, classifier, acoustic network) is
the same code as for one observer. The datalink between observers is modelled as
reliable with the camera latency; its loss is not.
"""

from __future__ import annotations

import math
from collections import Counter

import numpy as np

from kestrel.config import Config, PointSite, RectSite, Scenario
from kestrel.geofence import Geofence
from kestrel.interop.sapient import SapientExporter
from kestrel.mission.executive import Context, Event, Executive, declared_drone
from kestrel.mission.geometry import resolve_xy
from kestrel.mission.llm import ChatBackend
from kestrel.mission.schema import Mission, StandbyTask, SurveyTask, Task
from kestrel.perception.acoustic import AcousticFix, AcousticNetwork
from kestrel.perception.camera import CameraPose
from kestrel.perception.detector import Detection
from kestrel.perception.tracker import MultiTargetTracker, TrackEstimate
from kestrel.runtime import (
    FlightLog,
    Perception,
    RunResult,
    _cue_estimate,
    _explained,
    _on_bird,
    _span_yaw,
    plan_and_validate,
    same_bearing,
)
from kestrel.safety.monitor import SafetyMonitor, SafetyRequest
from kestrel.sim.world import SimWorld
from kestrel.vehicle.sim_vehicle import SimVehicle

TEAMMATE_SPAN_M = 0.45  # how big an observer looks in a teammate's camera
FRIENDLY_RADIUS_M = 8.0  # a track this close to a teammate's reported position is that teammate
HOME_SPACING_M = 15.0
ASSIST_RADIUS_M = 120.0
STATION_OFFSETS = [(0.0, 0.0), (70.0, 30.0), (-70.0, 30.0), (0.0, 70.0)]


def split_mission(mission: Mission, scenario: Scenario, k: int, n: int) -> tuple[Mission, Scenario]:
    """Observer ``k`` of ``n``: its share of the work and its own home."""
    world = scenario.world
    sites = dict(world.sites)
    tasks: list[Task] = []
    for task in mission.tasks:
        if isinstance(task, SurveyTask) and isinstance(sites.get(task.area), RectSite):
            site = sites[task.area]
            assert isinstance(site, RectSite)
            (x0, y0), (x1, y1) = site.min_xy, site.max_xy
            w = (x1 - x0) / n
            name = f"{task.area}__{k}"
            sites[name] = RectSite(min_xy=(x0 + k * w, y0), max_xy=(x0 + (k + 1) * w, y1))
            tasks.append(task.model_copy(update={"area": name}))
        elif isinstance(task, StandbyTask):
            xy = resolve_xy(task.at, world)
            dx, dy = STATION_OFFSETS[k % len(STATION_OFFSETS)]
            name = f"station__{k}"
            sites[name] = PointSite(xy=(float(xy[0] + dx), float(xy[1] + dy)))
            tasks.append(task.model_copy(update={"at": name}))
        else:
            tasks.append(task)
    hx, hy, hz = world.home
    offset = (k - (n - 1) / 2.0) * HOME_SPACING_M
    new_world = world.model_copy(update={"sites": sites, "home": (hx + offset, hy, hz)})
    return mission.model_copy(update={"tasks": tasks}), scenario.model_copy(update={"world": new_world})


class _Observer:
    def __init__(self, k: int, cfg: Config, scenario: Scenario, mission: Mission, avoidance: bool) -> None:
        self.k = k
        self.scenario = scenario
        self.world = SimWorld(cfg, scenario)
        self.vehicle = SimVehicle(self.world, cfg.state_noise, seed=scenario.seed + 100 * k)
        self.perc = Perception(cfg, scenario.seed, scenario.world.surface)  # same sky for everyone
        self.perc.renderer.rng = np.random.default_rng(scenario.seed + 2000 + 100 * k)  # own sensor noise
        self.executive = Executive(mission, cfg, scenario.world, avoidance=avoidance)
        # Cross-cueing: tracks are shared, so an observer turns its camera to a contact its
        # teammate holds from further away than it would for its own, to give the fusion node
        # a second line of sight. Range then comes from triangulation, not an assumed size.
        self.executive.watch_radius_m = ASSIST_RADIUS_M
        self.monitor = SafetyMonitor(cfg.safety, cfg.vehicle, scenario.world, separation=avoidance)
        self.tilt = cfg.camera.pitch_up_deg
        self.gimbal_rng = np.random.default_rng(scenario.seed + 77 + 100 * k)
        self.ctx: Context | None = None
        self.log = FlightLog()

    @property
    def done(self) -> bool:
        return self.executive.finished and not self.world.armed


def run_team(
    command: str | Mission,
    cfg: Config,
    scenario: Scenario,
    n: int = 2,
    backend: ChatBackend | None = None,
    avoidance: bool = True,
    sapient: SapientExporter | None = None,
    record_every: int = 0,
) -> RunResult:
    plan, report = plan_and_validate(command, cfg, scenario, backend)
    if report is None or not report.ok:
        return RunResult(plan, report, FlightLog(), [], Counter(), [], 0, 0.0, True, False, scenario, cfg)
    base = report.mission if avoidance else report.mission.model_copy(update={"on_contact": "report"})
    obs: list[_Observer] = []
    for k in range(n):
        m_k, sc_k = split_mission(base, scenario, k, n)
        obs.append(_Observer(k, cfg, sc_k, m_k, avoidance))
    # shared state between executives: who follows what, which reports were investigated
    shared_investigated = obs[0].executive.investigated
    shared_aircraft = obs[0].executive.handled_aircraft
    for o in obs[1:]:
        o.executive.investigated = shared_investigated
        o.executive.handled_aircraft = shared_aircraft  # an aircraft one follows, the other leaves

    fusion = MultiTargetTracker(cfg.tracker, obs[0].perc.camera)
    acoustic = (
        AcousticNetwork(cfg.acoustic, scenario.world, np.random.default_rng(scenario.seed + 91))
        if cfg.acoustic.enabled
        else None
    )
    fence = Geofence(scenario.world.fence)
    next_acoustic = 0.0
    acoustic_pending: list[tuple[float, AcousticFix]] = []
    acoustic_fixes: list[AcousticFix] = []
    pending: list[tuple[float, float, list[Detection], CameraPose]] = []
    substeps = max(1, round(1.0 / (cfg.autonomy_rate_hz * cfg.physics_dt)))
    cam_period = 1.0 / cfg.camera.rate_hz
    next_cam = 0.0
    jitter = math.radians(cfg.camera.gimbal_jitter_deg)
    span = {i: c.span_m for i, c in enumerate(scenario.intruders)}
    log = obs[0].log  # the team log: observer 0's flight, separation measured for every observer
    team_sep: list[float] = []
    track_errors: list[float] = []
    matched: set[int] = set()
    confirmed: set[int] = set()
    declared: set[int] = set()
    bird_ids: set[int] = set()
    friendly: set[int] = set()
    first_cue_t: float | None = None
    first_confirm_t: float | None = None
    first_track_t: float | None = None
    first_entry_t: float | None = None
    t = 0.0

    while t < scenario.max_duration_s:
        for o in obs:
            if not o.done:
                for _ in range(substeps):
                    o.world.step(cfg.physics_dt)
            else:  # parked: keep the clock with the others
                o.world.t += substeps * cfg.physics_dt
        t = obs[0].world.t
        world0 = obs[0].world
        intruders = world0.intruder_states()
        birds = world0.bird_states()
        states = [o.vehicle.state() for o in obs]
        truth = [o.world.quad.state for o in obs]
        if first_entry_t is None and any(fence.signed_distance(p) > 0.0 for _, p, _ in intruders):
            first_entry_t = t

        # ---------------------------------------------------------- perception (every camera)
        for o in obs:
            want = o.executive.gimbal_tilt(o.ctx) if o.ctx is not None else None
            want = cfg.camera.pitch_up_deg if want is None else want
            step = cfg.camera.gimbal_rate_deg_s / cfg.autonomy_rate_hz
            o.tilt = float(np.clip(o.tilt + np.clip(want - o.tilt, -step, step), cfg.camera.gimbal_min_deg,
                                   cfg.camera.gimbal_max_deg))  # fmt: skip
        if t >= next_cam:
            next_cam += cam_period
            for o, st, s in zip(obs, states, truth, strict=True):
                if not o.world.armed:
                    continue
                targets = [(p, span[i], _span_yaw(v)) for i, p, v in intruders]
                targets += [(q.pos, TEAMMATE_SPAN_M, 0.0) for j, q in enumerate(truth) if j != o.k]
                for off in cfg.camera.yaw_offsets_deg:
                    pose_c = o.perc.camera.pose(s.pos, s.quat, o.tilt, off).jittered(o.gimbal_rng, jitter)
                    est_c = o.perc.camera.pose(st.pos, st.quat, o.tilt, off)
                    img = o.perc.renderer.render(pose_c, targets, cfg.camera.noise_sigma, birds)
                    dets = o.perc.detector.detect(img, est_c)
                    pending.append((t + cfg.camera.latency_s, t, dets, est_c))
            pending.sort(key=lambda x: x[0])
        while pending and pending[0][0] <= t:
            _, stamp, dets, pose = pending.pop(0)
            fusion.update(stamp, dets, pose)
        all_tracks = fusion.estimates(t)
        mates = [st.pos for st in states]
        for tr in all_tracks:
            # A teammate seen by another observer: its range from size is wrong (it is bigger than
            # the assumed drone), its bearing is right, so match by bearing as well as position.
            if any(float(np.linalg.norm(tr.pos - p)) < FRIENDLY_RADIUS_M for p in mates) or any(
                same_bearing(a, tr.pos, b) for i, a in enumerate(mates) for j, b in enumerate(mates) if i != j
            ):
                friendly.add(tr.ident)
        tracks = [tr for tr in all_tracks if tr.ident not in friendly]

        # ---------------------------------------------------------- acoustic cueing
        if acoustic is not None and t >= next_acoustic:
            next_acoustic += 1.0 / cfg.acoustic.rate_hz
            acoustic_pending += [(t + cfg.acoustic.latency_s, f) for f in acoustic.sense(t, intruders)]
        released: list[AcousticFix] = []
        while acoustic_pending and acoustic_pending[0][0] <= t:
            released.append(acoustic_pending.pop(0)[1])
        link_ok = not any(e.start_s <= t < e.start_s + e.duration_s for e in scenario.link_loss)
        if link_ok:
            acoustic_fixes += released
            if released and first_cue_t is None:
                first_cue_t = t
        acoustic_fixes = [f for f in acoustic_fixes if t - f.t < cfg.acoustic.cue_memory_s]
        live_fixes = [f for f in acoustic_fixes if not _explained(f, tracks, t)]

        # ---------------------------------------------------------- decide (each observer)
        for o, st in zip(obs, states, strict=True):
            # contacts a teammate already handles, including one it took up earlier this tick
            handled = set().union(*(x.executive.handled_contacts for x in obs if x is not o))
            cues = [_cue_estimate(f, st.pos[2], -(k + 1)) for k, f in enumerate(live_fixes)]
            # teammates as cooperative traffic: exact position over the datalink
            coop = []
            for j, q in enumerate(states):
                if j == o.k or not obs[j].world.armed:
                    continue
                cov = np.diag([1.0, 1.0, 1.0, 0.25, 0.25, 0.25])
                coop.append(TrackEstimate(-1000 - j, q.pos.copy(), q.vel.copy(), cov, True, 100, t))
            o.executive.handled_contacts |= handled - {getattr(o.executive.current, "target_id", None)}
            ctx = Context(t=t, state=st, vehicle=o.vehicle, tracks=tracks, cues=cues)
            o.ctx = ctx
            o.monitor.check_vehicle(st, link_ok)
            if o.monitor.latched is not SafetyRequest.NONE:
                o.executive.abort(o.monitor.latched, ctx, o.monitor.reasons[-1])
            if o.done:
                continue
            sp = o.executive.step(ctx)
            # The monitor keeps clear of every fused track, friendly-looking or not: a label can be
            # wrong (an intruder in line with a teammate), and keeping clear of a teammate is right anyway.
            dec = o.monitor.filter(st, sp, all_tracks + cues + coop, o.executive.low_altitude_ok)
            if dec.setpoint is not None:
                o.vehicle.set_setpoint(dec.setpoint)
            if o.k == 0:
                log.interventions.append(",".join(dec.interventions))
        if obs[0].done:
            log.interventions.append("")
        if sapient is not None:
            sapient.set_link(t, link_ok)
            cur = obs[0].executive.current
            mode = cur.name if cur else "idle"
            home_alt = scenario.world.home[2]
            sapient.step(t, states[0].pos, mode, tracks, released, acoustic is not None, home_alt)

        # ---------------------------------------------------------- log (team view)
        s0 = truth[0]
        log.t.append(t)
        log.pos.append(s0.pos.copy())
        log.vel.append(s0.vel.copy())
        log.yaw.append(states[0].yaw)
        log.sp.append(np.array(world0.setpoint.pos, dtype=float))
        log.battery_wh.append(world0.battery_wh)
        cur0 = obs[0].executive.current
        log.behaviour.append(cur0.name if cur0 else "done")
        tpos = intruders[0][1] if intruders else None
        log.target.append(tpos.copy() if tpos is not None else np.full(3, np.nan))
        log.target_in_fov.append(False)
        near = (
            min((float(np.linalg.norm(tr.pos - tpos)), tr) for tr in tracks)
            if tracks and tpos is not None
            else None
        )
        if near is not None:
            log.track.append(near[1].pos.copy())
            log.track_std.append(near[1].pos_std)
        else:
            log.track.append(np.full(3, np.nan))
            log.track_std.append(float("nan"))
        log.n_confirmed.append(len(tracks))
        log.birds.append(np.array([b.pos for b in birds]).reshape(-1, 3))
        log.cues.append(np.array([f.xy for f in acoustic_fixes]).reshape(-1, 2))
        flying = [q.pos for o, q in zip(obs, truth, strict=True) if o.world.armed]
        log.intruder_dist.append(
            min((float(np.linalg.norm(p - q)) for _, p, _ in intruders for q in flying), default=math.nan)
        )
        log.nearest_in_fov.append(False)
        log.nearest_tracked.append(False)
        log.bird_dist.append(
            min((float(np.linalg.norm(b.pos - q)) for b in birds for q in flying), default=math.inf)
        )
        if len(flying) > 1:
            team_sep.append(
                min(float(np.linalg.norm(a - b)) for i, a in enumerate(flying) for b in flying[i + 1 :])
            )
        log.link_ok.append(link_ok)
        log.team_pos.append(np.array([q.pos for q in truth]))
        log.fused.append(
            np.array(
                [
                    tr.pos
                    for tr in tracks
                    if any(float(np.linalg.norm(tr.pos - p)) < 15.0 for _, p, _ in intruders)
                ]
            ).reshape(-1, 3)
        )
        log.on_intruder.append(
            any(
                declared_drone(tr) and any(float(np.linalg.norm(tr.pos - p)) < 15.0 for _, p, _ in intruders)
                for tr in tracks
            )
        )
        for tr in tracks:
            confirmed.add(tr.ident)
            if declared_drone(tr):
                declared.add(tr.ident)
            errs = [float(np.linalg.norm(tr.pos - p)) for _, p, _ in intruders]
            if errs and min(errs) < 15.0:
                matched.add(tr.ident)
                if first_confirm_t is None and tr.hits >= 8 and declared_drone(tr):
                    first_confirm_t = t
                if first_track_t is None and tr.hits >= 8 and (tr.p_uav is None or tr.p_uav > 0.4):
                    first_track_t = t
            elif birds and any(_on_bird(q.pos, tr.pos, birds) for q in states):
                bird_ids.add(tr.ident)
        for _, p, _ in intruders:  # how well the team knows where each intruder is
            on_it = [tr for tr in tracks if any(same_bearing(q.pos, tr.pos, p) for q in states)]
            if on_it:
                track_errors.append(min(float(np.linalg.norm(tr.pos - p)) for tr in on_it))

        if all(o.done for o in obs):
            break

    events: list[Event] = []
    for o in obs:
        events += [Event(e.t, e.kind, f"[observer {o.k}] {e.detail}") for e in o.executive.events]
    events.sort(key=lambda e: e.t)
    counts: Counter[str] = Counter()
    reasons: list[str] = []
    for o in obs:
        counts += o.monitor.counts
        reasons += o.monitor.reasons
    return RunResult(
        plan=plan,
        report=report,
        log=log,
        events=events,
        safety_counts=counts,
        safety_reasons=reasons,
        false_tracks=fusion.false_tracks_spawned,
        false_confirmed=len((confirmed & declared) - matched),
        bird_tracks=len(bird_ids - matched),
        birds_declared_drone=len((bird_ids & declared) - matched),
        drones_declared=len(matched & declared),
        drones_matched=len(matched),
        first_cue_t=first_cue_t,
        first_confirm_t=first_confirm_t,
        first_track_t=first_track_t,
        first_entry_t=first_entry_t,
        reports_buffered=sapient.late_deliveries if sapient is not None else 0,
        energy_used_wh=max(o.world.energy_used_wh for o in obs),
        landed=all(not o.world.armed and o.world.quad.on_ground for o in obs),
        completed=all(o.executive.finished and o.executive.aborted is None for o in obs),
        scenario=obs[0].scenario,  # observer 0's home, for the landing metrics
        config=cfg,
        observers=n,
        team_min_separation_m=min(team_sep) if team_sep else None,
        track_errors=track_errors,
    )
