"""A SAPIENT fusion node: one recognised air picture (RAP) from many sensor nodes.

It reads SAPIENT messages only (registrations and detection reports), so the same code
runs on simulated sensors and on recorded traffic from real ones. Per report it:

1. **associates** the report with a picture track: first through the sensor's own object
   id (a radar or RF track keeps feeding the same picture track), otherwise by a
   chi-square gate on position using the report's stated errors. A track never takes two
   live object ids from the same tracking sensor, because that sensor says they are two
   objects;
2. **fuses kinematics** with a constant-velocity Kalman filter, using each report's
   one-sigma errors as its measurement noise. An acoustic or RF fix with no altitude
   carries a vertical error spanning the altitude band, so it constrains only the
   horizontal position, which is all it knows;
3. **fuses class evidence** as a drone-versus-bird log-likelihood ratio. Each sensor
   node's opinion is the mean log-odds of its reports, weighted by how far that sensor is
   trusted (a radar's micro-Doppler score counts for less than a camera's or an RF
   detector's) and by at most three independent looks: one sensor's reports on one
   object are correlated, so a sensor that talks a lot cannot become certain on its own;
4. **removes duplicates**: two picture tracks that stay statistically indistinguishable
   for two seconds, and that no tracking sensor says are different objects, are merged;
5. **identifies**, using NATO standard identities (APP-6): *friend* only for a drone
   whose Remote ID serial is on our whitelist **and** whose broadcast position is
   corroborated by a non-cooperative sensor (otherwise *assumed friend*, so a spoofed
   serial cannot make a phantom friendly); *neutral* for a compliant civil drone or a
   bird; *suspect* for an unidentified drone inside or heading into a protected area;
   *unknown* otherwise. A serial broadcast from two places at once is a clone: no copy
   is shown as friend, and the uncorroborated copies are suspect. **Hostile is never set
   automatically**: it is a human decision, recorded with :meth:`FusionNode.declare`.

It also decides which track most needs a closer look and writes a SAPIENT ``LOOK_AT``
task for the airborne camera node: the cue-to-confirm loop run from the fusion node.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from kestrel.config import CircleSite, PointSite, Site, WorldConfig
from kestrel.geofence import Geofence
from kestrel.geometry import Vec
from kestrel.interop.sapient import SapientExporter

Message = dict[str, Any]

GATE_NEW = 11.34  # chi-square, 3 dof, 99%
GATE_KNOWN = 25.0  # looser for a report whose sensor already ties it to this track
MERGE_GATE = 7.81  # chi-square, 3 dof, 95%
MERGE_AFTER_S = 2.0
LLR_MAX = 8.0
MAX_LOOKS = 3.0  # independent looks credited to one sensor node
CLASS_WEIGHT = {
    "NODE_TYPE_RADAR": 0.5,  # micro-Doppler: a noisy opinion
    "NODE_TYPE_CAMERA": 0.6,  # Kestrel's wing-beat classifier: measured, and fallible on gliders
    "NODE_TYPE_PASSIVE_RF": 1.0,  # a control link is strong evidence of a drone
    "NODE_TYPE_ACOUSTIC": 0.6,
}
_I3 = np.eye(3)
IDENTITIES = ("PENDING", "UNKNOWN", "ASSUMED_FRIEND", "FRIEND", "NEUTRAL", "SUSPECT", "HOSTILE")


@dataclass
class FusionOptions:
    accel_noise: float = 3.0  # m/s^2, constant-velocity model
    confirm_updates: int = 3
    coast_s: float = 8.0  # a track with no report for this long is dropped
    p_uav_drone: float = 0.8  # class "UAV" above this
    p_uav_bird: float = 0.2  # class "Bird" below this
    corroborate_s: float = 5.0  # Remote ID needs a non-cooperative report this recent
    suspect_horizon_s: float = 60.0  # heading into a protected area within this time
    look_recent_s: float = 15.0  # a camera saw it this recently: no need to look again
    retask_s: float = 30.0  # do not re-task the same track sooner


@dataclass
class Report:
    t: float
    node_id: str
    node_type: str
    object_id: str
    tracking: bool
    pos: Vec
    r_pos: Vec  # 3x3
    vel: Vec | None = None
    r_vel: Vec | None = None
    p_uav: float | None = None
    remote_id: str | None = None
    rf_protocol: str | None = None


@dataclass
class RapTrack:
    ident: int
    x: Vec
    p: Vec
    t: float
    created_t: float
    updates: int = 0
    sources: dict[tuple[str, str], float] = field(default_factory=dict)
    node_types: set[str] = field(default_factory=set)
    llr: float = 0.0
    evidence: dict[str, tuple[str, int, float]] = field(default_factory=dict)  # node -> (type, n, sum logit)
    remote_id: str | None = None
    rid_pos: Vec | None = None  # last position the drone broadcast, and its stated error
    rid_r: Vec | None = None
    rid_t: float = -math.inf  # when it was last heard
    rf_protocol: str | None = None
    noncoop_t: float = -math.inf  # last non-cooperative report that agreed with the broadcast
    camera_t: float = -math.inf
    identity: str = "PENDING"
    declared: str | None = None
    tasked_t: float = -math.inf
    close: dict[int, float] = field(default_factory=dict)  # other track -> time it became close
    merged_from: list[int] = field(default_factory=list)

    @property
    def p_uav(self) -> float:
        return 1.0 / (1.0 + math.exp(-self.llr))

    @property
    def pos(self) -> Vec:
        return self.x[0:3]

    @property
    def vel(self) -> Vec:
        return self.x[3:6]


def _sigma_cov(loc: Message, default: float) -> Vec:
    s = [float(loc.get(k, default)) for k in ("x_error", "y_error", "z_error")]
    return np.diag(np.maximum(np.array(s), 0.1) ** 2)


def site_disc(site: Site, point_radius: float = 40.0) -> tuple[Vec, float]:
    """A protected site as a disc: points get a default radius, rectangles their circumcircle."""
    if isinstance(site, PointSite):
        return np.array(site.xy, dtype=float), point_radius
    if isinstance(site, CircleSite):
        return np.array(site.center, dtype=float), float(site.radius)
    lo, hi = np.array(site.min_xy, dtype=float), np.array(site.max_xy, dtype=float)
    return 0.5 * (lo + hi), 0.5 * float(np.linalg.norm(hi - lo))


def _logit(p: float) -> float:
    p = min(max(p, 0.01), 0.99)
    return math.log(p / (1.0 - p))


class FusionNode:
    def __init__(
        self,
        geo: SapientExporter,
        world: WorldConfig,
        whitelist: list[str] | None = None,
        protected: dict[str, Site] | None = None,
        opts: FusionOptions | None = None,
    ) -> None:
        self.geo = geo
        self.world = world
        self.fence = Geofence(world.fence)
        self.whitelist = set(whitelist or [])
        self.protected = protected or {}
        self.o = opts or FusionOptions()
        self.node_types: dict[str, str] = {}
        self.node_tracking: dict[str, bool] = {}
        self.tracks: list[RapTrack] = []
        self._by_source: dict[tuple[str, str], RapTrack] = {}
        self._next = 1
        self.merges = 0
        self.t = 0.0
        self.camera_node: str | None = None
        self.reports = 0
        self.node_id = geo.new_node_id()  # the fusion node's own SAPIENT identity, for its tasks

    # ------------------------------------------------------------------ input
    def ingest(self, msg: Message) -> None:
        node = str(msg.get("node_id", ""))
        if "registration" in msg:
            reg = msg["registration"]
            defs = reg.get("node_definition") or [{"node_type": t} for t in reg.get("node_type") or []]
            kind = str(defs[0].get("node_type", "NODE_TYPE_OTHER")) if defs else "NODE_TYPE_OTHER"
            self.node_types[node] = kind
            modes = reg.get("mode_definition") or [{}]
            self.node_tracking[node] = (
                modes[0].get("tracking_type", "TRACKING_TYPE_TRACK") != "TRACKING_TYPE_NONE"
            )
            if kind == "NODE_TYPE_CAMERA" and any(m.get("task") for m in modes):
                self.camera_node = node
            return
        det = msg.get("detection_report")
        if det is None or node not in self.node_types:
            return
        r = self._report(msg, node, det)
        if r is not None:
            self.reports += 1
            self._process(r)

    def _report(self, msg: Message, node: str, det: Message) -> Report | None:
        loc = det.get("location")
        if not isinstance(loc, dict):
            return None
        t = self.geo.sim_time(str(msg["timestamp"]))
        pos = self.geo.to_enu(loc)
        r = Report(
            t=t,
            node_id=node,
            node_type=self.node_types[node],
            object_id=str(det.get("object_id", "")),
            tracking=self.node_tracking.get(node, True),
            pos=pos,
            r_pos=_sigma_cov(loc, 10.0),
        )
        v = det.get("enu_velocity")
        if isinstance(v, dict):
            r.vel = np.array([float(v["east_rate"]), float(v["north_rate"]), float(v.get("up_rate", 0.0))])
            s = [float(v.get(k, 2.0)) for k in ("east_rate_error", "north_rate_error", "up_rate_error")]
            r.r_vel = np.diag(np.maximum(np.array(s), 0.1) ** 2)
        for c in det.get("classification") or []:
            if c.get("type") == "UAV":
                r.p_uav = float(c.get("confidence", 0.5))
        if det.get("id"):
            r.remote_id = str(det["id"])
        for info in det.get("track_info") or []:
            if info.get("type") == "remote_id":
                r.remote_id = str(info.get("value"))
            elif info.get("type") == "rf_protocol":
                r.rf_protocol = str(info.get("value"))
        return r

    # ------------------------------------------------------------------ filter
    def _predict(self, tr: RapTrack, t: float) -> tuple[Vec, Vec]:
        dt = max(0.0, t - tr.t)
        if dt == 0.0:
            return tr.x, tr.p
        f = np.eye(6)
        f[0:3, 3:6] = dt * np.eye(3)
        q1 = np.array([[dt**4 / 4, dt**3 / 2], [dt**3 / 2, dt**2]]) * self.o.accel_noise**2
        return f @ tr.x, f @ tr.p @ f.T + np.kron(q1, _I3)

    def _d2(self, tr: RapTrack, r: Report) -> float:
        x, p = self._predict(tr, r.t)
        s = p[0:3, 0:3] + r.r_pos
        d = r.pos - x[0:3]
        return float(d @ np.linalg.solve(s, d))

    def _conflict(self, tr: RapTrack, r: Report) -> bool:
        """Does a tracking sensor already feed this track with a different, live object?"""
        if not r.tracking:
            return False
        return any(
            node == r.node_id and obj != r.object_id and r.t - last < 2.0
            for (node, obj), last in tr.sources.items()
        )

    def _process(self, r: Report) -> None:
        key = (r.node_id, r.object_id)
        tr = self._by_source.get(key) if r.tracking else None
        if tr is not None and (tr not in self.tracks or self._d2(tr, r) > GATE_KNOWN):
            tr = None
        if tr is None:
            best, best_d = None, GATE_NEW
            for cand in self.tracks:
                if self._conflict(cand, r):
                    continue
                d = self._d2(cand, r)
                if d < best_d:
                    best, best_d = cand, d
            tr = best
        if tr is None:
            tr = self._new(r)
        else:
            self._update(tr, r)
        self._book(tr, r)

    def _new(self, r: Report) -> RapTrack:
        x = np.zeros(6)
        x[0:3] = r.pos
        p = np.zeros((6, 6))
        p[0:3, 0:3] = r.r_pos
        p[3:6, 3:6] = np.eye(3) * 10.0**2
        if r.vel is not None and r.r_vel is not None:
            x[3:6] = r.vel
            p[3:6, 3:6] = r.r_vel
        tr = RapTrack(self._next, x, p, r.t, r.t, updates=1)
        self._next += 1
        self.tracks.append(tr)
        return tr

    def _update(self, tr: RapTrack, r: Report) -> None:
        x, p = self._predict(tr, r.t)
        if r.vel is not None and r.r_vel is not None:
            h = np.eye(6)
            z = np.concatenate([r.pos, r.vel])
            rr = np.zeros((6, 6))
            rr[0:3, 0:3], rr[3:6, 3:6] = r.r_pos, r.r_vel
        else:
            h = np.hstack([np.eye(3), np.zeros((3, 3))])
            z, rr = r.pos, r.r_pos
        s = h @ p @ h.T + rr
        k = np.linalg.solve(s, h @ p).T
        tr.x = x + k @ (z - h @ x)
        tr.p = (np.eye(6) - k @ h) @ p
        tr.p = 0.5 * (tr.p + tr.p.T)
        tr.t = max(tr.t, r.t)
        tr.updates += 1

    def _book(self, tr: RapTrack, r: Report) -> None:
        if r.tracking:
            tr.sources[(r.node_id, r.object_id)] = r.t
            self._by_source[(r.node_id, r.object_id)] = tr
        tr.node_types.add(r.node_type)
        if r.remote_id is not None:
            tr.remote_id, tr.rid_pos, tr.rid_r, tr.rid_t = r.remote_id, r.pos, r.r_pos, r.t
        elif self._agrees_with_broadcast(tr, r):
            tr.noncoop_t = max(tr.noncoop_t, r.t)
        if r.node_type == "NODE_TYPE_CAMERA":
            tr.camera_t = max(tr.camera_t, r.t)
        if r.rf_protocol:
            tr.rf_protocol = r.rf_protocol
        if r.p_uav is not None and r.remote_id is None:
            _, n, total = tr.evidence.get(r.node_id, (r.node_type, 0, 0.0))
            tr.evidence[r.node_id] = (r.node_type, n + 1, total + _logit(r.p_uav))
        tr.llr = self._llr(tr)

    @staticmethod
    def _agrees_with_broadcast(tr: RapTrack, r: Report) -> bool:
        """Does an independent sensor put the aircraft where its Remote ID says it is?

        Only a report precise enough to tell (horizontal error under 30 m) and within the
        95% gate of the broadcast position counts. A coarse RF or acoustic fix that merely
        could be the same aircraft does not corroborate a serial.
        """
        if tr.rid_pos is None or tr.rid_r is None:
            return True  # no broadcast to check: any report is independent evidence
        if math.sqrt(max(r.r_pos[0, 0], r.r_pos[1, 1])) > 30.0:
            return False
        d = (r.pos - tr.rid_pos)[0:2]
        s = (r.r_pos + tr.rid_r)[0:2, 0:2]
        return float(d @ np.linalg.solve(s, d)) < 5.99  # chi-square, 2 dof, 95%

    @staticmethod
    def _llr(tr: RapTrack) -> float:
        if tr.remote_id is not None:  # a broadcasting aircraft is a drone, whatever else is said
            return LLR_MAX
        total = 0.0
        for kind, n, sum_logit in tr.evidence.values():
            total += CLASS_WEIGHT.get(kind, 0.5) * (sum_logit / n) * min(math.sqrt(n), MAX_LOOKS)
        return float(np.clip(total, -LLR_MAX, LLR_MAX))

    # ------------------------------------------------------------------ per tick
    def step(self, t: float) -> None:
        self.t = t
        self.tracks = [tr for tr in self.tracks if t - tr.t <= self.o.coast_s]
        self._merge(t)
        cloned = self._cloned_serials(t)
        for tr in self.tracks:
            tr.identity = self._identity(tr, t)
            if tr.remote_id in cloned and tr.declared is None:
                # One serial in two places: at most one is genuine, and Remote ID cannot say
                # which. No copy is shown as friend; the uncorroborated ones are suspect.
                tr.identity = "ASSUMED_FRIEND" if tr.identity == "FRIEND" else "SUSPECT"

    def _cloned_serials(self, t: float) -> set[str]:
        """Serials broadcast from two places at once: at most one of them is genuine."""
        where: dict[str, list[Vec]] = {}
        for tr in self.tracks:
            if tr.remote_id is not None and tr.rid_pos is not None and self.confirmed(tr):
                where.setdefault(tr.remote_id, []).append(tr.rid_pos)
        return {
            serial
            for serial, pts in where.items()
            if len(pts) > 1 and max(float(np.linalg.norm(a[0:2] - b[0:2])) for a in pts for b in pts) > 50.0
        }

    def _merge(self, t: float) -> None:
        live = sorted(self.tracks, key=lambda tr: tr.created_t)
        pred = {tr.ident: self._predict(tr, t) for tr in live}
        gone: set[int] = set()
        for i, a in enumerate(live):
            if a.ident in gone:
                continue
            xa, pa = pred[a.ident]
            for b in live[i + 1 :]:
                if b.ident in gone:
                    continue
                xb, pb = pred[b.ident]
                d = xa[0:3] - xb[0:3]
                # cheap reject: d'S^-1 d >= |d|^2 / trace(S), so this pair cannot pass the gate
                if float(d @ d) > MERGE_GATE * float(np.trace(pa[0:3, 0:3]) + np.trace(pb[0:3, 0:3])):
                    a.close.pop(b.ident, None)
                    continue
                close = float(d @ np.linalg.solve(pa[0:3, 0:3] + pb[0:3, 0:3], d)) < MERGE_GATE
                shared_node = any(
                    na == nb and oa != ob and t - la < 2.0 and t - lb < 2.0
                    for (na, oa), la in a.sources.items()
                    for (nb, ob), lb in b.sources.items()
                )
                if not close or shared_node:
                    a.close.pop(b.ident, None)
                    continue
                since = a.close.setdefault(b.ident, t)
                if t - since >= MERGE_AFTER_S:
                    self._absorb(a, b, t)
                    gone.add(b.ident)
        self.tracks = [tr for tr in self.tracks if tr.ident not in gone]

    def _absorb(self, a: RapTrack, b: RapTrack, t: float) -> None:
        """Merge duplicate ``b`` into the older track ``a`` (covariance-weighted state)."""
        xa, pa = self._predict(a, t)
        xb, pb = self._predict(b, t)
        ia, ib = np.linalg.inv(pa), np.linalg.inv(pb)
        p = np.linalg.inv(ia + ib)
        a.x, a.p, a.t = p @ (ia @ xa + ib @ xb), p, t
        a.updates += b.updates
        a.sources.update(b.sources)
        for key in b.sources:
            self._by_source[key] = a
        a.node_types |= b.node_types
        for node, (kind, n, total) in b.evidence.items():
            _, n0, t0 = a.evidence.get(node, (kind, 0, 0.0))
            a.evidence[node] = (kind, n0 + n, t0 + total)
        if a.remote_id is None or b.rid_t > a.rid_t:
            a.remote_id, a.rid_pos, a.rid_r, a.rid_t = b.remote_id, b.rid_pos, b.rid_r, b.rid_t
        a.llr = self._llr(a)
        a.rf_protocol = a.rf_protocol or b.rf_protocol
        a.noncoop_t = max(a.noncoop_t, b.noncoop_t)
        a.camera_t = max(a.camera_t, b.camera_t)
        a.merged_from.append(b.ident)
        self.merges += 1

    def class_of(self, tr: RapTrack) -> str:
        if tr.p_uav >= self.o.p_uav_drone:
            return "UAV"
        if tr.p_uav <= self.o.p_uav_bird:
            return "Bird"
        return "Unknown"

    def confirmed(self, tr: RapTrack) -> bool:
        return tr.updates >= self.o.confirm_updates or len(tr.node_types) >= 2

    def _identity(self, tr: RapTrack, t: float) -> str:
        if tr.declared is not None:
            return tr.declared
        if not self.confirmed(tr):
            return "PENDING"
        if tr.remote_id is not None and t - tr.rid_t > self.o.corroborate_s:
            # The broadcast has stopped while other sensors still hold the track: the serial
            # may belong to an aircraft this track no longer follows (two crossed paths), so
            # it no longer vouches for it.
            tr.remote_id = None
        if tr.remote_id is not None:
            if tr.remote_id in self.whitelist:
                return "FRIEND" if t - tr.noncoop_t <= self.o.corroborate_s else "ASSUMED_FRIEND"
            return "NEUTRAL"  # a compliant, broadcasting civil drone
        cls = self.class_of(tr)
        if cls == "Bird":
            return "NEUTRAL"
        if cls == "UAV" and self._threatens(tr, t):
            return "SUSPECT"
        return "UNKNOWN"

    def _threatens(self, tr: RapTrack, t: float) -> bool:
        x, _ = self._predict(tr, t)
        for site in self.protected.values():
            c, radius = site_disc(site)
            d = x[0:2] - c
            dist = float(np.linalg.norm(d))
            if dist <= radius:
                return True
            v = x[3:5]
            closing = -float(d @ v) / max(dist, 1e-6)
            if closing > 0.5 and (dist - radius) / closing <= self.o.suspect_horizon_s:
                return True
        return False

    def declare(self, ident: int, identity: str) -> None:
        """An operator's identification decision, e.g. ``HOSTILE``. Never made automatically."""
        if identity not in IDENTITIES:
            raise ValueError(identity)
        for tr in self.tracks:
            if tr.ident == ident:
                tr.declared = identity

    # ------------------------------------------------------------------ output
    def picture(self, t: float) -> list[RapTrack]:
        out = []
        for tr in self.tracks:
            if not self.confirmed(tr):
                continue
            x, p = self._predict(tr, t)
            shown = RapTrack(
                tr.ident, x, p, t, tr.created_t, tr.updates, dict(tr.sources), set(tr.node_types)
            )
            shown.llr, shown.remote_id, shown.identity = tr.llr, tr.remote_id, tr.identity
            shown.rf_protocol, shown.camera_t, shown.noncoop_t = tr.rf_protocol, tr.camera_t, tr.noncoop_t
            out.append(shown)
        return out

    def most_needed_look(self, t: float) -> RapTrack | None:
        """The confirmed track that most needs the airborne camera: unresolved and nearest harm.

        Candidates are tracks not identified as friend or neutral, whose class is not yet
        settled or that are suspect, and that no camera has seen recently. The most urgent
        is the one that would reach a protected area soonest; with no protected areas, the
        least certain.
        """
        best, best_key = None, (math.inf, math.inf)
        for tr in self.tracks:
            if not self.confirmed(tr) or tr.identity in ("FRIEND", "ASSUMED_FRIEND", "NEUTRAL"):
                continue
            if self.fence.signed_distance(self._predict(tr, t)[0]) <= 0.0:
                continue  # outside the area the airborne node may fly in
            if t - tr.camera_t < self.o.look_recent_s or t - tr.tasked_t < self.o.retask_s:
                continue
            settled = self.class_of(tr) == "UAV" and tr.p_uav > 0.97
            if settled and tr.identity != "SUSPECT":
                continue
            key = (self._time_to_protected(tr, t), -abs(0.5 - tr.p_uav))
            if key < best_key:
                best, best_key = tr, key
        return best

    def _time_to_protected(self, tr: RapTrack, t: float) -> float:
        if not self.protected:
            return 0.0
        x, _ = self._predict(tr, t)
        best = math.inf
        for site in self.protected.values():
            c, radius = site_disc(site)
            gap = max(0.0, float(np.linalg.norm(x[0:2] - c)) - radius)
            speed = max(float(np.linalg.norm(x[3:5])), 1.0)
            best = min(best, gap / speed)
        return best

    def look_at_task(self, t: float, tr: RapTrack) -> Message:
        """A SAPIENT task asking the airborne camera node to look at a picture track."""
        tr.tasked_t = t
        for live in self.tracks:
            if live.ident == tr.ident:
                live.tasked_t = t
        x, p = self._predict(tr, t)
        sigma = np.sqrt(np.clip(np.diag(p)[0:3], 0.0, None))
        return {
            "timestamp": self.geo.timestamp(t),
            "node_id": self.node_id,
            "destination_id": self.camera_node or "",
            "task": {
                "task_id": self.geo.new_ulid(t),
                "task_name": f"confirm RAP track {tr.ident}",
                "control": "CONTROL_START",
                "command": {"look_at": {"location_list": {"locations": [self.geo.location(x[0:3], sigma)]}}},
            },
        }
