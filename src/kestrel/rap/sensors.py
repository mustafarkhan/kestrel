"""Simulated ground sensors, each a SAPIENT node with the blind spots fielded sensors have.

Every sensor writes ICD v7 messages (registration, status, detection reports) in the same
proto3 JSON layout as Kestrel's own exporter, so the fusion node reads simulated and
recorded traffic the same way. The models are deliberately simple; what matters for the
picture is that each one misses what its real counterpart misses:

* **Radar** sees drones and birds alike (similar radar cross-section), loses low targets
  in clutter at range, breaks tracks after missed scans, raises clutter tracks, and its
  micro-Doppler classifier is a weak, noisy opinion.
* **RF direction finders** hear only drones with a radio link: fibre-optic and autonomous
  GNSS/INS drones are invisible to them. What they hear, they classify with confidence.
* **Remote ID** is cooperative: it reports what a drone *says* about itself, including a
  position a spoofer can fake.
* **Acoustic** nodes hear engines far and quiet electric drones near, give no altitude,
  and report fixes rather than tracks.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from kestrel.config import (
    GroundAcousticConfig,
    RadarConfig,
    RemoteIdConfig,
    RfConfig,
    SensorNetConfig,
    WorldConfig,
)
from kestrel.geometry import Vec
from kestrel.interop.sapient import SapientExporter
from kestrel.perception.acoustic import triangulate
from kestrel.rap.truth import TruthObject

Message = dict[str, Any]


def _duration(seconds: int) -> Message:
    return {"units": "TIME_UNITS_SECONDS", "value": seconds}


@dataclass
class GroundNode:
    """One SAPIENT node: registers, reports status, and writes detection reports."""

    geo: SapientExporter
    node_type: str
    name: str
    short_name: str
    out: list[Message]
    tracking: bool = True
    node_id: str = ""
    last_status_t: float = -math.inf

    def __post_init__(self) -> None:
        self.node_id = self.geo.new_node_id()

    def _emit(self, t: float, key: str, content: Message) -> None:
        self.out.append({"timestamp": self.geo.timestamp(t), "node_id": self.node_id, key: content})

    def register(self, t: float) -> None:
        loc_type = {
            "location_units": "LOCATION_COORDINATE_SYSTEM_UTM_M",
            "location_datum": "LOCATION_DATUM_WGS84_G",
            "zone": self.geo.zone,
        }
        mode = {
            "mode_name": "surveil",
            "settle_time": _duration(0),
            "scan_type": "SCAN_TYPE_SCANNING" if self.node_type == "NODE_TYPE_RADAR" else "SCAN_TYPE_FIXED",
            "tracking_type": "TRACKING_TYPE_TRACK" if self.tracking else "TRACKING_TYPE_NONE",
            "detection_definition": {"location_type": loc_type},
            "mode_type": "MODE_TYPE_PERMANENT",
        }
        self._emit(
            t,
            "registration",
            {
                "node_type": [self.node_type],
                "icd_version": "7.0",
                "name": self.name,
                "short_name": self.short_name,
                "status_definition": {"status_interval": _duration(5), "location_definition": loc_type},
                "mode_definition": [mode],
            },
        )

    def status(self, t: float, where: Vec | None) -> None:
        content: Message = {"report_id": self.geo.new_ulid(t), "system": "SYSTEM_OK", "mode": "surveil"}
        if where is not None:
            content["sensor_location"] = self.geo.location(where)
        self._emit(t, "status_report", content)
        self.last_status_t = t

    def detect(
        self,
        t: float,
        object_id: str,
        pos: Vec,
        sigma: Vec,
        vel: Vec | None = None,
        vel_sigma: Vec | None = None,
        p_uav: float | None = None,
        info: list[tuple[str, str]] | None = None,
    ) -> None:
        content: Message = {
            "report_id": self.geo.new_ulid(t),
            "object_id": object_id,
            "location": self.geo.location(pos, sigma),
        }
        if vel is not None and vel_sigma is not None:
            content["enu_velocity"] = {
                "east_rate": round(float(vel[0]), 2),
                "north_rate": round(float(vel[1]), 2),
                "up_rate": round(float(vel[2]), 2),
                "east_rate_error": round(float(vel_sigma[0]), 2),
                "north_rate_error": round(float(vel_sigma[1]), 2),
                "up_rate_error": round(float(vel_sigma[2]), 2),
            }
        if p_uav is not None:
            content["classification"] = [
                {"type": "UAV", "confidence": round(float(p_uav), 3)},
                {"type": "Bird", "confidence": round(1.0 - float(p_uav), 3)},
            ]
        if info:
            content["track_info"] = [{"type": k, "value": v} for k, v in info]
        self._emit(t, "detection_report", content)


# --------------------------------------------------------------------------- radar


@dataclass
class _RadarTrack:
    hits: list[bool] = field(default_factory=list)
    object_id: str | None = None
    misses: int = 0


@dataclass
class _Clutter:
    object_id: str
    pos: Vec
    vel: Vec
    until: float


class Radar:
    def __init__(self, cfg: RadarConfig, node: GroundNode, home: Vec, rng: np.random.Generator) -> None:
        self.cfg = cfg
        self.node = node
        self.xyz = home + np.array(cfg.xyz, dtype=float)
        self.rng = rng
        self.tracks: dict[str, _RadarTrack] = {}
        self.clutter: list[_Clutter] = []
        self.next_scan = 0.0
        self.ref_snr = 10.0 ** (cfg.ref_snr_db / 10.0)

    def p_detect(self, obj: TruthObject) -> float:
        d = obj.pos - self.xyz
        rng = float(np.linalg.norm(d))
        if rng < 1.0:
            return 1.0
        if math.degrees(math.asin(float(d[2]) / rng)) < self.cfg.min_elevation_deg:
            return 0.0
        low = obj.pos[2] - (self.xyz[2] - self.cfg.xyz[2]) < self.cfg.masked_below_m
        if low and float(np.linalg.norm(d[0:2])) > self.cfg.masked_beyond_m:
            return 0.0
        snr = self.ref_snr * (obj.rcs_m2 / self.cfg.ref_rcs_m2) * (self.cfg.ref_range_m / rng) ** 4
        return float(self.cfg.pfa ** (1.0 / (1.0 + snr)))

    def _measure(self, pos: Vec) -> tuple[Vec, Vec]:
        """A noisy plot in spherical coordinates, returned in ENU with per-axis sigmas."""
        c = self.cfg
        d = pos - self.xyz
        r = float(np.linalg.norm(d))
        az = math.atan2(float(d[1]), float(d[0]))
        el = math.asin(float(d[2]) / max(r, 1e-6))
        sa, se = math.radians(c.azimuth_sigma_deg), math.radians(c.elevation_sigma_deg)
        rm = r + self.rng.normal(0.0, c.range_sigma_m)
        am, em = az + self.rng.normal(0.0, sa), el + self.rng.normal(0.0, se)
        p = self.xyz + rm * np.array([math.cos(em) * math.cos(am), math.cos(em) * math.sin(am), math.sin(em)])
        # covariance by linearising the spherical-to-Cartesian map
        j = np.array(
            [
                [
                    math.cos(el) * math.cos(az),
                    -r * math.cos(el) * math.sin(az),
                    -r * math.sin(el) * math.cos(az),
                ],
                [
                    math.cos(el) * math.sin(az),
                    r * math.cos(el) * math.cos(az),
                    -r * math.sin(el) * math.sin(az),
                ],
                [math.sin(el), 0.0, r * math.cos(el)],
            ]
        )
        cov = j @ np.diag([c.range_sigma_m**2, sa**2, se**2]) @ j.T
        return p, np.sqrt(np.diag(cov))

    def _score(self, cue: float) -> float:
        return float(np.clip(cue + self.rng.normal(0.0, self.cfg.class_noise), 0.02, 0.98))

    def scan(self, t: float, objects: list[TruthObject]) -> None:
        if t < self.next_scan:
            return
        self.next_scan += self.cfg.scan_s
        c = self.cfg
        for obj in objects:
            tr = self.tracks.setdefault(obj.key, _RadarTrack())
            hit = bool(self.rng.random() < self.p_detect(obj))
            tr.hits = [*tr.hits, hit][-c.confirm_n :]
            if tr.object_id is None:
                if sum(tr.hits) >= c.confirm_m:
                    tr.object_id, tr.misses = self.node.geo.new_ulid(t), 0
                else:
                    continue
            if not hit:
                tr.misses += 1
                if tr.misses >= c.drop_after_misses:  # track lost: the next one gets a new id
                    self.tracks[obj.key] = _RadarTrack()
                continue
            tr.misses = 0
            pos, sigma = self._measure(obj.pos)
            self.node.detect(t, tr.object_id, pos, sigma, p_uav=self._score(obj.radar_class_cue))
        self._clutter(t)

    def _clutter(self, t: float) -> None:
        c = self.cfg
        if self.rng.random() < c.false_tracks_per_hour * c.scan_s / 3600.0:
            r = self.rng.uniform(100.0, c.ref_range_m)
            a = self.rng.uniform(-math.pi, math.pi)
            pos = self.xyz + np.array([r * math.cos(a), r * math.sin(a), self.rng.uniform(5.0, 60.0)])
            hd = self.rng.uniform(-math.pi, math.pi)
            vel = self.rng.uniform(0.0, 6.0) * np.array([math.cos(hd), math.sin(hd), 0.0])
            until = t + self.rng.uniform(3.0, 12.0)
            self.clutter.append(_Clutter(self.node.geo.new_ulid(t), pos, vel, until))
        live = []
        for cl in self.clutter:
            if t > cl.until:
                continue
            cl.pos = cl.pos + cl.vel * c.scan_s
            pos, sigma = self._measure(cl.pos)
            self.node.detect(t, cl.object_id, pos, sigma, p_uav=self._score(0.5))
            live.append(cl)
        self.clutter = live


# --------------------------------------------------------------------------- RF


class RfNetwork:
    def __init__(self, cfg: RfConfig, node: GroundNode, home: Vec, rng: np.random.Generator) -> None:
        self.cfg = cfg
        self.node = node
        self.nodes = np.array(cfg.nodes, dtype=float).reshape(-1, 2) + home[0:2]
        self.rng = rng
        self.sigma = math.radians(cfg.bearing_sigma_deg)
        self.next_t = 0.0
        self.ids: dict[str, str] = {}

    def sense(self, t: float, objects: list[TruthObject], band: tuple[float, float]) -> None:
        if t < self.next_t or len(self.nodes) < 2:
            return
        self.next_t += 1.0 / self.cfg.rate_hz
        for obj in objects:
            if not obj.rf_link:
                continue
            d = obj.pos[0:2] - self.nodes
            rng = np.linalg.norm(d, axis=1)
            heard = (rng < self.cfg.range_m) & (
                self.rng.random(len(self.nodes)) < self.cfg.detection_probability
            )
            if heard.sum() < 2:
                continue
            bearings = np.arctan2(d[heard, 1], d[heard, 0]) + self.rng.normal(
                0.0, self.sigma, int(heard.sum())
            )
            fix = triangulate(self.nodes[heard], bearings, rng[heard], self.sigma)
            if fix is None:
                continue
            xy, cov = fix
            lo, hi = band
            sig = np.sqrt(np.clip(np.diag(cov), 0.0, None))
            pos = np.array([xy[0], xy[1], 0.5 * (lo + hi)])
            oid = self.ids.setdefault(obj.key, self.node.geo.new_ulid(t))  # emitter fingerprint
            info = [("rf_protocol", obj.rf_protocol)]
            self.node.detect(t, oid, pos, np.array([sig[0], sig[1], 0.5 * (hi - lo)]), p_uav=0.97, info=info)


# --------------------------------------------------------------------------- Remote ID


class RemoteIdReceiver:
    def __init__(self, cfg: RemoteIdConfig, node: GroundNode, home: Vec, rng: np.random.Generator) -> None:
        self.cfg = cfg
        self.node = node
        self.xy = np.array(cfg.xy, dtype=float) + home[0:2]
        self.rng = rng
        self.next_t = 0.0
        self.ids: dict[str, str] = {}

    def sense(self, t: float, objects: list[TruthObject]) -> None:
        if t < self.next_t:
            return
        self.next_t += 1.0
        sh, sv = self.cfg.gnss_sigma_m
        sigma = np.array([sh, sh, sv])
        for obj in objects:
            if obj.remote_id is None or np.linalg.norm(obj.pos[0:2] - self.xy) > self.cfg.range_m:
                continue
            claimed = obj.pos + (obj.remote_id_offset if obj.remote_id_offset is not None else 0.0)
            pos = claimed + self.rng.normal(0.0, sigma)
            vel = obj.vel + self.rng.normal(0.0, 0.3, 3)
            oid = self.ids.setdefault(obj.key, self.node.geo.new_ulid(t))  # per broadcaster
            info = [("remote_id", obj.remote_id)]
            self.node.detect(t, oid, pos, sigma, vel, np.full(3, 0.5), p_uav=0.99, info=info)


# --------------------------------------------------------------------------- acoustic


class GroundAcoustic:
    def __init__(
        self, cfg: GroundAcousticConfig, node: GroundNode, world: WorldConfig, rng: np.random.Generator
    ) -> None:
        self.cfg = cfg
        self.node = node
        f = np.asarray(world.fence, dtype=float)
        lo, hi = f.min(axis=0), f.max(axis=0)
        xs = np.arange(lo[0] + cfg.spacing_m / 2, hi[0], cfg.spacing_m)
        ys = np.arange(lo[1] + cfg.spacing_m / 2, hi[1], cfg.spacing_m)
        self.nodes = np.array([(x, y) for x in xs for y in ys], dtype=float).reshape(-1, 2)
        self.rng = rng
        self.sigma = math.radians(cfg.bearing_sigma_deg)
        self.next_t = 0.0

    def range_for(self, obj: TruthObject) -> float:
        c = self.cfg
        return c.range_m * {"combustion": c.combustion_factor, "quiet": c.quiet_factor}.get(
            obj.propulsion, 1.0
        )

    def sense(self, t: float, objects: list[TruthObject], band: tuple[float, float]) -> None:
        if t < self.next_t or len(self.nodes) < 2:
            return
        self.next_t += 1.0
        for obj in objects:
            if obj.kind != "uav":  # the node's classifier is trained on rotor and engine sound
                continue
            d = obj.pos[0:2] - self.nodes
            rng = np.linalg.norm(d, axis=1)
            heard = (rng < self.range_for(obj)) & (
                self.rng.random(len(self.nodes)) < self.cfg.detection_probability
            )
            if heard.sum() < 2:
                continue
            bearings = np.arctan2(d[heard, 1], d[heard, 0]) + self.rng.normal(
                0.0, self.sigma, int(heard.sum())
            )
            fix = triangulate(self.nodes[heard], bearings, rng[heard], self.sigma)
            if fix is None:
                continue
            xy, cov = fix
            lo, hi = band
            sig = np.sqrt(np.clip(np.diag(cov), 0.0, None))
            pos = np.array([xy[0], xy[1], 0.5 * (lo + hi)])
            # a fix, not a track: every report is a new object, as fielded networks report
            self.node.detect(
                t, self.node.geo.new_ulid(t), pos, np.array([sig[0], sig[1], 0.5 * (hi - lo)]), p_uav=0.8
            )


# --------------------------------------------------------------------------- network


class SensorNetwork:
    """All ground sensors of a scenario; ``step`` returns the SAPIENT messages they sent."""

    def __init__(
        self,
        cfg: SensorNetConfig,
        world: WorldConfig,
        geo: SapientExporter,
        seed: int,
        include: set[str] | None = None,
        altitude_band_m: tuple[float, float] = (0.0, 120.0),
    ) -> None:
        self.out: list[Message] = []
        self.band = altitude_band_m
        home = np.array(world.home, dtype=float)
        use = include if include is not None else {"radar", "rf", "remote_id", "acoustic"}
        rngs = [np.random.default_rng(seed * 101 + k) for k in range(4)]
        self.nodes: list[GroundNode] = []

        def node(kind: str, name: str, short: str, tracking: bool = True) -> GroundNode:
            n = GroundNode(geo, kind, name, short, self.out, tracking)
            self.nodes.append(n)
            return n

        self.radar = (
            Radar(cfg.radar, node("NODE_TYPE_RADAR", "Ground surveillance radar", "RADAR"), home, rngs[0])
            if cfg.radar is not None and "radar" in use
            else None
        )
        self.rf = (
            RfNetwork(
                cfg.rf, node("NODE_TYPE_PASSIVE_RF", "RF direction-finding network", "RF-DF"), home, rngs[1]
            )
            if cfg.rf is not None and cfg.rf.nodes and "rf" in use
            else None
        )
        self.rid = (
            RemoteIdReceiver(
                cfg.remote_id, node("NODE_TYPE_PASSIVE_RF", "Remote ID receiver", "REMOTE-ID"), home, rngs[2]
            )
            if cfg.remote_id is not None and "remote_id" in use
            else None
        )
        self.acoustic = (
            GroundAcoustic(
                cfg.acoustic,
                node("NODE_TYPE_ACOUSTIC", "Ground acoustic network", "ACOUSTIC", tracking=False),
                world,
                rngs[3],
            )
            if cfg.acoustic is not None and "acoustic" in use
            else None
        )
        self.registered = False

    def step(self, t: float, objects: list[TruthObject]) -> list[Message]:
        start = len(self.out)
        if not self.registered:
            for n in self.nodes:
                n.register(t)
            self.registered = True
        for n in self.nodes:
            if t - n.last_status_t >= 5.0:
                n.status(t, None)
        if self.radar is not None:
            self.radar.scan(t, objects)
        if self.rf is not None:
            self.rf.sense(t, objects, self.band)
        if self.rid is not None:
            self.rid.sense(t, objects)
        if self.acoustic is not None:
            self.acoustic.sense(t, objects, self.band)
        return self.out[start:]
