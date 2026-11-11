"""SAPIENT reports: Kestrel's tracks in the message set NATO uses for counter-UAS sensors.

SAPIENT is the UK Dstl sensor-integration standard (BSI Flex 335), trialled at NATO's
counter-UAS Technical Interoperability Exercise and going through NATO standardisation.
A SAPIENT *node* (a sensor) registers with a fusion node, sends a status report at least
once a minute, and sends a detection report per object it sees. This module builds those
messages from Kestrel's state so its output can feed a SAPIENT fusion node next to other
sensors.

Field names, enumerations and units follow Dstl's published protobuf schema for BSI
Flex 335 v2.0 (https://github.com/dstl/SAPIENT-Proto-Files, vendored under
``third_party/sapient-protos``), and the message set of the Interface Control Document v7.0
(Dstl, 2023) it descends from: the ``SapientMessage`` wrapper (``timestamp``, ``node_id``,
optional ``destination_id``, one content field), ``Registration``, ``StatusReport``,
``DetectionReport`` and ``TaskAck``. Messages are produced as dictionaries in the proto3 JSON
layout (oneof member as the key, enum values by name, ``Timestamp`` as RFC 3339), and the
test suite parses every kind Kestrel writes against both the v1.0 and v2.0 schemas
(``tests/test_sapient_conformance.py``). The wire format is protobuf binary with a 4-byte
little-endian length prefix over TCP; with the schema compiled, the JSON converts to it
with ``google.protobuf.json_format``. No run against a SAPIENT test harness or fusion node
has been made.

Mapping choices, each stated rather than hidden:

* **Coordinates**: ``Location`` in ``UTM_M`` (easting, northing, altitude in metres; the
  zone is declared once at registration, as the ICD specifies) on the ``WGS84_G`` datum
  (altitude above the geoid, as ArduPilot home altitude is). Errors are one-sigma, metres.
* **Velocity**: ``ENUVelocity`` from the tracker, one-sigma errors.
* **Two nodes**: the observer's cameras (``NODE_TYPE_CAMERA``) and, when fitted, the ground
  acoustic network (``NODE_TYPE_ACOUSTIC``). An acoustic fix has no altitude, so it is sent
  at the middle of the altitude band with ``z_error`` covering the band, never on the
  ground plane (which is what the ICD assumes when ``z`` is omitted).
* **Classification**: class names are configurable. The ICD's taxonomy appendix was not
  available when this was written, so ``"UAV"`` and ``"Bird"`` are placeholders to be
  aligned with it.
"""

from __future__ import annotations

import json
import math
import uuid
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import UTC, datetime
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from typing import Any

import numpy as np

from kestrel.geometry import Vec
from kestrel.interop.utm import enu_to_lla, from_utm, lla_to_enu, to_utm, zone_of
from kestrel.perception.acoustic import AcousticFix
from kestrel.perception.tracker import TrackEstimate

ICD_VERSION = "BSI Flex 335 v2.0"
Message = dict[str, Any]
_CROCKFORD = "0123456789ABCDEFGHJKMNPQRSTVWXYZ"


def ulid(ms: int, rng: np.random.Generator) -> str:
    """A ULID: 48-bit millisecond time and 80 random bits, Crockford base32, 26 characters."""
    value = (ms & ((1 << 48) - 1)) << 80 | int.from_bytes(rng.bytes(10), "big")
    return "".join(_CROCKFORD[(value >> (5 * (25 - i))) & 31] for i in range(26))


def rfc3339(epoch_s: float) -> str:
    dt = datetime.fromtimestamp(epoch_s, tz=UTC)
    return dt.strftime("%Y-%m-%dT%H:%M:%S.") + f"{dt.microsecond // 1000:03d}Z"


def _duration(seconds: int) -> Message:
    return {"units": "TIME_UNITS_SECONDS", "value": seconds}


def software_version() -> str:
    try:
        return version("kestrel-uas")
    except PackageNotFoundError:  # running from a source tree that is not installed
        return "0+source"


def config_data(model: str, manufacturer: str = "Kestrel (open source)") -> list[Message]:
    """The mandatory ``config_data`` of a registration: who made the node and what runs on it."""
    return [{"manufacturer": manufacturer, "model": model, "software_version": software_version()}]


def capability(category: str, kind: str, value: str = "", units: str = "") -> Message:
    out: Message = {"category": category, "type": kind}
    if value:
        out["value"] = value
    if units:
        out["units"] = units
    return out


def region_task(location_type: Message, commands: list[Message] | None = None) -> Message:
    """The mandatory ``task`` of a mode definition: the regions (and commands) a node accepts."""
    out: Message = {
        "region_definition": {
            "region_type": ["REGION_TYPE_AREA_OF_INTEREST"],
            "region_area": [location_type],
        }
    }
    if commands:
        out["command"] = commands
    return out


@dataclass
class SapientOptions:
    origin_lla: tuple[float, float, float]  # geodetic position of the local ENU origin
    epoch_s: float = 1767225600.0  # wall-clock time of sim t = 0 (2026-01-01T00:00:00Z)
    status_interval_s: int = 1
    detection_rate_hz: float = 2.0  # per object; the ICD leaves rate to the deployment
    altitude_band_m: tuple[float, float] = (0.0, 60.0)  # AGL band an acoustic fix may lie in
    camera_band: str = "visible"  # declared in the camera node's registration
    uav_class: str = "UAV"
    bird_class: str = "Bird"
    seed: int = 0


@dataclass
class _Node:
    node_id: str
    node_type: str
    name: str
    short_name: str
    mode: str = "observe"
    registered: bool = False
    last_status_t: float = -math.inf
    last_mode: str = ""


@dataclass
class SapientExporter:
    """Turns Kestrel state into SAPIENT messages and collects (or streams) them."""

    opts: SapientOptions
    sink: Callable[[Message], None] | None = None
    messages: list[Message] = field(default_factory=list)
    keep: bool = True  # collect messages in ``messages``; a long flight streams them instead
    system: str = "SYSTEM_OK"  # reported in the camera node's status (WARNING / ERROR on a fault)

    def __post_init__(self) -> None:
        self.link_ok = True
        self.backlog: list[Message] = []
        self.late_deliveries = 0
        self._rng = np.random.default_rng(self.opts.seed)
        lat, lon, _ = self.opts.origin_lla
        self.zone = zone_of(lat, lon)
        self._zone_number = int(self.zone[:-1])
        self.camera = _Node(self._uuid(), "NODE_TYPE_CAMERA", "Kestrel airborne EO observer", "KESTREL-EO")
        self.acoustic = _Node(self._uuid(), "NODE_TYPE_ACOUSTIC", "Ground acoustic network", "KESTREL-AC")
        self._object_ids: dict[int, str] = {}
        self._last_detection: dict[int, float] = {}

    # ------------------------------------------------------------------ helpers
    def _uuid(self) -> str:
        return str(uuid.UUID(bytes=self._rng.bytes(16), version=4))

    def _ulid(self, t: float) -> str:
        return ulid(int((self.opts.epoch_s + t) * 1000), self._rng)

    def new_node_id(self) -> str:
        """A fresh UUID v4 for another node sharing this exporter's frame and random stream."""
        return self._uuid()

    def new_ulid(self, t: float) -> str:
        return self._ulid(t)

    def timestamp(self, t: float) -> str:
        return rfc3339(self.opts.epoch_s + t)

    def sim_time(self, timestamp: str) -> float:
        """Seconds since this exporter's epoch of an RFC 3339 timestamp (any fraction digits)."""
        dt = datetime.fromisoformat(timestamp.replace("Z", "+00:00"))
        return dt.timestamp() - self.opts.epoch_s

    @property
    def camera_node_id(self) -> str:
        return self.camera.node_id

    def _emit(self, t: float, node: _Node, content_key: str, content: Message) -> None:
        msg = {"timestamp": rfc3339(self.opts.epoch_s + t), "node_id": node.node_id, content_key: content}
        if not self.link_ok:
            # Store and forward: while the link is down, reports keep their measurement time and
            # wait. Only the latest status report per node is worth sending late.
            if content_key == "status_report":
                self.backlog = [
                    m for m in self.backlog if not ("status_report" in m and m["node_id"] == node.node_id)
                ]
            self.backlog.append(msg)
            return
        self._deliver(msg)

    def _deliver(self, msg: Message) -> None:
        if self.keep:
            self.messages.append(msg)
        if self.sink is not None:
            self.sink(msg)

    def set_link(self, t: float, ok: bool) -> None:
        if ok and not self.link_ok:
            for m in self.backlog:
                self._deliver(m)
            self.late_deliveries += len(self.backlog)
            self.backlog = []
        self.link_ok = ok

    def location(self, enu: Vec, sigma: Vec | None = None) -> Message:
        """A SAPIENT ``Location`` (UTM_M, WGS84_G) for a point in the local ENU frame."""
        lat, lon, alt = enu_to_lla(self.opts.origin_lla, float(enu[0]), float(enu[1]), float(enu[2]))
        easting, northing, _, _ = to_utm(lat, lon, self._zone_number)
        loc: Message = {
            "x": round(easting, 2),
            "y": round(northing, 2),
            "z": round(alt, 2),
            "coordinate_system": "LOCATION_COORDINATE_SYSTEM_UTM_M",
            "datum": "LOCATION_DATUM_WGS84_G",
        }
        if sigma is not None:
            loc |= {"x_error": round(float(sigma[0]), 2), "y_error": round(float(sigma[1]), 2)}
            loc |= {"z_error": round(float(sigma[2]), 2)}
        return loc

    def _location_type(self) -> Message:
        return {
            "location_units": "LOCATION_COORDINATE_SYSTEM_UTM_M",
            "location_datum": "LOCATION_DATUM_WGS84_G",
            "zone": self.zone,
        }

    def _look_at_definition(self) -> Message:
        """The camera node accepts LOOK_AT tasks over an area of interest (ICD 5.6, 5.6.8-9)."""
        return region_task(
            self._location_type(),
            [{"units": "locationList", "completion_time": _duration(60), "type": "COMMAND_TYPE_LOOK_AT"}],
        )

    def to_enu(self, loc: Message) -> Vec:
        """A SAPIENT ``Location`` (UTM_M in our zone, or LAT_LNG_DEG_M) in the local ENU frame."""
        system = str(loc.get("coordinate_system", ""))
        if system.endswith("UTM_M"):
            north = self.opts.origin_lla[0] >= 0.0
            lat, lon = from_utm(float(loc["x"]), float(loc["y"]), self._zone_number, north)
        elif system.endswith("LAT_LNG_DEG_M"):
            lon, lat = float(loc["x"]), float(loc["y"])  # x as longitude (easting-like), see interop.md
        else:
            raise ValueError(f"unsupported coordinate system {system!r}")
        z = float(loc.get("z", self.opts.origin_lla[2]))
        return np.array(lla_to_enu(self.opts.origin_lla, lat, lon, z))

    def look_at(self, msg: Message) -> tuple[str, Vec] | None:
        """If ``msg`` is a task asking this node to look at an area, return its id and centre.

        The field path is Dstl's published protobuf (BSI Flex 335 v1.0 and v2.0):
        ``task.command.look_at`` is a ``LocationOrRangeBearing`` whose ``location_list`` holds
        the locations. A bare ``look_at.locations`` list, as earlier Kestrel versions wrote,
        is also read.
        """
        detail = self.look_at_detail(msg)
        return None if detail is None else (detail[0], detail[1])

    def look_at_detail(self, msg: Message) -> tuple[str, Vec, bool] | None:
        """As :meth:`look_at`, plus whether the locations give a usable altitude.

        A location whose ``z_error`` is under 15 m comes from a sensor that measured height
        (a radar or a fused track); without one, the place is a vertical column, as an
        acoustic or RF fix is.
        """
        task = msg.get("task")
        if not isinstance(task, dict):
            return None
        look = (task.get("command") or {}).get("look_at") or {}
        locs = (look.get("location_list") or {}).get("locations") or look.get("locations") or []
        if not locs:
            return None
        pts = np.array([self.to_enu(loc) for loc in locs])
        z_known = all(float(loc.get("z_error", math.inf)) < 15.0 for loc in locs)
        return str(task.get("task_id", "")), pts.mean(axis=0), z_known

    def traffic_estimate(self, msg: Message, ident: int) -> TrackEstimate | None:
        """A fusion node's detection report as a contact for the separation filter.

        Returns ``None`` unless the report places the aircraft in three dimensions. The
        contact's time is when the track was last *measured* (``measured_at``), not when the
        report was sent, so the filter's reachable-set term covers the age honestly.
        """
        det = msg.get("detection_report")
        loc = det.get("location") if isinstance(det, dict) else None
        if not isinstance(det, dict) or not isinstance(loc, dict):
            return None
        sig = [float(loc.get(k, math.inf)) for k in ("x_error", "y_error", "z_error")]
        if not all(math.isfinite(v) for v in sig):
            return None
        info = {str(i.get("type")): str(i.get("value")) for i in det.get("track_info") or []}
        sent = self.sim_time(str(msg["timestamp"]))
        measured = min(self.sim_time(info.get("measured_at", str(msg["timestamp"]))), sent)
        v = det.get("enu_velocity") or {}
        vel = np.array([float(v.get(k, 0.0)) for k in ("east_rate", "north_rate", "up_rate")])
        vsig = [float(v.get(k, 5.0)) for k in ("east_rate_error", "north_rate_error", "up_rate_error")]
        cov = np.diag(np.array(sig + vsig) ** 2)
        return TrackEstimate(ident, self.to_enu(loc), vel, cov, True, 0, measured)

    def task_ack(self, t: float, task_id: str, accepted: bool, reason: str = "") -> None:
        content: Message = {
            "task_id": task_id,
            "task_status": "TASK_STATUS_ACCEPTED" if accepted else "TASK_STATUS_REJECTED",
        }
        if reason:
            content["reason"] = [reason]
        self._emit(t, self.camera, "task_ack", content)

    # ------------------------------------------------------------------ messages
    def registration(self, t: float, node: _Node) -> None:
        mode = {
            "mode_name": node.mode,
            "mode_description": "autonomous observation",
            "settle_time": _duration(0),
            "scan_type": "SCAN_TYPE_STEERABLE" if node is self.camera else "SCAN_TYPE_FIXED",
            "tracking_type": "TRACKING_TYPE_TRACK" if node is self.camera else "TRACKING_TYPE_NONE",
            "detection_definition": [{"location_type": self._location_type()}],
            "mode_type": "MODE_TYPE_PERMANENT",
        }
        # a mode definition must declare its task; a node that takes no commands declares none
        mode["task"] = (
            self._look_at_definition() if node is self.camera else region_task(self._location_type())
        )
        if node is self.camera:
            caps = [
                capability("Camera", "Band", self.opts.camera_band),
                capability("Camera", "Detection report rate", f"{self.opts.detection_rate_hz:g}", "Hz"),
                capability("Platform", "Mobile airborne node (multirotor)"),
            ]
        else:
            caps = [capability("Acoustic", "Bearing-only nodes, triangulated fixes without altitude")]
        content = {
            "node_definition": [{"node_type": node.node_type}]
            + ([{"node_type": "NODE_TYPE_MOBILE_NODE"}] if node is self.camera else []),
            "icd_version": ICD_VERSION,
            "name": node.name,
            "short_name": node.short_name,
            "capabilities": caps,
            "config_data": config_data(node.short_name),
            "status_definition": {
                "status_interval": _duration(self.opts.status_interval_s),
                "location_definition": self._location_type(),
            },
            "mode_definition": [mode],
        }
        self._emit(t, node, "registration", content)
        node.registered = True

    def status(
        self, t: float, node: _Node, sensor_enu: Vec | None, mode: str, system: str = "SYSTEM_OK"
    ) -> None:
        if not node.registered:
            self.registration(t, node)
        changed = mode != node.last_mode
        content: Message = {
            "report_id": self._ulid(t),
            "system": system,
            "info": "INFO_NEW" if changed else "INFO_UNCHANGED",
            "mode": mode,
        }
        if sensor_enu is not None:
            content["node_location"] = self.location(sensor_enu)
        self._emit(t, node, "status_report", content)
        node.last_status_t = t
        node.last_mode = mode

    def detection(self, t: float, track: TrackEstimate, p_uav: float | None = None) -> None:
        if track.ident not in self._object_ids:
            self._object_ids[track.ident] = self._ulid(t)
        sigma_p = np.sqrt(np.clip(np.diag(track.cov)[0:3], 0.0, None))
        sigma_v = np.sqrt(np.clip(np.diag(track.cov)[3:6], 0.0, None))
        content: Message = {
            "report_id": self._ulid(t),
            "object_id": self._object_ids[track.ident],
            "location": self.location(track.pos, sigma_p),
            "detection_confidence": round(float(track.hit_ratio), 3),
            "enu_velocity": {
                "east_rate": round(float(track.vel[0]), 2),
                "north_rate": round(float(track.vel[1]), 2),
                "up_rate": round(float(track.vel[2]), 2),
                "east_rate_error": round(float(sigma_v[0]), 2),
                "north_rate_error": round(float(sigma_v[1]), 2),
                "up_rate_error": round(float(sigma_v[2]), 2),
            },
            "track_info": [{"type": "kestrel_track_id", "value": str(track.ident)}],
        }
        if p_uav is not None:
            content["classification"] = [
                {"type": self.opts.uav_class, "confidence": round(p_uav, 3)},
                {"type": self.opts.bird_class, "confidence": round(1.0 - p_uav, 3)},
            ]
        self._emit(t, self.camera, "detection_report", content)

    def acoustic_detection(self, fix: AcousticFix, home_alt: float) -> None:
        lo, hi = self.opts.altitude_band_m
        sigma = np.sqrt(np.clip(np.diag(fix.cov_xy), 0.0, None))
        enu = np.array([fix.xy[0], fix.xy[1], home_alt + 0.5 * (lo + hi)])
        content: Message = {
            "report_id": self._ulid(fix.t),
            "object_id": self._ulid(fix.t),  # the network reports fixes, not tracks
            "location": self.location(enu, np.array([sigma[0], sigma[1], 0.5 * (hi - lo)])),
            "track_info": [{"type": "nodes_contributing", "value": str(fix.nodes)}],
        }
        self._emit(fix.t, self.acoustic, "detection_report", content)

    # ------------------------------------------------------------------ per tick
    def step(
        self,
        t: float,
        observer_enu: Vec,
        mode: str,
        tracks: list[TrackEstimate],
        new_fixes: list[AcousticFix],
        acoustic_fitted: bool,
        home_alt: float = 0.0,
    ) -> None:
        if t - self.camera.last_status_t >= self.opts.status_interval_s or mode != self.camera.last_mode:
            self.status(t, self.camera, observer_enu, mode, self.system)
        if acoustic_fitted and t - self.acoustic.last_status_t >= self.opts.status_interval_s:
            self.status(t, self.acoustic, None, "listen")
        period = 1.0 / self.opts.detection_rate_hz
        for tr in tracks:
            if t - self._last_detection.get(tr.ident, -math.inf) >= period - 1e-9:
                self.detection(t, tr, tr.p_uav)
                self._last_detection[tr.ident] = t
        for fix in new_fixes:
            self.acoustic_detection(fix, home_alt)
        if not self.keep and len(self._object_ids) > 4 * len(tracks) + 64:
            # a long flight: forget tracks that are long gone
            live = {tr.ident for tr in tracks}
            self._object_ids = {k: v for k, v in self._object_ids.items() if k in live}
            self._last_detection = {k: v for k, v in self._last_detection.items() if k in live}

    def goodbye(self, t: float, observer_enu: Vec | None = None) -> None:
        """Tell the fusion node this node is leaving on purpose (not a failure)."""
        self.status(t, self.camera, observer_enu, self.camera.last_mode or "observe", "SYSTEM_GOODBYE")

    def write_jsonl(self, path: str | Path) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            for m in self.messages:
                fh.write(json.dumps(m, separators=(",", ":")) + "\n")
