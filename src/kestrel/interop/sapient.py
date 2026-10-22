"""SAPIENT reports: Kestrel's tracks in the message set NATO uses for counter-UAS sensors.

SAPIENT is the UK Dstl sensor-integration standard (BSI Flex 335), trialled at NATO's
counter-UAS Technical Interoperability Exercise and going through NATO standardisation.
A SAPIENT *node* (a sensor) registers with a fusion node, sends a status report at least
once a minute, and sends a detection report per object it sees. This module builds those
messages from Kestrel's state so its output can feed a SAPIENT fusion node next to other
sensors.

Field names, enumerations and units follow the published Interface Control Document,
version 7.0 (Dstl, January 2023): the ``SapientMessage`` wrapper (``timestamp``,
``node_id``, one content field), ``Registration``, ``StatusReport`` and
``DetectionReport``. Messages are produced as dictionaries in the proto3 JSON layout
(oneof member as the key, enum values by name, ``Timestamp`` as RFC 3339). The ICD's
wire format is protobuf binary with a 4-byte little-endian length prefix over TCP; the
``.proto`` files are available from Dstl on request rather than published, so the binary
encoding is a mechanical step left to whoever holds them. No conformance test against a
SAPIENT test harness has been run.

Mapping choices, each stated rather than hidden:

* **Coordinates**: ``Location`` in ``UTM_M`` (easting, northing, altitude in metres; the
  zone is declared once at registration, as the ICD specifies) on the ``WGS84_G`` datum
  (altitude above the geoid, as ArduPilot home altitude is). Errors are one-sigma, metres.
* **Velocity**: ``ENUVelocity`` from the tracker, one-sigma errors.
* **Two nodes**: the observer's cameras (``NODE_TYPE_CAMERA``) and, when fitted, the ground
  acoustic network (``NODE_TYPE_ACOUSTIC``). An acoustic fix has no altitude, so it is sent
  at the middle of the altitude band with ``z_error`` covering the band, never on the
  ground plane (which is what the ICD assumes when ``z`` is omitted).
* **Oneof members**: where the ICD names a ``oneof`` but not its members (``LocationType``'s
  ``coordinates_oneof`` and ``datum_oneof``; the location and velocity oneofs of
  ``DetectionReport``), the member names here are the snake_case of the type
  (``location``, ``enu_velocity``, ``location_units``, ``location_datum``) and must be checked
  against the ``.proto`` files.
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
from pathlib import Path
from typing import Any

import numpy as np

from kestrel.geometry import Vec
from kestrel.interop.utm import enu_to_lla, to_utm, zone_of
from kestrel.perception.acoustic import AcousticFix
from kestrel.perception.tracker import TrackEstimate

ICD_VERSION = "7.0"
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


@dataclass
class SapientOptions:
    origin_lla: tuple[float, float, float]  # geodetic position of the local ENU origin
    epoch_s: float = 1767225600.0  # wall-clock time of sim t = 0 (2026-01-01T00:00:00Z)
    status_interval_s: int = 1
    detection_rate_hz: float = 2.0  # per object; the ICD leaves rate to the deployment
    altitude_band_m: tuple[float, float] = (0.0, 60.0)  # AGL band an acoustic fix may lie in
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

    def __post_init__(self) -> None:
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

    def _emit(self, t: float, node: _Node, content_key: str, content: Message) -> None:
        msg = {"timestamp": rfc3339(self.opts.epoch_s + t), "node_id": node.node_id, content_key: content}
        self.messages.append(msg)
        if self.sink is not None:
            self.sink(msg)

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

    # ------------------------------------------------------------------ messages
    def registration(self, t: float, node: _Node) -> None:
        mode = {
            "mode_name": node.mode,
            "mode_description": "autonomous observation",
            "settle_time": _duration(0),
            "scan_type": "SCAN_TYPE_STEERABLE" if node is self.camera else "SCAN_TYPE_FIXED",
            "tracking_type": "TRACKING_TYPE_TRACK" if node is self.camera else "TRACKING_TYPE_NONE",
            "detection_definition": {"location_type": self._location_type()},
            "task": [],
            "mode_type": "MODE_TYPE_PERMANENT",
        }
        content = {
            "node_type": [node.node_type],
            "icd_version": ICD_VERSION,
            "name": node.name,
            "short_name": node.short_name,
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
            content["sensor_location"] = self.location(sensor_enu)
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
            self.status(t, self.camera, observer_enu, mode)
        if acoustic_fitted and t - self.acoustic.last_status_t >= self.opts.status_interval_s:
            self.status(t, self.acoustic, None, "listen")
        period = 1.0 / self.opts.detection_rate_hz
        for tr in tracks:
            if t - self._last_detection.get(tr.ident, -math.inf) >= period - 1e-9:
                self.detection(t, tr, tr.p_uav)
                self._last_detection[tr.ident] = t
        for fix in new_fixes:
            self.acoustic_detection(fix, home_alt)

    def write_jsonl(self, path: str | Path) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            for m in self.messages:
                fh.write(json.dumps(m, separators=(",", ":")) + "\n")
