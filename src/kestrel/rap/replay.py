"""Fuse a recorded SAPIENT message log into a recognised air picture.

Input is JSON lines, one SAPIENT message per line in the proto3 JSON layout (as Kestrel
and its simulated sensors write them; binary captures need converting with the ``.proto``
files first). The local frame's origin is taken from the first located report, and time
from the first timestamp, so no scenario file is needed. Output is JSON lines, one picture
per ``period_s``: every confirmed track with its position (local ENU and WGS84), one-sigma
error, velocity, class, drone probability and standard identity.
"""

from __future__ import annotations

import json
import math
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from kestrel.config import WorldConfig
from kestrel.interop.sapient import SapientExporter, SapientOptions
from kestrel.interop.utm import enu_to_lla, from_utm
from kestrel.rap.fusion import FusionNode, Message


def _epoch(ts: str) -> float:
    return datetime.fromisoformat(ts.replace("Z", "+00:00")).timestamp()


def _zone(msgs: list[Message]) -> str | None:
    for m in msgs:
        reg = m.get("registration")
        if not isinstance(reg, dict):
            continue
        zone = ((reg.get("status_definition") or {}).get("location_definition") or {}).get("zone")
        if zone:
            return str(zone)
    return None


def _origin(msgs: list[Message], zone: str | None) -> tuple[float, float, float]:
    for m in msgs:
        loc = (m.get("detection_report") or {}).get("location")
        if not isinstance(loc, dict):
            continue
        system = str(loc.get("coordinate_system", ""))
        if system.endswith("LAT_LNG_DEG_M"):
            return float(loc["y"]), float(loc["x"]), 0.0
        if system.endswith("UTM_M") and zone:
            north = zone[-1].upper() >= "N"
            lat, lon = from_utm(float(loc["x"]), float(loc["y"]), int(zone[:-1]), north)
            return lat, lon, 0.0
    raise ValueError("no located detection report to take the frame origin from")


def load(path: str | Path) -> list[Message]:
    with open(path, encoding="utf-8") as fh:
        msgs = [json.loads(line) for line in fh if line.strip()]
    return sorted(msgs, key=lambda m: _epoch(str(m["timestamp"])))


def fuse_messages(
    msgs: list[Message], whitelist: list[str] | None = None, period_s: float = 1.0
) -> tuple[FusionNode, list[dict[str, Any]]]:
    if not msgs:
        raise ValueError("no messages")
    origin = _origin(msgs, _zone(msgs))
    epoch0 = _epoch(str(msgs[0]["timestamp"]))
    geo = SapientExporter(SapientOptions(origin_lla=origin, epoch_s=epoch0))
    fusion = FusionNode(
        geo, WorldConfig(fence=[(-1e5, -1e5), (1e5, -1e5), (1e5, 1e5), (-1e5, 1e5)]), whitelist
    )
    pictures: list[dict[str, Any]] = []
    next_t = 0.0
    for m in msgs:
        t = _epoch(str(m["timestamp"])) - epoch0
        while t >= next_t:
            fusion.step(next_t)
            pictures.append(snapshot(fusion, next_t, origin, epoch0))
            next_t += period_s
        fusion.ingest(m)
    fusion.step(next_t)
    pictures.append(snapshot(fusion, next_t, origin, epoch0))
    return fusion, pictures


def snapshot(
    fusion: FusionNode, t: float, origin: tuple[float, float, float], epoch0: float
) -> dict[str, Any]:
    tracks = []
    for tr in fusion.picture(t):
        lat, lon, alt = enu_to_lla(origin, float(tr.pos[0]), float(tr.pos[1]), float(tr.pos[2]))
        tracks.append(
            {
                "id": tr.ident,
                "lat": round(lat, 7),
                "lon": round(lon, 7),
                "alt_m": round(alt, 1),
                "enu_m": [round(float(v), 1) for v in tr.pos],
                "sigma_m": round(math.sqrt(max(float(tr.p[0:3, 0:3].trace()), 0.0)), 1),
                "vel_mps": [round(float(v), 1) for v in tr.vel],
                "class": fusion.class_of(tr),
                "p_uav": round(tr.p_uav, 3),
                "identity": tr.identity,
                "remote_id": tr.remote_id,
                "sensors": sorted(tr.node_types),
            }
        )
    return {
        "timestamp": datetime.fromtimestamp(epoch0 + t, tz=UTC).isoformat(),
        "t": round(t, 2),
        "tracks": tracks,
    }


def fuse_file(
    src: str | Path, dst: str | Path, whitelist: list[str] | None = None, period_s: float = 1.0
) -> int:
    fusion, pictures = fuse_messages(load(src), whitelist, period_s)
    with open(dst, "w", encoding="utf-8") as fh:
        for p in pictures:
            fh.write(json.dumps(p, separators=(",", ":")) + "\n")
    return fusion.reports
