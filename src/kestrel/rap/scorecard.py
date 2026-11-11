"""Score any air picture against the threat library, with the same measures for every system.

Three steps, each a file format anyone can read or write:

1. ``export`` writes one threat situation as what a system under test would receive and
   what it will be judged against: ``sensors.jsonl`` (the simulated sensors' SAPIENT messages,
   JSON layout), ``truth.jsonl`` (where every aircraft and bird really was) and ``site.json``
   (origin, fence, protected areas, friendly serials).
2. The system under test produces a **picture file**: JSON lines, one picture per line,
   ``{"t": seconds, "tracks": [{"id", "lat", "lon", "alt_m" (or "enu_m"), "class",
   "identity"}]}``. ``kestrel fuse`` and ``kestrel fusion`` write this format; a third-party
   fusion engine's output needs a small converter.
3. ``score_files`` scores that picture against the truth with the benchmark's own scoring
   (``rap/score.py``), and ``run_library`` does all of it for Kestrel's own fusion node, which
   gives the reference column.

There is no pass mark. A threshold would be this project's opinion; a table of the same
measures on the same situations, next to a published reference, is something two parties
can read the same way.
"""

from __future__ import annotations

import json
import math
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from kestrel.config import Scenario
from kestrel.geofence import Geofence
from kestrel.interop.utm import enu_to_lla, lla_to_enu
from kestrel.rap.picture import ALTITUDE_BAND, make_geo, run_picture
from kestrel.rap.score import PictureScore
from kestrel.rap.sensors import SensorNetwork
from kestrel.rap.threats import LIBRARY_VERSION, THREATS, build
from kestrel.rap.truth import Truth, TruthObject

STEP_S = 0.5
STALE_S = 2.0  # a picture older than this at scoring time counts as no picture
SEED0 = 9000  # the published reference uses seeds 9000 upward

# measure -> (label, "up" if more is better else "down", format)
MEASURES: dict[str, tuple[str, str, str]] = {
    "hostile_coverage": ("hostile drone-time tracked", "up", "pct"),
    "first_track_s": ("entry to first track (median)", "down", "s"),
    "drones_never_tracked": ("drones never tracked", "down", "n"),
    "drone_as_uav": ("tracked drone-time shown as UAV", "up", "pct"),
    "drone_as_bird": ("tracked drone-time shown as bird", "down", "pct"),
    "track_to_uav_s": ("first track to UAV, hostile (median)", "down", "s"),
    "bird_time_as_uav": ("bird-time shown as UAV", "down", "pct"),
    "false_mean": ("false tracks at a time (mean)", "down", "n"),
    "duplicate_ratio": ("duplicate tracks per object", "down", "pct"),
    "ids_per_drone": ("track identities per drone", "down", "n"),
    "friend_identified": ("friend-time shown as friend", "up", "pct"),
    "hostile_as_friend_s": ("hostile shown as friend", "down", "s"),
    "phantom_friend_s": ("friend shown where none is", "down", "s"),
    "gospa_m": ("GOSPA", "down", "m"),
}


@dataclass
class ExtTrack:
    """A track of any system's picture, reduced to what is scored."""

    ident: Any
    pos: np.ndarray
    identity: str
    cls: str


class _Classes:
    """Stands in for the fusion node where the scoring asks what class a track is shown as."""

    @staticmethod
    def class_of(tr: Any) -> str:
        return str(tr.cls)


def _site(sc: Scenario) -> dict[str, Any]:
    assert sc.sensors is not None
    origin = sc.world.origin_lla
    return {
        "threat_library": LIBRARY_VERSION,
        "scenario": sc.name,
        "seed": sc.seed,
        "duration_s": sc.max_duration_s,
        "step_s": STEP_S,
        "origin_lla": list(origin),
        "fence_enu_m": [list(p) for p in sc.world.fence],
        "fence_lla": [list(enu_to_lla(origin, x, y, 0.0)[:2]) for x, y in sc.world.fence],
        "protected": {k: v.model_dump() for k, v in sc.sensors.protected.items()},
        "friendly_serials": list(sc.sensors.whitelist),
    }


def _truth_line(t: float, objects: list[TruthObject], origin: tuple[float, float, float]) -> dict[str, Any]:
    out = []
    for o in objects:
        lat, lon, alt = enu_to_lla(origin, float(o.pos[0]), float(o.pos[1]), float(o.pos[2]))
        out.append(
            {
                "key": o.key,
                "kind": o.kind,
                "role": o.role,
                "enu_m": [round(float(v), 2) for v in o.pos],
                "lat": round(lat, 7),
                "lon": round(lon, 7),
                "alt_m": round(alt, 1),
            }
        )
    return {"t": t, "objects": out}


def export(sc: Scenario, out_dir: Path) -> dict[str, Path]:
    """Write a situation as sensor traffic, truth and site description."""
    assert sc.sensors is not None, "scenario has no sensor network"
    out_dir.mkdir(parents=True, exist_ok=True)
    geo = make_geo(sc, sc.seed)
    truth = Truth(sc)
    net = SensorNetwork(sc.sensors, sc.world, geo, sc.seed, None, ALTITUDE_BAND)
    paths = {
        k: out_dir / f"{k}.{ext}" for k, ext in (("sensors", "jsonl"), ("truth", "jsonl"), ("site", "json"))
    }
    with paths["sensors"].open("w", encoding="utf-8") as fs, paths["truth"].open("w", encoding="utf-8") as ft:
        t = 0.0
        while t <= sc.max_duration_s:
            objects = truth.objects(t)
            for msg in net.step(t, objects):
                fs.write(json.dumps(msg) + "\n")
            ft.write(json.dumps(_truth_line(t, objects, sc.world.origin_lla)) + "\n")
            t = round(t + STEP_S, 6)
    paths["site"].write_text(json.dumps(_site(sc), indent=1), encoding="utf-8")
    return paths


def _lines(path: Path) -> list[dict[str, Any]]:
    with path.open(encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


def _ext_tracks(picture: dict[str, Any], origin: tuple[float, float, float]) -> list[ExtTrack]:
    """Tracks of one picture in the site's frame. Latitude and longitude are used where given:
    a system's own local frame (``enu_m``) need not share the site's origin."""
    out = []
    for k, tr in enumerate(picture.get("tracks") or []):
        if tr.get("lat") is not None and tr.get("lon") is not None:
            lla = (float(tr["lat"]), float(tr["lon"]), float(tr.get("alt_m", origin[2])))
            pos = np.array(lla_to_enu(origin, *lla))
        else:
            pos = np.array([float(v) for v in tr.get("enu_m") or []][:3])
        if pos.shape != (3,) or not np.all(np.isfinite(pos)):
            continue  # a track with no usable position cannot be scored
        identity, cls = str(tr.get("identity", "UNKNOWN")), str(tr.get("class", "Unknown"))
        out.append(ExtTrack(tr.get("id", k), pos, identity, cls))
    return out


def score_files(picture_path: Path, truth_path: Path, site_path: Path) -> dict[str, float | None]:
    """Score a picture file against a truth file. At each truth step the latest picture not
    newer than the step is used, unless it is more than ``STALE_S`` old."""
    site = json.loads(site_path.read_text(encoding="utf-8"))
    origin = (float(site["origin_lla"][0]), float(site["origin_lla"][1]), float(site["origin_lla"][2]))
    fence = Geofence([(float(x), float(y)) for x, y in site["fence_enu_m"]])
    pictures = sorted(_lines(picture_path), key=lambda p: float(p["t"]))
    score = PictureScore()
    k = -1
    for line in _lines(truth_path):
        t = float(line["t"])
        while k + 1 < len(pictures) and float(pictures[k + 1]["t"]) <= t + 1e-9:
            k += 1
        fresh = k >= 0 and t - float(pictures[k]["t"]) <= STALE_S
        tracks = _ext_tracks(pictures[k], origin) if fresh else []
        objects = [
            TruthObject(
                o["key"], o["kind"], o["role"], np.array(o["enu_m"], dtype=float), np.zeros(3), 0.0, 0.0
            )
            for o in line["objects"]
        ]
        score.add(t, float(site.get("step_s", STEP_S)), objects, tracks, _Classes(), fence)  # type: ignore[arg-type]
    return score.summary()


def _one(args: tuple[str, int]) -> dict[str, Any]:
    name, seed = args
    return {"threat": name, "seed": seed, **run_picture(build(name, seed), dt=STEP_S).score.summary()}


def mean_rows(rows: list[dict[str, Any]]) -> dict[str, float | None]:
    """Mean of each measure over the runs that have it; seconds of error are totals."""
    out: dict[str, float | None] = {}
    for key in MEASURES:
        vals = [float(r[key]) for r in rows if r.get(key) is not None]
        if not vals:
            out[key] = None
        elif key in ("hostile_as_friend_s", "phantom_friend_s", "drones_never_tracked"):
            out[key] = round(float(sum(vals)), 1)
        else:
            out[key] = round(float(np.mean(vals)), 3)
    return out


def run_library(names: list[str], seeds: int, seed0: int = SEED0, workers: int = 2) -> dict[str, Any]:
    """Kestrel's own fusion node on the library: the reference a system under test is set beside."""
    jobs = [(n, seed0 + i) for n in names for i in range(seeds)]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        rows = list(pool.map(_one, jobs))
    card: dict[str, Any] = {
        "threat_library": LIBRARY_VERSION,
        "system": "Kestrel fusion node, simulated ground sensors",
        "seeds": [seed0, seed0 + seeds - 1],
        "threats": {},
        "runs": rows,
    }
    for n in names:
        card["threats"][n] = mean_rows([r for r in rows if r["threat"] == n])
    return card


def _fmt(key: str, v: float | None) -> str:
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "n/a"
    kind = MEASURES[key][2]
    if kind == "pct":
        return f"{100 * v:.0f}%"
    if kind == "s":
        return f"{v:.0f} s"
    if kind == "m":
        return f"{v:.1f} m"
    return f"{v:.2f}".rstrip("0").rstrip(".") if v != int(v) else f"{int(v)}"


def markdown(card: dict[str, Any], reference: dict[str, Any] | None = None) -> str:
    """The scorecard as a table per threat; with a reference, the two side by side."""
    lo, hi = card["seeds"]
    lines = [
        f"# Air-picture scorecard: {card['system']}",
        "",
        f"Threat library {card['threat_library']}, seeds {lo} to {hi} ({hi - lo + 1} per threat). "
        "Measures and file formats: `docs/scorecard.md`. Simulated sensors and simulated sky.",
        "",
    ]
    for name, row in card["threats"].items():
        th = THREATS[name]
        lines += [f"## {th.title} (`{name}`)", "", f"{th.tests[0].upper()}{th.tests[1:]}.", ""]
        ref = (reference or {}).get("threats", {}).get(name)
        head = "| measure | better | result |" + (" reference |" if ref else "")
        lines += [head, "|---|---|---|" + ("---|" if ref else "")]
        for key, (label, way, _) in MEASURES.items():
            if row.get(key) is None and not (ref and ref.get(key) is not None):
                continue
            mark = "**" if key in th.watch else ""
            better = "higher" if way == "up" else "lower"
            cells = f"| {mark}{label}{mark} | {better} | {_fmt(key, row.get(key))} |"
            lines.append(cells + (f" {_fmt(key, ref.get(key))} |" if ref else ""))
        lines.append("")
    lines.append("Measures in bold are the ones the threat is aimed at.")
    return "\n".join(lines) + "\n"
