"""Copy finished benchmark sets into docs/results and write matrix.json.

    python scripts/collect_results.py runs/m        # collect
    python scripts/collect_results.py --table       # print the README tables

Each set is read from ``runs/m_<key>/`` (or merged from parts ``runs/m_<key>_<n>/``, for
sets flown in pieces). The README tables and the video read ``docs/results/matrix.json``;
``tests/test_published_results.py`` checks that they agree, and that the command stored
for each set names the seeds it was actually flown on.

Two groups, each on encounters never used while developing:

* the **encounter matrix** (seeds 4000-4023): the survey mission against one or two
  intruders, birds, and sensor layouts, with and without avoidance;
* the **sensor-node matrix** (seeds 6000-6023): the standby-and-investigate mission,
  measuring time from a report to a track, two observers, and a jammed link.
"""

from __future__ import annotations

import csv
import json
import shutil
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
RUNS = 24
sys.path.insert(0, str(ROOT / "src"))

from kestrel.bench import risk_ratio, summarise  # noqa: E402

# (key, label, command without --runs/--seed0)
ENCOUNTER_SEED0 = 4000
ENCOUNTER_SETS = [
    ("baseline", "no avoidance (baseline)", "kestrel bench --no-avoidance"),
    ("single", "one camera", "kestrel bench"),
    ("acoustic", "camera + acoustic cueing", "kestrel bench --config configs/acoustic.yaml"),
    ("ring", "five-camera ring", "kestrel bench --config configs/camera_ring.yaml"),
    ("birds_off", "4 birds, no classifier", "kestrel bench --scenario scenarios/airfield_birds.yaml"),
    (
        "birds_on",
        "4 birds + classifier",
        "kestrel bench --scenario scenarios/airfield_birds.yaml --config configs/birds.yaml",
    ),
    ("two", "two intruders", "kestrel bench --intruders 2"),
    ("two_baseline", "two intruders, no avoidance", "kestrel bench --intruders 2 --no-avoidance"),
    (
        "two_acoustic",
        "two intruders, camera + acoustic",
        "kestrel bench --intruders 2 --config configs/acoustic.yaml",
    ),
    ("two_team", "two intruders, two observers", "kestrel bench --intruders 2 --observers 2"),
]
NODE_SEED0 = 6000
NODE_SETS = [
    ("node_camera", "camera only", "kestrel bench --mission standby"),
    (
        "node_acoustic",
        "camera + acoustic cueing",
        "kestrel bench --mission standby --config configs/acoustic.yaml",
    ),
    (
        "node_team",
        "two observers + acoustic",
        "kestrel bench --mission standby --config configs/acoustic.yaml --observers 2",
    ),
    (
        "node_jam_rtl",
        "link jammed 60 s, return home",
        "kestrel bench --mission standby --config configs/acoustic.yaml --jam 60",
    ),
    (
        "node_jam_continue",
        "link jammed 60 s, keep working",
        "kestrel bench --mission standby --config configs/acoustic_continue.yaml --jam 60",
    ),
]


def _rows(path: Path) -> list[dict[str, Any]]:
    with open(path, encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def _typed(row: dict[str, str]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for k, v in row.items():
        if v in ("True", "False"):
            out[k] = v == "True"
        elif v == "":
            out[k] = None
        else:
            try:
                out[k] = float(v) if any(c in v for c in ".e") else int(v)
            except ValueError:
                out[k] = v
    return out


def _collect(prefix: str, key: str, seed0: int) -> dict[str, Any] | None:
    src = Path(f"{prefix}_{key}")
    dst = ROOT / "docs" / "results" / key
    if src.exists():
        dst.mkdir(parents=True, exist_ok=True)
        shutil.copy(src / "summary.json", dst / "summary.json")
        shutil.copy(src / "results.csv", dst / "results.csv")
    else:  # merge parts
        parts = sorted(p for p in Path(prefix).parent.glob(f"{Path(prefix).name}_{key}_*") if p.is_dir())
        if not parts:
            return json.loads((dst / "summary.json").read_text(encoding="utf-8")) if dst.exists() else None
        dst.mkdir(parents=True, exist_ok=True)
        rows = [r for p in parts for r in _rows(p / "results.csv")]
        rows.sort(key=lambda r: r["scenario"])
        with open(dst / "results.csv", "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        first = json.loads((parts[0] / "summary.json").read_text(encoding="utf-8"))
        typed = [_typed(r) for r in rows]
        for r in typed:
            r["seed"] = seed0
            r["avoidance"] = first["avoidance"]
        summary = summarise(typed)
        (dst / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    summary: dict[str, Any] = json.loads((dst / "summary.json").read_text(encoding="utf-8"))
    summary["seed0"] = seed0  # a set flown in parts reports its first part's seed otherwise
    (dst / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def _group(prefix: str, sets: list[tuple[str, str, str]], seed0: int) -> list[dict[str, Any]]:
    out = []
    for key, label, cmd in sets:
        summary = _collect(prefix, key, seed0)
        if summary is None:
            print(f"skipping {key}: no results")
            continue
        command = f"{cmd} --runs {RUNS} --seed0 {seed0}"
        out.append({"key": key, "label": label, "command": command, "summary": summary})
    return out


def main(prefix: str) -> None:
    enc = _group(prefix, ENCOUNTER_SETS, ENCOUNTER_SEED0)
    node = _group(prefix, NODE_SETS, NODE_SEED0)
    by = {s["key"]: s["summary"] for s in enc}
    matrix: dict[str, Any] = {"seed0": ENCOUNTER_SEED0, "runs": RUNS, "sets": enc}
    if "single" in by and "baseline" in by:
        matrix["risk_ratio"] = risk_ratio(by["single"], by["baseline"])
        matrix["risk_ratio_from"] = by["baseline"]["separation_violations"]
        matrix["risk_ratio_to"] = by["single"]["separation_violations"]
    if "two" in by and "two_baseline" in by:
        matrix["risk_ratio_two"] = risk_ratio(by["two"], by["two_baseline"])
    matrix["sensor_node"] = {"seed0": NODE_SEED0, "runs": RUNS, "sets": node}
    path = ROOT / "docs" / "results" / "matrix.json"
    path.write_text(json.dumps(matrix, indent=2), encoding="utf-8")
    print(f"wrote {path} with {len(enc)} + {len(node)} sets")


def table_row(label: str, s: dict[str, Any]) -> str:
    """One README encounter row; tests/test_published_results.py builds the same string."""
    sep = s["min_separation_m"]
    return (
        f"| {label} | {s['separation_violations']}/{s['runs']} ({s['lowc_events']}) | {sep['min']:.1f} m "
        f"| {s['encounters_detected']}/{s['encounters']} | {s['false_alarms_per_flight_hour']:.1f} "
        f"| {s['completed_and_landed']}/{s['runs']} ({s['returned_early_for_traffic']}) |"
    )


def bird_row(label: str, s: dict[str, Any]) -> str:
    return (
        f"| {label} | {s['bird_tracks']} | {s['birds_reported_as_drone']} "
        f"| {s['drone_tracks_reported_as_drone']}/{s['drone_tracks']} |"
    )


def _mp(d: dict[str, float] | None) -> str:
    return "n/a" if d is None else f"{d['median']:.0f} s / {d['p90']:.0f} s"


def node_row(label: str, s: dict[str, Any]) -> str:
    """One README sensor-node row."""
    err = s.get("track_error_all_m")
    err_txt = "n/a" if err is None else f"{err['median']:.1f} m"
    jam = s.get("observed_while_jammed_s")
    jam_txt = "n/a" if jam is None else f"{jam:.0f} of {s['jammed_s']:.0f} s"
    return (
        f"| {label} | {_mp(s.get('cue_to_track_s'))} | {_mp(s.get('entry_to_track_s'))} "
        f"| {s['runs_confirmed']}/{s['runs']} "
        f"| {err_txt} "
        f"| {s['separation_violations']}/{s['runs']} | {s['landed_safely']}/{s['runs']} "
        f"| {jam_txt} | {s['reports_buffered']} |"
    )


def print_tables() -> None:
    m = json.loads((ROOT / "docs" / "results" / "matrix.json").read_text(encoding="utf-8"))
    print(
        "| configuration | runs inside 12 m (entries) | closest approach "
        "| encounters detected 3 s before 30 m | false alarms per flight hour "
        "| missions completed (returned early for traffic) |"
    )
    print("|---|---|---|---|---|---|")
    for st in m["sets"]:
        print(table_row(st["label"], st["summary"]))
    print()
    head = [
        "birds in the area",
        "bird tracks",
        "bird tracks reported as a drone",
        "drone tracks reported as a drone",
    ]
    print("| " + " | ".join(head) + " |")
    print("|---|---|---|---|")
    for st in m["sets"]:
        if st["key"].startswith("birds"):
            print(bird_row(st["label"], st["summary"]))
    print()
    head = [
        "sensor node",
        "report to track (median / p90)",
        "geofence entry to track (median / p90)",
        "runs with a drone confirmed",
        "intruder position error (median)",
        "runs inside 12 m",
        "landed",
        "intruder held while jammed",
        "reports sent late",
    ]
    print("| " + " | ".join(head) + " |")
    print("|" + "---|" * len(head))
    for st in m.get("sensor_node", {}).get("sets", []):
        print(node_row(st["label"], st["summary"]))


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--table":
        print_tables()
    else:
        main(sys.argv[1] if len(sys.argv) > 1 else "runs/m")
