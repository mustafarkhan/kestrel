"""Copy a finished benchmark matrix into docs/results and write matrix.json.

    python scripts/collect_results.py runs/m

expects ``runs/m_<key>/summary.json`` and ``results.csv`` for each set below (the ring is
run in parts, ``runs/m_ring_<n>``, and merged here). The README table and the video read
``docs/results/matrix.json``; ``tests/test_published_results.py`` checks they agree.
"""

from __future__ import annotations

import csv
import json
import shutil
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
SEED0 = 4000  # the published set: never used while developing (1000-2999 tuning, 3000 diagnosis)
RUNS = 24
sys.path.insert(0, str(ROOT / "src"))

from kestrel.bench import risk_ratio, summarise  # noqa: E402

SETS = [
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


def main(prefix: str) -> None:
    out = ROOT / "docs" / "results"
    sets = []
    for key, label, cmd in SETS:
        src = Path(f"{prefix}_{key}")
        dst = out / key
        dst.mkdir(parents=True, exist_ok=True)
        if src.exists():
            shutil.copy(src / "summary.json", dst / "summary.json")
            shutil.copy(src / "results.csv", dst / "results.csv")
        else:  # merge parts
            parts = sorted(p for p in Path(prefix).parent.glob(f"{Path(prefix).name}_{key}_*") if p.is_dir())
            if not parts:
                print(f"skipping {key}: no results")
                continue
            rows = [r for p in parts for r in _rows(p / "results.csv")]
            rows.sort(key=lambda r: r["scenario"])
            with open(dst / "results.csv", "w", newline="", encoding="utf-8") as fh:
                w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
            first = json.loads((parts[0] / "summary.json").read_text(encoding="utf-8"))
            typed = [_typed(r) for r in rows]
            for r in typed:
                r["seed"] = first["seed0"]
                r["avoidance"] = first["avoidance"]
            summary = summarise(typed)
            summary["seed0"] = first["seed0"]
            (dst / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        summary = json.loads((dst / "summary.json").read_text(encoding="utf-8"))
        sets.append(
            {"key": key, "label": label, "command": f"{cmd} --runs 24 --seed0 3000", "summary": summary}
        )
    by = {s["key"]: s["summary"] for s in sets}
    matrix: dict[str, Any] = {"seed0": SEED0, "runs": RUNS, "sets": sets}
    if "single" in by and "baseline" in by:
        matrix["risk_ratio"] = risk_ratio(by["single"], by["baseline"])
        matrix["risk_ratio_from"] = by["baseline"]["separation_violations"]
        matrix["risk_ratio_to"] = by["single"]["separation_violations"]
    if "two" in by and "two_baseline" in by:
        matrix["risk_ratio_two"] = risk_ratio(by["two"], by["two_baseline"])
    (out / "matrix.json").write_text(json.dumps(matrix, indent=2), encoding="utf-8")
    print(f"wrote {out / 'matrix.json'} with {len(sets)} sets")


def table_row(label: str, s: dict[str, Any]) -> str:
    """One README results row; tests/test_published_results.py builds the same string."""
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


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--table":
        print_tables()
    else:
        main(sys.argv[1] if len(sys.argv) > 1 else "runs/m")
