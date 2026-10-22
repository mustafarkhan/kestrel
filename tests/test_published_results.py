"""The README's benchmark table must match the committed results it came from."""

from __future__ import annotations

import csv
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "docs" / "results"


def _summary() -> dict[str, object]:
    return json.loads((RESULTS / "bench_summary.json").read_text(encoding="utf-8"))


def test_readme_table_matches_summary() -> None:
    s = _summary()
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    sep = s["min_separation_m"]
    err = s["track_error_median_m"]
    assert isinstance(sep, dict) and isinstance(err, dict)
    expected = [
        f"{s['completed_and_landed']}/{s['runs']}",
        f"{s['runs_with_contact_tracked']}/{s['runs']}",
        f"{s['separation_violations']}/{s['runs']}",
        f"{sep['min']:.1f} m",
        f"{err['median']:.1f} m",
        f"{err['p90']:.1f} m",
        f"{100 * float(s['target_in_fov_fraction_median']):.0f}%",  # type: ignore[arg-type]
    ]
    missing = [e for e in expected if e not in readme]
    assert not missing, f"README out of date with docs/results/bench_summary.json: {missing}"


def test_summary_matches_per_run_csv() -> None:
    s = _summary()
    with open(RESULTS / "bench_results.csv", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == s["runs"]
    landed = sum(1 for r in rows if r["completed"] == "True" and r["landed"] == "True")
    assert landed == s["completed_and_landed"]
    violations = sum(
        1 for r in rows if r["time_below_min_separation_s"] and float(r["time_below_min_separation_s"]) > 0
    )
    assert violations == s["separation_violations"]
    longest = max(float(r["time_below_min_separation_s"] or 0) for r in rows)
    closest = sorted(
        float(r["min_separation_m"]) for r in rows if float(r["time_below_min_separation_s"] or 0) > 0
    )
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert f"at most {longest:.1f} s" in readme and f"none longer than {longest:.1f} s" in readme
    assert ", ".join(f"{c:.1f}" for c in closest[:-1]) + f" and\n{closest[-1]:.1f} m" in readme
