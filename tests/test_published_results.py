"""The README's results must match the committed benchmark results they came from."""

from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "docs" / "results"

_spec = importlib.util.spec_from_file_location("collect_results", ROOT / "scripts" / "collect_results.py")
assert _spec is not None and _spec.loader is not None
collect = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(collect)


def _matrix() -> dict[str, Any]:
    data: dict[str, Any] = json.loads((RESULTS / "matrix.json").read_text(encoding="utf-8"))
    return data


def test_readme_rows_match_the_matrix() -> None:
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    missing = [
        st["key"] for st in _matrix()["sets"] if collect.table_row(st["label"], st["summary"]) not in readme
    ]
    assert not missing, f"README out of date with docs/results/matrix.json: {missing}"
    for st in _matrix()["sets"]:
        if st["key"].startswith("birds"):
            assert collect.bird_row(st["label"], st["summary"]) in readme, st["key"]


def test_risk_ratio_is_stated_as_computed() -> None:
    m = _matrix()
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert f"risk ratio of {m['risk_ratio']:.2f}" in readme


def test_summaries_match_per_run_rows() -> None:
    for st in _matrix()["sets"]:
        with open(RESULTS / st["key"] / "results.csv", encoding="utf-8") as fh:
            rows = list(csv.DictReader(fh))
        s = st["summary"]
        assert len(rows) == s["runs"], st["key"]
        landed = sum(1 for r in rows if r["completed"] == "True" and r["landed"] == "True")
        assert landed == s["completed_and_landed"], st["key"]
        inside = sum(1 for r in rows if float(r["time_below_min_separation_s"] or 0) > 0)
        assert inside == s["separation_violations"], st["key"]
        closest = min(float(r["min_separation_m"]) for r in rows if r["min_separation_m"])
        assert abs(closest - s["min_separation_m"]["min"]) < 1e-6, st["key"]
