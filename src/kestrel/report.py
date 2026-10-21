"""Write a run's report folder: metrics, plan, plots, log and optional GIF."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from kestrel.runtime import RunResult


def write_report(res: RunResult, metrics: dict[str, Any], out: Path, gif: bool = False) -> None:
    out.mkdir(parents=True, exist_ok=True)
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    if res.report is not None:
        (out / "plan.json").write_text(res.report.mission.model_dump_json(indent=2), encoding="utf-8")
        (out / "validation.txt").write_text(res.report.summary() + "\n", encoding="utf-8")
    if not res.flown:
        return
    from kestrel.viz.plots import overview_map, timeline

    arrays: dict[str, Any] = dict(res.log.arrays())
    np.savez_compressed(out / "flight.npz", allow_pickle=False, **arrays)
    overview_map(res, out / "overview.png")
    timeline(res, out / "timeline.png")
    if gif and res.log.frames:
        from kestrel.viz.animate import make_gif

        first_track = next((e.t for e in res.events if e.kind == "interrupt"), None)
        start = max(0.0, first_track - 8.0) if first_track is not None else 0.0
        make_gif(res, out / "demo.gif", t_start=start, t_end=start + 45.0, fps=8)
