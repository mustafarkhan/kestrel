"""Picture benchmark: each sensor alone against all of them fused, on the same situations."""

from __future__ import annotations

import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np

from kestrel.rap.picture import flank_scenario, run_picture

# name -> (sensors included, whitelist known to the fusion node)
CONFIGS: dict[str, tuple[set[str] | None, bool]] = {
    "radar": ({"radar"}, True),
    "rf": ({"rf"}, True),
    "acoustic": ({"acoustic"}, True),
    "remote_id": ({"remote_id"}, True),
    "fused_no_whitelist": (None, False),
    "fused": (None, True),
}


def _one(args: tuple[int, str]) -> dict[str, Any]:
    seed, name = args
    include, whitelist = CONFIGS[name]
    sc = flank_scenario(seed)
    res = run_picture(sc, include, whitelist)
    return {"seed": seed, "spoofer": len(sc.intruders) > 6, **res.score.summary()}


def aggregate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {"runs": len(rows), "runs_with_spoofer": sum(bool(r["spoofer"]) for r in rows)}
    keys = [k for k in rows[0] if k not in ("seed", "spoofer")]
    for k in keys:
        vals = [r[k] for r in rows if r[k] is not None]
        if not vals:
            out[k] = None
        elif k in (
            "hostile_as_friend_s",
            "phantom_friend_s",
            "drones_never_tracked",
            "hostiles_tracked",
            "hostiles_classified",
        ):
            out[k] = round(float(np.sum(vals)), 1)
        else:
            out[k] = round(float(np.mean(vals)), 3)
    return out


def run_picture_benchmark(
    n: int, seed0: int, out_dir: Path, configs: list[str], workers: int = 2
) -> dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    summary: dict[str, Any] = {}
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for name in configs:
            rows = list(pool.map(_one, [(seed0 + i, name) for i in range(n)]))
            (out_dir / f"{name}.json").write_text(json.dumps(rows, indent=1))
            summary[name] = aggregate(rows)
            print(name, json.dumps(summary[name]), flush=True)
    summary["_meta"] = {"seeds": [seed0, seed0 + n - 1], "duration_s": 600, "area_m": 1600}
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=1))
    return summary


# --------------------------------------------------------------------------- with Kestrel

LOOP_MISSION = "take off to 25 m, stand by at the hangar for 4 minutes, then return home"


def loop_scenario(base: Any, i: int, seed0: int) -> Any:
    """Encounter ``i``: the airfield picture scenario with a freshly drawn radio-silent hostile.

    The hostile is drawn exactly as the single-intruder benchmark draws its intruder, then
    made radio-silent and quiet, with a radar micro-Doppler score drawn between 0.55 and
    0.75: the radar sees it but cannot say with confidence that it is a drone.
    """
    from kestrel.bench import make_scenario

    sc = make_scenario(base, i, seed0)
    rng = np.random.default_rng(seed0 + i + 90_000)
    hostile = sc.intruders[0].model_copy(
        update={"rf_link": False, "propulsion": "quiet", "radar_class_cue": float(rng.uniform(0.55, 0.75))}
    )
    friendly = base.intruders[1]
    return sc.model_copy(update={"intruders": [hostile, friendly], "birds": base.birds})


def _loop_one(args: tuple[Any, Any, bool]) -> dict[str, Any]:
    from kestrel.interop.sapient import SapientExporter, SapientOptions
    from kestrel.metrics import compute_metrics
    from kestrel.rap.loop import AirPicture
    from kestrel.runtime import run

    cfg, sc, tasking = args
    exp = SapientExporter(
        SapientOptions(origin_lla=sc.world.origin_lla, seed=sc.seed, altitude_band_m=(0.0, 60.0))
    )
    ap = AirPicture(sc, exp, tasking=tasking)
    res = run(LOOP_MISSION, cfg, sc, sapient=exp, air_picture=ap)
    m = compute_metrics(res)
    keep = ("completed", "landed", "min_separation_m", "lowc_events", "time_below_min_separation_s")
    return {"scenario": sc.name, "tasking": tasking, **{k: m.get(k) for k in keep}, **ap.summary()}


def run_loop_benchmark(
    cfg: Any,
    base: Any,
    n: int,
    seed0: int,
    out_dir: Path,
    workers: int = 2,
    modes: tuple[bool, ...] = (False, True),
) -> dict[str, Any]:
    """Fly encounters with the fusion node tasking Kestrel (``True``) or only listening (``False``)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    summary: dict[str, Any] = {}
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for tasking in modes:
            name = "tasked" if tasking else "untasked"
            jobs = [(cfg, loop_scenario(base, i, seed0), tasking) for i in range(n)]
            rows = list(pool.map(_loop_one, jobs))
            (out_dir / f"{name}.json").write_text(json.dumps(rows, indent=1))
            agg = aggregate([{k: v for k, v in r.items() if k != "scenario"} | {"seed": 0, "spoofer": False}
                             for r in rows])  # fmt: skip
            classify = [r["track_to_uav_s"] for r in rows if r["track_to_uav_s"] is not None]
            agg["track_to_uav_median_s"] = round(float(np.median(classify)), 1) if classify else None
            agg["track_to_uav_p90_s"] = round(float(np.percentile(classify, 90)), 1) if classify else None
            agg["completed_runs"] = sum(bool(r["completed"]) for r in rows)
            agg["runs_with_lowc"] = sum((r["lowc_events"] or 0) > 0 for r in rows)
            summary[name] = agg
            print(name, json.dumps(agg), flush=True)
    summary["_meta"] = {"seeds": [seed0, seed0 + n - 1], "mission": LOOP_MISSION}
    for name, agg in summary.items():
        (out_dir / f"summary_{name.strip('_')}.json").write_text(json.dumps(agg, indent=1))
    return summary
