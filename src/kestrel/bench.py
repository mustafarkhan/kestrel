"""Monte Carlo benchmark: one mission, many randomised encounters.

Each run draws the intruder pattern, speed, altitude and position, the wind and
the random seed (which also changes the sky). Results are aggregated into the
table published in the README, so the numbers there are reproducible with
``kestrel bench``.
"""

from __future__ import annotations

import csv
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np

from kestrel.config import Config, IntruderConfig, Scenario, WindParams
from kestrel.metrics import compute_metrics
from kestrel.runtime import run

BENCH_MISSION = (
    "take off to 25 m, survey the north field at 25 m with 30 m spacing, "
    "and if you see a drone, follow it from 30 m for 45 seconds, then return home"
)


def _intruder(rng: np.random.Generator) -> IntruderConfig:
    pattern = str(rng.choice(["lemniscate", "orbit", "waypoints"]))
    cx, cy = rng.uniform(-30, 30), rng.uniform(60, 100)
    alt = rng.uniform(26, 40)
    speed = rng.uniform(2.0, 6.0)
    wps: list[tuple[float, float, float]] = []
    if pattern == "waypoints":
        wps = [(cx + rng.uniform(-45, 45), cy + rng.uniform(-30, 30), alt) for _ in range(4)]
    return IntruderConfig(
        pattern=pattern,
        center=(cx, cy, alt),
        size=(rng.uniform(25, 45), rng.uniform(15, 30)),
        speed=speed,
        waypoints=wps,
        appear_at_s=float(rng.uniform(0, 40)),
    )


def make_scenario(base: Scenario, i: int, seed0: int = 1000, intruders: int = 1) -> Scenario:
    """Encounter ``i`` of a benchmark set.

    The first intruder and the wind are drawn exactly as in single-intruder sets, so adding
    intruders (from a separate random stream) leaves the first encounter unchanged.
    """
    rng = np.random.default_rng(seed0 + i)
    first = _intruder(rng)
    wdir = rng.uniform(0, 2 * np.pi)
    wspd = rng.uniform(0, 5)
    world = base.world.model_copy(
        update={
            "wind": WindParams(
                mean=(wspd * np.cos(wdir), wspd * np.sin(wdir), 0.0), gust_sigma=float(rng.uniform(0.3, 1.5))
            )
        }
    )
    extra_rng = np.random.default_rng(seed0 + i + 50_000)
    extra = [_intruder(extra_rng) for _ in range(intruders - 1)]
    return base.model_copy(
        update={
            "name": f"{base.name}-mc{i:03d}",
            "seed": seed0 + i,
            "world": world,
            "intruders": [first, *extra],
        }
    )


def _one(args: tuple[Config, Scenario, bool]) -> dict[str, Any]:
    cfg, sc, avoidance = args
    res = run(BENCH_MISSION, cfg, sc, avoidance=avoidance)
    m = compute_metrics(res)
    m["avoidance"] = avoidance
    m["intruder"] = sc.intruders[0].pattern
    m["intruder_speed"] = round(sc.intruders[0].speed, 2)
    return m


def run_benchmark(
    cfg: Config,
    base: Scenario,
    runs: int,
    out: Path,
    jobs: int = 2,
    seed0: int = 1000,
    avoidance: bool = True,
    intruders: int = 1,
) -> dict[str, Any]:
    out.mkdir(parents=True, exist_ok=True)
    scenarios = [make_scenario(base, i, seed0, intruders) for i in range(runs)]
    with ProcessPoolExecutor(max_workers=jobs) as pool:
        results = list(pool.map(_one, [(cfg, s, avoidance) for s in scenarios]))
    cols = [
        "scenario", "intruder", "intruder_speed", "completed", "landed", "duration_s", "energy_wh",
        "est_energy_wh", "min_separation_m", "time_below_min_separation_s", "time_to_first_track_s",
        "tracking_time_s", "track_error_median_m", "track_error_p90_m", "target_in_fov_fraction",
        "yields", "returned_for_traffic", "encounter", "encounter_detected", "lowc_events",
        "false_alarms", "flight_hours", "bird_tracks", "birds_declared_drone", "drones_matched",
        "drones_declared", "min_bird_separation_m",
        "range_mean_m", "false_tracks", "final_distance_from_home_m",
    ]  # fmt: skip
    with open(out / "results.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(results)
    summary = summarise(results)
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def _vals(rs: list[dict[str, Any]], key: str) -> np.ndarray:
    return np.array([r[key] for r in rs if r.get(key) is not None], dtype=float)


def summarise(rs: list[dict[str, Any]]) -> dict[str, Any]:
    tracked = [r for r in rs if r.get("tracking_time_s", 0) > 0]
    sep = _vals(rs, "min_separation_m")
    err = _vals(tracked, "track_error_median_m")
    fov = _vals(tracked, "target_in_fov_fraction")
    energy_ratio = np.array(
        [r["energy_wh"] / r["est_energy_wh"] for r in rs if r.get("energy_wh") and r.get("est_energy_wh")]
    )
    return {
        "seed0": int(str(rs[0]["seed"])) if rs else None,
        "avoidance": bool(rs[0].get("avoidance", True)) if rs else True,
        "runs": len(rs),
        "completed_and_landed": sum(1 for r in rs if r.get("completed") and r.get("landed")),
        "returned_early_for_traffic": sum(1 for r in rs if r.get("returned_for_traffic")),
        "landed_safely": sum(1 for r in rs if r.get("landed")),
        "runs_with_contact_tracked": len(tracked),
        "separation_violations": int(sum(1 for r in rs if r.get("time_below_min_separation_s", 0) > 0)),
        "lowc_events": int(sum(r.get("lowc_events", 0) for r in rs)),
        "encounters": sum(1 for r in rs if r.get("encounter")),
        "encounters_detected": sum(1 for r in rs if r.get("encounter_detected")),
        "false_alarms_per_flight_hour": round(
            sum(r.get("false_alarms", 0) for r in rs) / max(sum(r.get("flight_hours", 0) for r in rs), 1e-9),
            2,
        ),
        "bird_tracks": int(sum(r.get("bird_tracks", 0) for r in rs)),
        "birds_reported_as_drone": int(sum(r.get("birds_declared_drone", 0) for r in rs)),
        "drone_tracks": int(sum(r.get("drones_matched", 0) for r in rs)),
        "drone_tracks_reported_as_drone": int(sum(r.get("drones_declared", 0) for r in rs)),
        "min_bird_separation_m": float(_vals(rs, "min_bird_separation_m").min())
        if len(_vals(rs, "min_bird_separation_m"))
        else None,
        "min_separation_m": {"min": float(sep.min()), "median": float(np.median(sep))} if len(sep) else None,
        "time_to_first_track_s_median": float(np.median(_vals(rs, "time_to_first_track_s")))
        if len(_vals(rs, "time_to_first_track_s"))
        else None,
        "track_error_median_m": {"median": float(np.median(err)), "p90": float(np.percentile(err, 90))}
        if len(err)
        else None,
        "target_in_fov_fraction_median": float(np.median(fov)) if len(fov) else None,
        "false_tracks_per_run_median": float(np.median(_vals(rs, "false_tracks"))),
        "energy_actual_over_estimate_median": float(np.median(energy_ratio)) if len(energy_ratio) else None,
        "final_distance_from_home_m_max": float(_vals(rs, "final_distance_from_home_m").max()),
    }


def risk_ratio(mitigated: dict[str, Any], unmitigated: dict[str, Any]) -> float | None:
    """Detect-and-avoid risk ratio: runs losing well clear with avoidance / without it.

    Both summaries must cover the same encounters (same ``seed0`` and run count).
    """
    if mitigated.get("seed0") != unmitigated.get("seed0") or mitigated["runs"] != unmitigated["runs"]:
        raise ValueError("risk ratio needs the same encounter set with and without avoidance")
    base = unmitigated["separation_violations"]
    return None if base == 0 else round(mitigated["separation_violations"] / base, 3)
