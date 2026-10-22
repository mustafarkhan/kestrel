"""Run metrics, computed against ground truth the autonomy never sees."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from kestrel.geofence import Geofence
from kestrel.runtime import RunResult


def _nan(x: float) -> float | None:
    return None if x is None or (isinstance(x, float) and math.isnan(x)) else round(float(x), 3)


def compute_metrics(r: RunResult) -> dict[str, Any]:
    m: dict[str, Any] = {
        "scenario": r.scenario.name,
        "seed": r.scenario.seed,
        "plan_source": r.plan.source,
        "accepted": bool(r.report and r.report.ok),
        "completed": r.completed,
        "landed": r.landed,
    }
    if r.report is not None:
        m["est_duration_s"] = round(r.report.est_duration_s, 1)
        m["est_energy_wh"] = round(r.report.est_energy_wh, 2)
    if not r.flown:
        return m
    a = r.log.arrays()
    t, pos, vel = a["t"], a["pos"], a["vel"]
    lim = r.config.safety
    home = np.array(r.scenario.world.home)
    fence = Geofence(r.scenario.world.fence)
    beh = np.array(r.log.behaviour)
    m.update(
        duration_s=round(float(t[-1]), 1),
        energy_wh=round(r.energy_used_wh, 2),
        max_altitude_m=round(float(pos[:, 2].max()), 2),
        max_horizontal_speed=round(float(np.linalg.norm(vel[:, 0:2], axis=1).max()), 2),
        min_fence_margin_m=round(min(fence.signed_distance(p) for p in pos[::10]), 1),
        final_distance_from_home_m=round(float(np.linalg.norm(pos[-1, 0:2] - home[0:2])), 2),
        false_tracks=r.false_tracks,
        safety_interventions=dict(r.safety_counts),
        safety_latches=list(r.safety_reasons),
    )
    tgt = a["target"]
    present = ~np.isnan(tgt[:, 0])
    if present.any():
        sep = np.linalg.norm(pos - tgt, axis=1)
        dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.05
        m["min_separation_m"] = round(float(np.nanmin(sep)), 2)
        m["time_below_min_separation_s"] = round(float(np.sum(sep[present] < lim.min_separation) * dt), 2)
        trk = a["track"]
        good = present & ~np.isnan(trk[:, 0]) & (np.linalg.norm(trk - tgt, axis=1) < 15.0)
        first = np.argmax(good) if good.any() else None
        m["time_to_first_track_s"] = (
            _nan(float(t[first] - t[np.argmax(present)])) if first is not None else None
        )
        tracking = beh == "track"
        m["tracking_time_s"] = round(float(tracking.sum() * dt), 1)
        if tracking.any():
            have = tracking & ~np.isnan(trk[:, 0])
            err = np.linalg.norm(trk[have] - tgt[have], axis=1)
            rng = sep[tracking]
            m.update(
                track_error_median_m=_nan(float(np.median(err))) if len(err) else None,
                track_error_p90_m=_nan(float(np.percentile(err, 90))) if len(err) else None,
                track_continuity=round(float(have.sum() / tracking.sum()), 3),
                target_in_fov_fraction=round(float(a["target_in_fov"][tracking].mean()), 3),
                range_mean_m=round(float(rng.mean()), 1),
                range_min_m=round(float(rng.min()), 1),
            )
    m["yields"] = sum(1 for e in r.events if e.kind == "yield")
    m["returned_for_traffic"] = any(e.kind == "abort" and "traffic" in e.detail for e in r.events)
    m["events"] = [f"{e.t:.1f}s {e.kind}: {e.detail}" for e in r.events if e.kind != "contact"]
    return m
