"""Run metrics, computed against ground truth the autonomy never sees.

Definitions follow the detect-and-avoid literature, scaled to small-drone encounters:

* **Encounter detected** (after the Amazon AOT benchmark, which requires a track for
  3 s before the intruder is within 300 m of a manned-aircraft-sized encounter): a
  correct track held for ``DETECT_LEAD_S`` before the intruder first comes within
  ``DETECT_RANGE_M``, Kestrel's 30 m standoff.
* **False alarm**: a *confirmed* track that never corresponds to a real aircraft.
  Short-lived tentative tracks are not alarms (they never reach the autonomy).
* **Birds**: a confirmed track on a bird is a false alarm only if it is *reported as a
  drone* (with the classifier off, every track is). Bird tracks are matched by bearing,
  since a bird's range estimated from an assumed drone size is wrong by the size ratio.
* **Cue to track** and **cue to confirm**: from the first ground-sensor report reaching
  the observer to the first track on an intruder with 8 or more detections (not called a
  bird), and to the first such track reported as a drone. The figures a commander cares
  about: how long from "something was heard" to "an aircraft is here", and to "it is a
  drone". Entry-to-track and entry-to-confirm measure the same from the intruder crossing
  the geofence, for configurations without ground sensors.
* **Loss of well clear (LoWC)**: an entry into the ``min_separation`` sphere (12 m).
  ASTM F3442's well-clear volume (610 m x 77 m) is defined for manned traffic; no
  standard exists for drone-to-drone well clear, so Kestrel states its own. Separation
  and LoWC count the nearest intruder at each instant; detection and tracking metrics
  refer to the first intruder.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from kestrel.geofence import Geofence
from kestrel.runtime import RunResult

DETECT_LEAD_S = 3.0
DETECT_RANGE_M = 30.0
TRACK_MATCH_M = 15.0


def _nan(x: float) -> float | None:
    return None if x is None or (isinstance(x, float) and math.isnan(x)) else round(float(x), 3)


def present_any(d: np.ndarray) -> np.ndarray:
    return np.asarray(~np.isnan(d))


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
        false_alarms=r.false_confirmed,
        bird_tracks=r.bird_tracks,
        birds_declared_drone=r.birds_declared_drone,
        drones_matched=r.drones_matched,
        drones_declared=r.drones_declared,
        flight_hours=round(float(t[-1]) / 3600.0, 4),
        safety_interventions=dict(r.safety_counts),
        safety_latches=list(r.safety_reasons),
    )
    bird_d = a["bird_dist"]
    if np.isfinite(bird_d).any():
        m["min_bird_separation_m"] = round(float(np.min(bird_d)), 2)
    tgt = a["target"]
    present = ~np.isnan(tgt[:, 0])
    if present.any():
        sep = np.linalg.norm(pos - tgt, axis=1)  # to the first intruder: detection and tracking metrics
        near = a["intruder_dist"]  # to whichever intruder is nearest: separation metrics
        dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.05
        m["min_separation_m"] = round(float(np.nanmin(near)), 2)
        below = near[present_any(near)] < lim.min_separation
        m["time_below_min_separation_s"] = round(float(np.sum(below) * dt), 2)
        trk = a["track"]
        good = present & ~np.isnan(trk[:, 0]) & (np.linalg.norm(trk - tgt, axis=1) < TRACK_MATCH_M)
        m["lowc_events"] = int(np.sum(np.diff(np.concatenate([[0], below.astype(int)])) == 1))
        inside = np.flatnonzero(present & (sep < DETECT_RANGE_M))
        if len(inside):
            k = int(inside[0])
            window = (t > t[k] - DETECT_LEAD_S) & (t <= t[k])
            m["encounter"] = True
            m["encounter_detected"] = bool(good[window].all()) if window.any() else False
        else:
            m["encounter"] = False
            m["encounter_detected"] = None
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
    if r.first_cue_t is not None:
        m["cue_t_s"] = round(r.first_cue_t, 2)
        if r.first_track_t is not None:
            m["cue_to_track_s"] = round(r.first_track_t - r.first_cue_t, 2)
        if r.first_confirm_t is not None:
            # negative when the camera had it before any ground sensor did
            m["cue_to_confirm_s"] = round(r.first_confirm_t - r.first_cue_t, 2)
    if r.first_track_t is not None and r.first_entry_t is not None:
        m["entry_to_track_s"] = round(r.first_track_t - r.first_entry_t, 2)
    if r.first_confirm_t is not None:
        m["confirm_t_s"] = round(r.first_confirm_t, 2)
        if r.first_entry_t is not None:
            m["entry_to_confirm_s"] = round(r.first_confirm_t - r.first_entry_t, 2)
    if r.scenario.link_loss and len(a["on_intruder"]):
        dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.05
        jammed = np.zeros(len(t), dtype=bool)
        for ev in r.scenario.link_loss:
            jammed |= (t >= ev.start_s) & (t < ev.start_s + ev.duration_s)
        m["jammed_s"] = round(float(jammed.sum() * dt), 1)
        m["observed_while_jammed_s"] = round(float((jammed & a["on_intruder"]).sum() * dt), 1)
    m["observers"] = r.observers
    if r.team_min_separation_m is not None:
        m["team_min_separation_m"] = round(r.team_min_separation_m, 2)
    if r.track_errors:
        e = np.array(r.track_errors)
        m["track_error_all_median_m"] = round(float(np.median(e)), 2)
        m["track_error_all_p90_m"] = round(float(np.percentile(e, 90)), 2)
    m["investigations"] = sum(1 for e in r.events if e.kind == "investigate")
    m["investigations_confirmed"] = sum(1 for e in r.events if e.kind == "confirmed")
    m["reports_buffered"] = r.reports_buffered
    m["yields"] = sum(1 for e in r.events if e.kind == "yield")
    m["returned_for_traffic"] = any(e.kind == "abort" and "traffic" in e.detail for e in r.events)
    m["events"] = [f"{e.t:.1f}s {e.kind}: {e.detail}" for e in r.events if e.kind != "contact"]
    return m
