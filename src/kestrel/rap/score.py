"""Scoring a recognised air picture against ground truth.

The NATO counter-UAS data challenge judges a fused picture on tracking accuracy,
classification and identification; this module measures the same three, with stated
definitions:

* **GOSPA** (generalised optimal sub-pattern assignment; Rahmathullah, García-Fernández and
  Svensson, 2017) per time step, with cut-off ``c`` = 25 m, ``p`` = 2 and ``alpha`` = 2, so it
  splits into a localisation part for matched tracks and a cost of ``c**2 / 2`` for each
  missed object and each false track. Distances are horizontal, because acoustic and RF
  fixes carry no altitude and would otherwise be charged for an error they declare.
* **Coverage**: share of drone-seconds inside the area with a matching picture track.
* **Duplicates**: extra picture tracks on one object, per object-second.
* **Track continuity**: picture track identities used per drone (1 is perfect).
* **Classification**: share of matched drone-seconds labelled UAV, and labelled Bird (the
  dangerous error); share of bird-seconds labelled UAV (the false alarm), of those matched
  to a picture track and of all bird-seconds inside the area (``bird_time_as_uav``: what an
  operator sees, since a sensor that never reports birds raises no bird alarms).
* **Identification**: share of matched friendly-drone seconds shown as FRIEND; seconds a
  hostile drone was shown as friend or assumed friend, and seconds a friend was shown
  where no friendly drone is (a spoofed or phantom friend); both should be zero.
"""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import linear_sum_assignment

from kestrel.geofence import Geofence
from kestrel.rap.fusion import FusionNode, RapTrack
from kestrel.rap.truth import TruthObject

CUTOFF_M = 25.0


@dataclass
class PictureScore:
    steps: int = 0
    gospa: list[float] = field(default_factory=list)
    missed: list[int] = field(default_factory=list)
    false: list[int] = field(default_factory=list)
    drone_s: float = 0.0
    drone_covered_s: float = 0.0
    hostile_s: float = 0.0
    hostile_covered_s: float = 0.0
    dup_s: float = 0.0
    object_s: float = 0.0
    ids: dict[str, set[int]] = field(default_factory=lambda: defaultdict(set))
    drone_uav_s: float = 0.0
    drone_bird_s: float = 0.0
    drone_matched_s: float = 0.0
    bird_matched_s: float = 0.0
    bird_s: float = 0.0  # every bird-second inside the area, tracked or not
    bird_uav_s: float = 0.0
    friend_s: float = 0.0
    friend_ok_s: float = 0.0
    hostile_as_friend_s: float = 0.0
    phantom_friend_s: float = 0.0
    entry_t: dict[str, float] = field(default_factory=dict)
    first_track_t: dict[str, float] = field(default_factory=dict)
    first_suspect_t: dict[str, float] = field(default_factory=dict)
    first_uav_t: dict[str, float] = field(default_factory=dict)  # hostile first shown as class UAV
    duration_s: float = 0.0
    hostile_keys: set[str] = field(default_factory=set)

    def add(self, t: float, dt: float, truth: list[TruthObject], picture: list[RapTrack], fusion: FusionNode,
            fence: Geofence) -> None:  # fmt: skip
        self.steps += 1
        self.duration_s += dt
        objs = [o for o in truth if fence.signed_distance(o.pos) > 0.0]
        trks = [tr for tr in picture if fence.signed_distance(tr.pos) > 0.0]
        n, m = len(objs), len(trks)
        assigned: dict[int, int] = {}
        loc = 0.0
        if n and m:
            d = np.array([[float(np.linalg.norm(o.pos[0:2] - tr.pos[0:2])) for tr in trks] for o in objs])
            cost = np.minimum(d, CUTOFF_M) ** 2
            rows, cols = linear_sum_assignment(cost)
            for i, j in zip(rows, cols, strict=True):
                if d[i, j] < CUTOFF_M:
                    assigned[i] = j
                    loc += d[i, j] ** 2
        miss, false = n - len(assigned), m - len(assigned)
        self.gospa.append(math.sqrt(loc + CUTOFF_M**2 / 2.0 * (miss + false)))
        self.missed.append(miss)
        self.false.append(false)
        friends = [o for o in objs if o.role in ("friendly", "own")]
        for tr in trks:
            if tr.identity in ("FRIEND", "ASSUMED_FRIEND") and not any(
                float(np.linalg.norm(o.pos[0:2] - tr.pos[0:2])) < CUTOFF_M for o in friends
            ):
                self.phantom_friend_s += dt
        # duplicates: unassigned tracks that sit on an object anyway
        used = set(assigned.values())
        for j, tr in enumerate(trks):
            if j in used:
                continue
            if any(float(np.linalg.norm(o.pos[0:2] - tr.pos[0:2])) < CUTOFF_M for o in objs):
                self.dup_s += dt
        self.object_s += dt * n
        for i, o in enumerate(objs):
            if o.role == "own":
                continue  # our own airborne node: matched so it is no false track, scored nowhere
            if o.kind == "uav":
                self.entry_t.setdefault(o.key, t)
                if o.role == "hostile":
                    self.hostile_keys.add(o.key)
                self.drone_s += dt
                if o.role == "hostile":
                    self.hostile_s += dt
            else:
                self.bird_s += dt
            j = assigned.get(i)
            if j is None:
                continue
            tr = trks[j]
            cls = fusion.class_of(tr)
            if o.kind == "uav":
                self.ids[o.key].add(tr.ident)
                self.first_track_t.setdefault(o.key, t)
                self.drone_covered_s += dt
                self.drone_matched_s += dt
                self.drone_uav_s += dt * (cls == "UAV")
                self.drone_bird_s += dt * (cls == "Bird")
                if o.role == "hostile":
                    self.hostile_covered_s += dt
                    if cls == "UAV":
                        self.first_uav_t.setdefault(o.key, t)
                    if tr.identity in ("FRIEND", "ASSUMED_FRIEND"):
                        self.hostile_as_friend_s += dt
                    if tr.identity == "SUSPECT":
                        self.first_suspect_t.setdefault(o.key, t)
                if o.role == "friendly":
                    self.friend_s += dt
                    self.friend_ok_s += dt * (tr.identity == "FRIEND")
            else:
                self.bird_matched_s += dt
                self.bird_uav_s += dt * (cls == "UAV")

    def summary(self) -> dict[str, float | None]:
        def ratio(a: float, b: float) -> float | None:
            return round(a / b, 3) if b > 0 else None

        delays = [self.first_track_t[k] - self.entry_t[k] for k in self.first_track_t]
        hostiles = [k for k in self.first_track_t if k in self.hostile_keys]
        classify = [self.first_uav_t[k] - self.first_track_t[k] for k in hostiles if k in self.first_uav_t]
        return {
            "gospa_m": round(float(np.mean(self.gospa)), 2) if self.gospa else None,
            "missed_mean": round(float(np.mean(self.missed)), 2) if self.missed else None,
            "false_mean": round(float(np.mean(self.false)), 2) if self.false else None,
            "drone_coverage": ratio(self.drone_covered_s, self.drone_s),
            "hostile_coverage": ratio(self.hostile_covered_s, self.hostile_s),
            "duplicate_ratio": ratio(self.dup_s, self.object_s),
            "ids_per_drone": round(float(np.mean([len(v) for v in self.ids.values()])), 2)
            if self.ids
            else None,
            "drone_as_uav": ratio(self.drone_uav_s, self.drone_matched_s),
            "drone_as_bird": ratio(self.drone_bird_s, self.drone_matched_s),
            "bird_as_uav": ratio(self.bird_uav_s, self.bird_matched_s),
            "bird_time_as_uav": ratio(self.bird_uav_s, self.bird_s),
            "friend_identified": ratio(self.friend_ok_s, self.friend_s),
            "hostile_as_friend_s": round(self.hostile_as_friend_s, 1),
            "phantom_friend_s": round(self.phantom_friend_s, 1),
            "first_track_s": round(float(np.median(delays)), 1) if delays else None,
            "drones_never_tracked": float(len(self.entry_t) - len(self.first_track_t)),
            "hostiles_tracked": float(len(hostiles)),
            "hostiles_classified": float(len(classify)),
            "track_to_uav_s": round(float(np.median(classify)), 1) if classify else None,
        }
