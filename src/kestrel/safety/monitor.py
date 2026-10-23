"""Runtime safety monitor (run-time assurance / simplex pattern).

Every setpoint the mission executive produces passes through
:meth:`SafetyMonitor.filter` before it reaches the autopilot. The executive may
be wrong (an LLM wrote its plan, a tracker fed its guidance); the monitor is
small, has no learned components, and enforces invariants regardless:

=====================  ====================================================
Invariant              Enforcement
=====================  ====================================================
Geofence               setpoints projected ``fence_margin`` inside; braking
                       when the predicted stopping point would cross; RTL on
                       actual breach
Altitude band          setpoints clamped to ``[min_altitude, max_altitude]``
                       (floor relaxed only during takeoff/landing)
Kinematic limits       velocity and acceleration feed-forward clipped
Separation             setpoints pushed out of ``min_separation + k*sigma``
                       around every confirmed track, sigma floored at a
                       fraction of range, plus stopping distance at the
                       current closing speed; unobserved tracks grow a
                       reachable-set keep-out; inside the buffer an
                       escape velocity is commanded
Energy reserve         latched RTL when energy left minus energy to get home
                       drops below the reserve; latched LAND at half reserve
Link loss              latched RTL after ``link_loss_timeout_s`` without a
                       ground-station heartbeat
=====================  ====================================================

Requests (RTL / LAND) are *latched*: once raised they are not withdrawn by later
good news, which is the predictable behaviour an operator needs.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass, field
from enum import StrEnum

import numpy as np

from kestrel.config import SafetyLimits, VehicleParams, WorldConfig
from kestrel.geofence import Geofence
from kestrel.perception.tracker import TrackEstimate
from kestrel.safety.energy import EnergyModel
from kestrel.types import Setpoint, VehicleState


class SafetyRequest(StrEnum):
    NONE = "none"
    RTL = "rtl"
    LAND = "land"


@dataclass
class SafetyDecision:
    setpoint: Setpoint | None
    interventions: list[str] = field(default_factory=list)
    request: SafetyRequest = SafetyRequest.NONE


class SafetyMonitor:
    def __init__(
        self, limits: SafetyLimits, vehicle: VehicleParams, world: WorldConfig, separation: bool = True
    ) -> None:
        self.lim = limits
        self.separation = separation  # False only for the unmitigated benchmark baseline
        self.fence = Geofence(world.fence)
        self.home = np.array(world.home, dtype=float)
        self.energy = EnergyModel(vehicle, limits)
        self.latched = SafetyRequest.NONE
        self.reasons: list[str] = []
        self.counts: Counter[str] = Counter()
        self.last_link_ok_t = 0.0

    # ------------------------------------------------------------------ requests
    def _latch(self, req: SafetyRequest, reason: str) -> None:
        order = [SafetyRequest.NONE, SafetyRequest.RTL, SafetyRequest.LAND]
        if order.index(req) > order.index(self.latched):
            self.latched = req
            self.reasons.append(reason)
            self.counts[f"latch_{req.value}"] += 1

    def check_vehicle(self, st: VehicleState, link_ok: bool) -> None:
        if link_ok:
            self.last_link_ok_t = st.t
        if not st.armed or st.on_ground:
            return
        limit = (
            self.lim.link_loss_timeout_s
            if self.lim.link_loss_action == "rtl"
            else self.lim.link_loss_autonomy_s
        )
        if st.t - self.last_link_ok_t > limit:
            self._latch(SafetyRequest.RTL, f"ground link lost for {st.t - self.last_link_ok_t:.0f} s")
        if self.fence.signed_distance(st.pos) < 0:
            self._latch(SafetyRequest.RTL, "geofence breached")
        need = self.energy.to_home_wh(st.pos, self.home)
        left = st.battery_wh - need
        reserve = self.energy.reserve_wh()
        if left < 0.5 * reserve:
            self._latch(SafetyRequest.LAND, f"energy critical ({st.battery_wh:.1f} Wh left)")
        elif left < reserve:
            self._latch(SafetyRequest.RTL, f"energy reserve reached ({st.battery_wh:.1f} Wh left)")

    # ------------------------------------------------------------------ setpoint filter
    def filter(
        self,
        st: VehicleState,
        sp: Setpoint | None,
        tracks: list[TrackEstimate],
        low_altitude_ok: bool = False,
    ) -> SafetyDecision:
        dec = SafetyDecision(setpoint=sp, request=self.latched)
        if sp is None:
            return dec
        lim = self.lim
        pos = np.array(sp.pos, dtype=float)
        vel = np.array(sp.vel, dtype=float)
        acc = np.array(sp.acc, dtype=float)

        def note(tag: str) -> None:
            dec.interventions.append(tag)
            self.counts[tag] += 1

        # separation from every confirmed track (first, so the fence has the final word)
        for tr in tracks if self.separation else []:
            # Monocular range comes from an assumed target size, so it is never better than
            # ``range_uncertainty_floor`` of the range however confident the filter claims to be.
            rng = float(np.linalg.norm(tr.pos - st.pos))
            sigma = max(tr.pos_std / math.sqrt(3), lim.range_uncertainty_floor * rng)
            # An aircraft we cannot currently see may have flown towards us at top speed:
            # keep out of its reachable set, which grows with the time since it was last seen.
            reach = lim.intruder_max_speed * max(st.t - tr.last_hit_t - 0.3, 0.0)
            # Stopping distance at the current closing speed, plus reaction time: a static
            # buffer is useless if both aircraft are closing at 8 m/s.
            los = (st.pos - tr.pos) / max(rng, 1e-6)
            v_close = max(0.0, -float((st.vel - tr.vel) @ los))
            brake = v_close * v_close / (2.0 * lim.max_acceleration) + v_close * lim.reaction_time_s
            need = lim.min_separation + max(lim.separation_margin_sigmas * sigma, reach) + brake
            d = pos - tr.pos
            dist = float(np.linalg.norm(d))
            if dist < need:
                # Push horizontally and keep altitude: a multirotor has about 3x more horizontal
                # than vertical speed authority, so moving along the 3-D line (downwards when
                # the intruder is above) escapes on the weakest axis.
                dz = float(d[2])
                h_need = math.sqrt(max(need * need - dz * dz, 0.0))
                hd = math.hypot(float(d[0]), float(d[1]))
                if hd < h_need:
                    if hd > 0.5:
                        u_h = np.array([d[0], d[1], 0.0]) / hd
                    else:
                        away = st.pos - tr.pos
                        ah = math.hypot(float(away[0]), float(away[1]))
                        u_h = np.array([away[0], away[1], 0.0]) / ah if ah > 0.5 else np.array([1.0, 0, 0])
                    pos = np.array([tr.pos[0], tr.pos[1], pos[2]]) + u_h * h_need
                    note("separation_push")
            rel = st.pos - tr.pos
            rd = float(np.linalg.norm(rel))
            if rd < need and rd > 1e-6:
                # Inside the buffer: a position nudge is not enough against an intruder
                # flying at us. Command an escape velocity along the line of sight of at least
                # the intruder's own speed towards us plus a margin that grows with depth.
                # Escape horizontally where possible: a multirotor has ~3x more horizontal
                # than vertical speed authority.
                h = float(np.hypot(rel[0], rel[1]))
                u = np.array([rel[0] / h, rel[1] / h, 0.0]) if h > 2.0 else rel / rd
                required = max(float(tr.vel @ u), 0.0) + 2.0 + 1.0 * (need - rd)
                outward = float(vel @ u)
                if outward < required:
                    vel = vel + (required - outward) * u
                    note("separation_escape")

        # geofence: setpoint must sit inside the margin
        if self.fence.signed_distance(pos) < lim.fence_margin:
            pos[0:2] = self.fence.project_inside(pos, lim.fence_margin)
            vel[0:2] = 0.0
            acc[0:2] = 0.0
            note("fence_clamp")
        # geofence: will the aircraft stop in time from where it is going now?
        v_h = float(np.linalg.norm(st.vel[0:2]))
        if v_h > 0.5:
            stop = st.pos[0:2] + st.vel[0:2] / v_h * (v_h * v_h / (2 * lim.max_acceleration) + 1.0)
            if self.fence.signed_distance(stop) < 0.5 * lim.fence_margin:
                pos[0:2] = self.fence.project_inside(st.pos, lim.fence_margin)
                vel[0:2] = 0.0
                acc[0:2] = 0.0
                note("fence_brake")

        # altitude band
        floor = 0.0 if low_altitude_ok else lim.min_altitude
        if pos[2] > lim.max_altitude:
            pos[2] = lim.max_altitude
            vel[2] = min(vel[2], 0.0)
            note("ceiling_clamp")
        elif pos[2] < floor:
            pos[2] = floor
            vel[2] = max(vel[2], 0.0)
            note("floor_clamp")

        # kinematic limits
        vh = float(np.linalg.norm(vel[0:2]))
        if vh > lim.max_horizontal_speed:
            vel[0:2] *= lim.max_horizontal_speed / vh
            note("speed_clip")
        if abs(vel[2]) > lim.max_vertical_speed:
            vel[2] = math.copysign(lim.max_vertical_speed, vel[2])
            note("speed_clip")
        an = float(np.linalg.norm(acc))
        if an > lim.max_acceleration:
            acc *= lim.max_acceleration / an
            note("accel_clip")

        dec.setpoint = sp.with_(pos=pos, vel=vel, acc=acc)
        return dec
