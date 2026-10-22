"""Battery energy model used for planning and for the runtime return-home reserve.

Hover power from momentum theory, plus a parasitic term from the same linear
drag coefficients the simulator uses, plus a margin factor for wind and
manoeuvring. Deliberately conservative: the cost of over-estimating is an early
return, the cost of under-estimating is a forced landing somewhere else.
"""

from __future__ import annotations

import math

import numpy as np

from kestrel.config import SafetyLimits, VehicleParams
from kestrel.geometry import Vec
from kestrel.sim.quadrotor import AIR_DENSITY, GRAVITY

MARGIN = 1.2
AVIONICS_W = 8.0


class EnergyModel:
    def __init__(self, vehicle: VehicleParams, limits: SafetyLimits) -> None:
        self.v = vehicle
        self.limits = limits
        disk = math.pi * vehicle.rotor_radius**2
        per_motor = vehicle.mass * GRAVITY / 4
        self.p_hover = (
            4 * per_motor**1.5 / (math.sqrt(2 * AIR_DENSITY * disk) * vehicle.propulsive_efficiency)
        )
        self.p_hover += AVIONICS_W
        self.c_drag = vehicle.linear_drag[0] / vehicle.propulsive_efficiency

    def power(self, speed: float) -> float:
        return float(MARGIN * (self.p_hover + self.c_drag * speed * speed))

    def flight_wh(self, distance_m: float, speed: float) -> float:
        speed = max(speed, 0.5)
        return self.power(speed) * (distance_m / speed) / 3600.0

    def hover_wh(self, seconds: float) -> float:
        return self.power(0.0) * seconds / 3600.0

    def to_home_wh(self, pos: Vec, home: Vec) -> float:
        """Energy to fly home at RTL speed and land, with a 20 s hover allowance."""
        d = float(np.linalg.norm(np.asarray(pos[:2]) - np.asarray(home[:2])))
        descent = max(float(pos[2] - home[2]), 0.0) / self.limits.land_speed
        return self.flight_wh(d, self.limits.rtl_speed) + self.hover_wh(descent + 20.0)

    def reserve_wh(self) -> float:
        return self.v.battery_wh * self.limits.energy_reserve_fraction
