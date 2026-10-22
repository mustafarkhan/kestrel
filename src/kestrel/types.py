"""Small value types shared across layers."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import StrEnum

import numpy as np

from kestrel.geometry import Vec, yaw_of


def _zeros() -> Vec:
    return np.zeros(3)


@dataclass(frozen=True)
class Setpoint:
    """A full-state reference for the flight controller (world ENU).

    This is the contract between autonomy and the autopilot: it maps 1:1 onto
    MAVLink ``SET_POSITION_TARGET_LOCAL_NED`` (position + velocity + acceleration
    feed-forward + yaw + yaw rate), which ArduPilot GUIDED and PX4 OFFBOARD accept.
    """

    pos: Vec
    vel: Vec = field(default_factory=_zeros)
    acc: Vec = field(default_factory=_zeros)
    yaw: float = 0.0
    yaw_rate: float = 0.0

    def with_(self, **kw: object) -> Setpoint:
        return replace(self, **kw)  # type: ignore[arg-type]

    @staticmethod
    def hold(pos: Vec, yaw: float) -> Setpoint:
        return Setpoint(pos=np.array(pos, dtype=float), yaw=yaw)


class FlightMode(StrEnum):
    DISARMED = "DISARMED"
    GUIDED = "GUIDED"  # following external setpoints
    LAND = "LAND"  # autopilot-owned descent and disarm


@dataclass(frozen=True)
class VehicleState:
    """Navigation solution as reported by the autopilot (may be noisy)."""

    t: float
    pos: Vec
    vel: Vec
    quat: Vec
    omega: Vec
    armed: bool
    on_ground: bool
    mode: FlightMode
    battery_wh: float
    battery_capacity_wh: float

    @property
    def yaw(self) -> float:
        return yaw_of(self.quat)

    @property
    def battery_fraction(self) -> float:
        return self.battery_wh / self.battery_capacity_wh
