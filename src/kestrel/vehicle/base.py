"""The vehicle interface the autonomy stack talks to.

Kept to exactly what a companion computer can ask of an ArduPilot (GUIDED) or
PX4 (OFFBOARD) autopilot, so the simulated and real vehicles are interchangeable.
"""

from __future__ import annotations

from typing import Protocol

from kestrel.types import Setpoint, VehicleState


class Vehicle(Protocol):
    def state(self) -> VehicleState:
        """Latest navigation solution (estimate, not ground truth)."""
        ...

    def arm(self) -> bool: ...

    def takeoff(self, altitude: float) -> None:
        """Autopilot-owned climb to ``altitude`` metres above home."""
        ...

    def set_setpoint(self, sp: Setpoint) -> None: ...

    def land(self) -> None:
        """Autopilot-owned descent, touchdown and disarm."""
        ...

    def disarm(self) -> bool: ...
