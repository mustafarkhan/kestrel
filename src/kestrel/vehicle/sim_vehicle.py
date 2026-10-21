"""Vehicle backed by :class:`~kestrel.sim.world.SimWorld`, with navigation noise."""

from __future__ import annotations

import math

import numpy as np

from kestrel.config import StateNoise
from kestrel.geometry import quat_from_euler, quat_mul
from kestrel.sim.world import SimWorld
from kestrel.types import Setpoint, VehicleState


class SimVehicle:
    def __init__(self, world: SimWorld, noise: StateNoise, seed: int = 0) -> None:
        self.world = world
        self.noise = noise
        self.rng = np.random.default_rng(seed + 31)

    def state(self) -> VehicleState:
        w = self.world
        s = w.quad.state
        n = self.noise
        tilt = math.radians(n.tilt_sigma_deg)
        dq = quat_from_euler(
            self.rng.normal(0, tilt),
            self.rng.normal(0, tilt),
            self.rng.normal(0, math.radians(n.yaw_sigma_deg)),
        )
        return VehicleState(
            t=w.t,
            pos=s.pos + self.rng.normal(0.0, n.position_sigma, 3),
            vel=s.vel + self.rng.normal(0.0, n.velocity_sigma, 3),
            quat=quat_mul(s.quat, dq),
            omega=s.omega.copy(),
            armed=w.armed,
            on_ground=w.quad.on_ground,
            mode=w.mode,
            battery_wh=w.battery_wh,
            battery_capacity_wh=w.cfg.vehicle.battery_wh,
        )

    def arm(self) -> bool:
        return self.world.arm()

    def takeoff(self, altitude: float) -> None:
        self.world.takeoff(altitude)

    def set_setpoint(self, sp: Setpoint) -> None:
        self.world.set_setpoint(sp)

    def land(self) -> None:
        self.world.land()

    def disarm(self) -> bool:
        return self.world.disarm()
