"""Rigid-body quadrotor model.

State ``x = [p(3), v(3), q(4), omega(3), f(4)]`` with ``f`` the actual rotor thrusts,
which follow the commanded thrusts through a first-order motor lag. Forces:
rotor thrust along body z, linear air-relative drag in body axes, gravity.
Integrated with classical RK4; the quaternion is renormalised after each step.

Electrical power is estimated from momentum theory (ideal induced power divided
by a propulsive efficiency), which is what drains the simulated battery.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from kestrel.config import VehicleParams
from kestrel.control.allocation import Mixer
from kestrel.geometry import E3, Vec, quat_from_euler, quat_to_rot, yaw_of

GRAVITY = 9.80665
AIR_DENSITY = 1.225


@dataclass
class RigidBodyState:
    pos: Vec
    vel: Vec
    quat: Vec
    omega: Vec
    thrusts: Vec

    def pack(self) -> Vec:
        return np.concatenate([self.pos, self.vel, self.quat, self.omega, self.thrusts])

    @staticmethod
    def unpack(x: Vec) -> RigidBodyState:
        return RigidBodyState(x[0:3].copy(), x[3:6].copy(), x[6:10].copy(), x[10:13].copy(), x[13:17].copy())


class Quadrotor:
    def __init__(self, params: VehicleParams, pos: Vec | None = None, yaw: float = 0.0) -> None:
        self.p = params
        self.mixer = Mixer(params)
        self.mass = params.mass
        self.inertia = np.array(params.inertia)
        self.inertia_inv = 1.0 / self.inertia
        self.drag = np.array(params.linear_drag)
        self.on_ground = True
        disk_area = math.pi * params.rotor_radius**2
        self._power_k = 1.0 / (math.sqrt(2.0 * AIR_DENSITY * disk_area) * params.propulsive_efficiency)
        start = np.zeros(3) if pos is None else np.array(pos, dtype=float)
        self.state = RigidBodyState(
            pos=start,
            vel=np.zeros(3),
            quat=quat_from_euler(0.0, 0.0, yaw),
            omega=np.zeros(3),
            thrusts=np.zeros(4),
        )

    @property
    def hover_thrust(self) -> float:
        return self.mass * GRAVITY

    def electrical_power(self) -> float:
        f = np.clip(self.state.thrusts, 0.0, None)
        return float(self._power_k * np.sum(f**1.5)) + 8.0  # + avionics/companion load

    def _deriv(self, x: Vec, cmd: Vec, wind: Vec) -> Vec:
        q = x[6:10]
        w = x[10:13]
        f = x[13:17]
        r = quat_to_rot(q)
        wrench = self.mixer.wrench(f)
        v_air_body = r.T @ (x[3:6] - wind)
        drag_world = r @ (-self.drag * v_air_body)
        acc = (r @ E3) * wrench[0] / self.mass + drag_world / self.mass - GRAVITY * E3
        jw = self.inertia * w
        wdot = self.inertia_inv * (wrench[1:4] - np.cross(w, jw))
        qw, qx, qy, qz = q
        ox, oy, oz = w
        qdot = 0.5 * np.array(
            [
                -qx * ox - qy * oy - qz * oz,
                qw * ox + qy * oz - qz * oy,
                qw * oy - qx * oz + qz * ox,
                qw * oz + qx * oy - qy * ox,
            ]
        )
        fdot = (cmd - f) / self.p.motor_time_constant
        return np.concatenate([x[3:6], acc, qdot, wdot, fdot])

    def step(self, dt: float, thrust_cmd: Vec, wind: Vec) -> None:
        cmd = np.clip(thrust_cmd, 0.0, self.p.max_thrust_per_motor)
        x = self.state.pack()
        k1 = self._deriv(x, cmd, wind)
        k2 = self._deriv(x + 0.5 * dt * k1, cmd, wind)
        k3 = self._deriv(x + 0.5 * dt * k2, cmd, wind)
        k4 = self._deriv(x + dt * k3, cmd, wind)
        x = x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        x[6:10] /= np.linalg.norm(x[6:10])
        self.state = RigidBodyState.unpack(x)
        self._ground_contact()

    def _ground_contact(self) -> None:
        s = self.state
        total = float(np.sum(s.thrusts))
        if s.pos[2] <= 0.0:
            s.pos[2] = 0.0
            if s.vel[2] < 0.0:
                s.vel[2] = 0.0
            if total < 0.98 * self.hover_thrust:
                # Resting on the ground: friction holds it, attitude settles level.
                s.vel[:] = 0.0
                s.omega[:] = 0.0
                s.quat = quat_from_euler(0.0, 0.0, yaw_of(s.quat))
            self.on_ground = True
        else:
            self.on_ground = s.pos[2] < 0.05 and total < 0.98 * self.hover_thrust
