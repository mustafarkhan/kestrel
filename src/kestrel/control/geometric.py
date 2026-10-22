"""Cascaded position/velocity control with a geometric attitude controller on SE(3).

T. Lee, M. Leok, N. H. McClamroch, "Geometric tracking control of a quadrotor UAV
on SE(3)", CDC 2010. The attitude error is defined on SO(3) directly, so there
are no Euler-angle singularities and large-angle manoeuvres stay well behaved.

Additions over the textbook law, all needed on a real airframe:

* integral action on position (wind, mass error) with clamped anti-windup;
* a tilt limit applied to the desired force so aggressive references cannot
  command the aircraft past ``max_tilt_deg``;
* a total-thrust ceiling below the mixer limit to keep attitude authority;
* a slew-limited yaw reference. Near a 180 degree heading error the SO(3) error
  is almost degenerate and a raw heading step couples into roll and pitch
  (simulation showed a 180 degree turn in hover producing 47 degrees of tilt and
  a limit cycle). Autopilots rate-limit yaw for the same reason.

In the simulator this plays the role of the autopilot's position controller.
On hardware, ArduPilot/PX4 run their own and Kestrel only sends setpoints.
"""

from __future__ import annotations

import math

import numpy as np

from kestrel.config import ControlGains, VehicleParams
from kestrel.geometry import E3, Vec, hat, quat_to_rot, vee, wrap_pi, yaw_of
from kestrel.sim.quadrotor import GRAVITY, RigidBodyState
from kestrel.types import Setpoint


class GeometricController:
    def __init__(self, params: VehicleParams, gains: ControlGains) -> None:
        self.mass = params.mass
        self.inertia = np.array(params.inertia)
        self.kp = np.array(gains.kp)
        self.kv = np.array(gains.kv)
        self.ki = np.array(gains.ki)
        self.k_rot = np.array(gains.k_rot)
        self.k_omega = np.array(gains.k_omega)
        self.i_limit = gains.integral_limit
        self.tan_tilt = math.tan(math.radians(gains.max_tilt_deg))
        self.max_total = 0.92 * 4 * params.max_thrust_per_motor
        self.v_max_xy = gains.max_speed_xy
        self.v_max_z = gains.max_speed_z
        self.integral = np.zeros(3)
        self.max_yaw_rate = math.radians(gains.max_yaw_rate_deg_s)
        self.yaw_ref: float | None = None

    def reset(self) -> None:
        self.integral[:] = 0.0
        self.yaw_ref = None

    def _slew_yaw(self, s: RigidBodyState, target: float, dt: float) -> float:
        if self.yaw_ref is None:
            self.yaw_ref = yaw_of(s.quat)
        err = wrap_pi(target - self.yaw_ref)
        step = self.max_yaw_rate * dt
        self.yaw_ref = wrap_pi(self.yaw_ref + max(-step, min(step, err)))
        return self.yaw_ref

    def compute(self, s: RigidBodyState, sp: Setpoint, dt: float) -> tuple[float, Vec]:
        r = quat_to_rot(s.quat)
        ep = s.pos - sp.pos
        self.integral = np.clip(self.integral + ep * dt, -self.i_limit, self.i_limit)

        # Outer loop: position error -> velocity command, saturated like a real autopilot,
        # so a far-away setpoint can never drive the vehicle past its speed envelope.
        v_cmd = sp.vel - self.kp * ep
        vh = math.hypot(v_cmd[0], v_cmd[1])
        if vh > self.v_max_xy:
            v_cmd[0:2] *= self.v_max_xy / vh
        v_cmd[2] = float(np.clip(v_cmd[2], -self.v_max_z, self.v_max_z))
        ev = s.vel - v_cmd
        f_des = (-self.kv * ev - self.ki * self.integral) * self.mass + self.mass * (GRAVITY * E3 + sp.acc)
        f_des[2] = max(f_des[2], 0.2 * self.mass * GRAVITY)
        horiz = math.hypot(f_des[0], f_des[1])
        max_h = self.tan_tilt * f_des[2]
        if horiz > max_h:
            f_des[0:2] *= max_h / horiz

        b3d = f_des / np.linalg.norm(f_des)
        yaw_d = self._slew_yaw(s, sp.yaw, dt)
        b1c = np.array([math.cos(yaw_d), math.sin(yaw_d), 0.0])
        b2d = np.cross(b3d, b1c)
        n = np.linalg.norm(b2d)
        b2d = b2d / n if n > 1e-6 else r[:, 1]
        b1d = np.cross(b2d, b3d)
        rd = np.column_stack([b1d, b2d, b3d])

        thrust = float(np.clip(f_des @ (r @ E3), 0.0, self.max_total))
        e_rot = 0.5 * vee(rd.T @ r - r.T @ rd)
        yaw_rate = float(np.clip(sp.yaw_rate, -self.max_yaw_rate, self.max_yaw_rate))
        omega_d = r.T @ rd @ np.array([0.0, 0.0, yaw_rate])
        e_omega = s.omega - omega_d
        torque = (
            -self.k_rot * e_rot
            - self.k_omega * e_omega
            + np.cross(s.omega, self.inertia * s.omega)
            - self.inertia * (hat(s.omega) @ omega_d)
        )
        return thrust, torque
