"""Control allocation (mixer) for an X-configuration quadrotor.

Maps a wrench ``[T, tau_x, tau_y, tau_z]`` to four rotor thrusts. When the
request is not achievable, priority is roll/pitch, then collective thrust, then
yaw (see :meth:`Mixer.mix`).
"""

from __future__ import annotations

import math

import numpy as np

from kestrel.config import VehicleParams
from kestrel.geometry import Vec

# Rotor azimuths (FLU body, from +x towards +y) and spin-induced yaw-torque sign.
_AZIMUTHS_DEG = (45.0, 135.0, 225.0, 315.0)
_YAW_SIGN = (-1.0, 1.0, -1.0, 1.0)


class Mixer:
    def __init__(self, params: VehicleParams) -> None:
        self.max_thrust = params.max_thrust_per_motor
        cols = []
        for az, s in zip(_AZIMUTHS_DEG, _YAW_SIGN, strict=True):
            x = params.arm_length * math.cos(math.radians(az))
            y = params.arm_length * math.sin(math.radians(az))
            # thrust f along +z at (x, y): torque = r x f = (y f, -x f, 0)
            cols.append([1.0, y, -x, s * params.yaw_moment_coeff])
        self.allocation: Vec = np.array(cols).T
        self.inverse: Vec = np.linalg.inv(self.allocation)

    def wrench(self, thrusts: Vec) -> Vec:
        return np.asarray(self.allocation @ thrusts)

    def mix(self, total_thrust: float, torque: Vec) -> Vec:
        """Allocate with priority roll/pitch > collective thrust > yaw.

        1. Roll/pitch torque is kept whole if the rotor range allows it, otherwise
           scaled down (it is what keeps the aircraft upright).
        2. Collective thrust is then shifted as little as needed to fit the range.
        3. Yaw gets whatever headroom is left.
        """
        att = self.inverse @ np.array([0.0, torque[0], torque[1], 0.0])
        span = float(att.max() - att.min())
        if span > self.max_thrust:
            att *= self.max_thrust / span
        base = self.inverse @ np.array([total_thrust, 0.0, 0.0, 0.0]) + att
        lo, hi = float(base.min()), float(base.max())
        if lo < 0.0:
            base -= lo
        elif hi > self.max_thrust:
            base -= hi - self.max_thrust
        yaw = self.inverse @ np.array([0.0, 0.0, 0.0, torque[2]])
        alpha = 1.0
        for b, y in zip(base, yaw, strict=True):
            if y > 1e-12:  # b + a*y <= max_thrust
                alpha = min(alpha, max(0.0, (self.max_thrust - b) / y))
            elif y < -1e-12:  # b + a*y >= 0
                alpha = min(alpha, max(0.0, b / -y))
        return np.asarray(np.clip(base + alpha * yaw, 0.0, self.max_thrust))
