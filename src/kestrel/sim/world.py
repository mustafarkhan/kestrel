"""The simulated world: observer airframe + emulated autopilot + wind + intruders.

The autopilot emulation is deliberately thin and mirrors what ArduPilot GUIDED /
PX4 OFFBOARD expose to a companion computer: arm, takeoff, follow setpoints,
land-and-disarm. Everything smarter lives in the autonomy stack, so the same
autonomy code drives this simulator or a real vehicle over MAVLink.
"""

from __future__ import annotations

import numpy as np

from kestrel.config import Config, Scenario
from kestrel.control.geometric import GeometricController
from kestrel.geometry import Vec, yaw_of
from kestrel.sim.birds import BirdState, Flock
from kestrel.sim.intruder import Intruder
from kestrel.sim.quadrotor import Quadrotor
from kestrel.sim.wind import Wind
from kestrel.types import FlightMode, Setpoint


class SimWorld:
    def __init__(self, cfg: Config, scenario: Scenario) -> None:
        self.cfg = cfg
        self.scenario = scenario
        self.rng = np.random.default_rng(scenario.seed)
        home = np.array(scenario.world.home, dtype=float)
        self.quad = Quadrotor(cfg.vehicle, pos=home, yaw=0.0)
        self.ctrl = GeometricController(cfg.vehicle, cfg.control)
        self.wind = Wind(scenario.world.wind, self.rng)
        self.intruders = [Intruder(c, i) for i, c in enumerate(scenario.intruders)]
        self.flock = (
            Flock(scenario.birds, scenario.world, float(home[2]), scenario.seed)
            if scenario.birds.count
            else None
        )
        self.t = 0.0
        self.mode = FlightMode.DISARMED
        self.setpoint = Setpoint.hold(home, 0.0)
        self.battery_wh = cfg.vehicle.battery_wh
        self.energy_used_wh = 0.0
        self._land_sp: Setpoint | None = None
        self._climb_to: float | None = None
        self.wind_now = np.zeros(3)

    # ------------------------------------------------------------------ autopilot API
    @property
    def armed(self) -> bool:
        return self.mode is not FlightMode.DISARMED

    def arm(self) -> bool:
        if self.mode is FlightMode.DISARMED and self.quad.on_ground and self.battery_wh > 0:
            p = self.quad.state.pos.copy()
            self.setpoint = Setpoint.hold(p, self._yaw())
            self.ctrl.reset()
            self.mode = FlightMode.GUIDED
            return True
        return False

    def disarm(self) -> bool:
        if self.quad.on_ground:
            self.mode = FlightMode.DISARMED
            return True
        return False

    def takeoff(self, altitude: float) -> None:
        """Autopilot-owned vertical climb, like ArduPilot NAV_TAKEOFF in GUIDED."""
        if self.mode is FlightMode.GUIDED:
            self._climb_to = altitude

    def set_setpoint(self, sp: Setpoint) -> None:
        if self.mode is FlightMode.GUIDED:
            self._climb_to = None
            self.setpoint = sp

    def land(self) -> None:
        if self.armed:
            self.mode = FlightMode.LAND
            p = self.quad.state.pos
            self._land_sp = Setpoint(
                pos=np.array([p[0], p[1], p[2]]),
                vel=np.array([0.0, 0.0, -self.cfg.safety.land_speed]),
                yaw=self._yaw(),
            )

    def _yaw(self) -> float:
        return yaw_of(self.quad.state.quat)

    # ------------------------------------------------------------------ simulation
    def intruder_states(self) -> list[tuple[int, Vec, Vec]]:
        out = []
        for it in self.intruders:
            s = it.state(self.t)
            if s is not None:
                out.append((it.ident, s[0], s[1]))
        return out

    def bird_states(self) -> list[BirdState]:
        return self.flock.states(self.t) if self.flock is not None else []

    def step(self, dt: float) -> None:
        self.wind_now = self.wind.step(dt)
        if self.mode is FlightMode.DISARMED:
            cmd = np.zeros(4)
        else:
            if self._climb_to is not None:
                cur = self.setpoint
                vz = self.cfg.safety.max_vertical_speed * 0.6
                z = min(cur.pos[2] + vz * dt, self._climb_to)
                self.setpoint = cur.with_(
                    pos=np.array([cur.pos[0], cur.pos[1], z]),
                    vel=np.array([0.0, 0.0, vz if z < self._climb_to else 0.0]),
                )
            sp = self.setpoint
            if self.mode is FlightMode.LAND and self._land_sp is not None:
                # descending reference; position and velocity stay consistent
                lp = self._land_sp
                z = max(lp.pos[2] - self.cfg.safety.land_speed * dt, -0.5)
                vz = -self.cfg.safety.land_speed if z > -0.5 else 0.0
                self._land_sp = lp.with_(
                    pos=np.array([lp.pos[0], lp.pos[1], z]), vel=np.array([0.0, 0.0, vz])
                )
                sp = self._land_sp
            thrust, torque = self.ctrl.compute(self.quad.state, sp, dt)
            cmd = self.quad.mixer.mix(thrust, torque)
        self.quad.step(dt, cmd, self.wind_now)
        if self.armed:
            e = self.quad.electrical_power() * dt / 3600.0
            self.battery_wh = max(0.0, self.battery_wh - e)
            self.energy_used_wh += e
        if (
            self.mode is FlightMode.LAND
            and self.quad.state.pos[2] < 0.05
            and abs(self.quad.state.vel[2]) < 0.3
            and self._land_sp is not None
            and self._land_sp.pos[2] < -0.2
        ):
            self.mode = FlightMode.DISARMED
            self._land_sp = None
        self.t += dt
