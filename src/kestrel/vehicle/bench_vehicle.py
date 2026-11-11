"""A simulated vehicle that runs behind the *real* flight loop.

``runtime.run`` simulates everything in one process for benchmarks. This is the other use of
the simulator: it stands in for the autopilot and the sky only, so the real loop
(``live.run_live``), with its real camera path, SAPIENT link, recorder and fault handling,
can be exercised on a bench without an aircraft. ``kestrel fly --connect bench`` uses it in
real time; the test suite steps it deterministically, a fixed interval per state read.

It can also misbehave on purpose (go silent, hand control to a "pilot", start somewhere
else on the site), which is how the loop's handling of those is tested.
"""

from __future__ import annotations

import time

import numpy as np

from kestrel.config import Config, Scenario
from kestrel.geometry import Vec
from kestrel.interop.utm import lla_to_enu
from kestrel.perception.camera import Camera
from kestrel.perception.render import SkyRenderer
from kestrel.perception.source import CallableSource
from kestrel.sim.world import SimWorld
from kestrel.types import Setpoint, VehicleState
from kestrel.vehicle.sim_vehicle import SimVehicle


class BenchVehicle:
    def __init__(
        self, cfg: Config, scenario: Scenario, step_dt: float | None = None, speedup: float = 1.0
    ) -> None:
        """``step_dt``: advance this much per ``state()`` call (deterministic); ``None``: follow
        the wall clock times ``speedup``."""
        self.cfg = cfg
        self.scenario = scenario
        self.world = SimWorld(cfg, scenario)
        self._sim = SimVehicle(self.world, cfg.state_noise, seed=scenario.seed)
        self.step_dt = step_dt
        self.speedup = speedup
        self._wall0 = time.monotonic()
        self.offset = np.zeros(3)  # where the simulated take-off point lies on the site
        self.global_lla: tuple[float, float, float] | None = None
        self.silent = False  # stop answering, as a dead telemetry link does
        self.pilot = False  # a safety pilot has switched out of GUIDED
        self.setpoints = 0
        self.land_requests = 0
        self._silent_since = 0.0

    # ------------------------------------------------------------------ time
    def clock(self) -> float:
        """Monotonic time for the loop: simulated time when stepped, wall time otherwise."""
        return self.world.t if self.step_dt is not None else time.monotonic()

    def _advance(self) -> None:
        dt = self.cfg.physics_dt
        if self.step_dt is not None:
            n = max(1, round(self.step_dt / dt))
        else:
            n = int(((time.monotonic() - self._wall0) * self.speedup - self.world.t) / dt)
        for _ in range(max(n, 0)):
            self.world.step(dt)

    def idle(self) -> None:
        """Let simulated time pass while the autopilot is silent (the loop reads no state then)."""
        self._advance()

    # ------------------------------------------------------------------ what the loop probes
    def telemetry_age_s(self) -> float:
        if not self.silent:
            self._silent_since = self.clock()
            return 0.0
        self.idle()
        return self.clock() - self._silent_since

    def pilot_has_control(self) -> bool:
        return self.pilot and self.world.armed

    def mode_name(self) -> str:
        return "LOITER" if self.pilot else "GUIDED"

    def global_position(self) -> tuple[float, float, float] | None:
        return self.global_lla

    def align_to_site(self, origin_lla: tuple[float, float, float]) -> Vec:
        if self.global_lla is None:
            raise RuntimeError("no global position")
        e, n, _ = lla_to_enu(origin_lla, self.global_lla[0], self.global_lla[1], origin_lla[2])
        self.offset = np.array([e, n, 0.0])
        return self.offset.copy()

    # ------------------------------------------------------------------ Vehicle API
    def state(self) -> VehicleState:
        self._advance()
        s = self._sim.state()
        return VehicleState(
            s.t, s.pos + self.offset, s.vel, s.quat, s.omega, s.armed, s.on_ground, s.mode,
            s.battery_wh, s.battery_capacity_wh,
        )  # fmt: skip

    def arm(self) -> bool:
        return self._sim.arm()

    def takeoff(self, altitude: float) -> None:
        self._sim.takeoff(altitude)

    def set_setpoint(self, sp: Setpoint) -> None:
        self.setpoints += 1
        self._sim.set_setpoint(sp.with_(pos=np.asarray(sp.pos) - self.offset))

    def land(self) -> None:
        self.land_requests += 1
        self._sim.land()

    def disarm(self) -> bool:
        return self._sim.disarm()

    # ------------------------------------------------------------------ the sky it sees
    def camera_source(self) -> CallableSource:
        """Frames rendered from the vehicle's true pose: the scenario's intruders and birds
        against the sky, as the benchmarks render them."""
        cam = Camera(self.cfg.camera, self.cfg.vehicle.camera_offset)
        w = self.scenario.world
        ren = SkyRenderer(
            cam, seed=self.scenario.seed, surface=w.surface, band=self.cfg.camera.band, light=w.light
        )
        span = {i: c.span_m for i, c in enumerate(self.scenario.intruders)}

        def render(_now: float) -> np.ndarray:
            s = self.world.quad.state
            pose = cam.pose(s.pos + self.offset, s.quat)
            targets = [
                (p + self.offset, span[i], float(np.arctan2(v[1], v[0])))
                for i, p, v in self.world.intruder_states()
            ]
            return ren.render(pose, targets, self.cfg.camera.noise_sigma, self.world.bird_states())

        return CallableSource(render, self.cfg.camera.rate_hz, clock=self.clock)
