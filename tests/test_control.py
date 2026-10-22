from __future__ import annotations

import math

import numpy as np
from hypothesis import given, settings
from hypothesis import strategies as st

from kestrel.config import Config
from kestrel.control.allocation import Mixer
from kestrel.control.geometric import GeometricController
from kestrel.geometry import quat_to_euler, wrap_pi
from kestrel.sim.quadrotor import GRAVITY, Quadrotor
from kestrel.types import Setpoint


def _fly(cfg: Config, sp: Setpoint, seconds: float, yaw0: float = 0.0, wind=(0.0, 0.0, 0.0)):
    q = Quadrotor(cfg.vehicle, pos=np.array([0.0, 0.0, 20.0]), yaw=yaw0)
    q.state.thrusts[:] = cfg.vehicle.mass * GRAVITY / 4
    ctl = GeometricController(cfg.vehicle, cfg.control)
    dt = cfg.physics_dt
    max_tilt = 0.0
    min_z = math.inf
    max_speed = 0.0
    for _ in range(int(seconds / dt)):
        thrust, torque = ctl.compute(q.state, sp, dt)
        q.step(dt, q.mixer.mix(thrust, torque), np.array(wind))
        r, p, _ = quat_to_euler(q.state.quat)
        max_tilt = max(max_tilt, math.degrees(math.acos(math.cos(r) * math.cos(p))))
        min_z = min(min_z, q.state.pos[2])
        max_speed = max(max_speed, float(np.linalg.norm(q.state.vel[0:2])))
    return q, max_tilt, min_z, max_speed


def test_hover_rejects_wind(cfg: Config) -> None:
    q, tilt, _, _ = _fly(cfg, Setpoint.hold(np.array([0.0, 0.0, 20.0]), 0.0), 15.0, wind=(4.0, -2.0, 0.0))
    assert np.linalg.norm(q.state.pos - [0, 0, 20]) < 0.15  # integral action removes the offset
    assert tilt < 15.0


def test_large_step_respects_envelope(cfg: Config) -> None:
    q, tilt, min_z, vmax = _fly(cfg, Setpoint.hold(np.array([60.0, -20.0, 30.0]), 1.0), 20.0)
    assert np.linalg.norm(q.state.pos - [60, -20, 30]) < 0.3
    assert vmax < cfg.control.max_speed_xy * 1.1  # cascaded loop saturates speed
    assert tilt < cfg.control.max_tilt_deg + 3.0
    assert min_z > 19.0


def test_yaw_reversal_does_not_upset_attitude(cfg: Config) -> None:
    # Regression: a 180 degree heading step once drove a roll/pitch limit cycle.
    q, tilt, min_z, _ = _fly(cfg, Setpoint.hold(np.array([0.0, 0.0, 20.0]), math.pi), 6.0)
    assert tilt < 8.0
    assert min_z > 19.5
    assert abs(wrap_pi(quat_to_euler(q.state.quat)[2] - math.pi)) < math.radians(2)


@settings(max_examples=200, deadline=None)
@given(
    st.floats(0.0, 40.0),
    st.floats(-3.0, 3.0),
    st.floats(-3.0, 3.0),
    st.floats(-0.5, 0.5),
)
def test_mixer_outputs_are_feasible(total: float, tx: float, ty: float, tz: float) -> None:
    m = Mixer(Config().vehicle)
    f = m.mix(total, np.array([tx, ty, tz]))
    assert np.all(f >= 0.0) and np.all(f <= m.max_thrust + 1e-9)


def test_mixer_exact_when_feasible() -> None:
    m = Mixer(Config().vehicle)
    want = np.array([16.0, 0.2, -0.1, 0.02])
    f = m.mix(want[0], want[1:])
    assert np.allclose(m.wrench(f), want, atol=1e-9)
