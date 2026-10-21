"""Runtime safety monitor: invariants checked as properties over random inputs."""

from __future__ import annotations

import math

import numpy as np
from hypothesis import given, settings
from hypothesis import strategies as st

from kestrel.config import Config, WorldConfig
from kestrel.geofence import Geofence
from kestrel.geometry import quat_from_euler
from kestrel.perception.tracker import TrackEstimate
from kestrel.safety.monitor import SafetyMonitor, SafetyRequest
from kestrel.types import FlightMode, Setpoint, VehicleState

CFG = Config()
WORLD = WorldConfig(fence=[(-100, -100), (100, -100), (120, 60), (0, 140), (-100, 100)])
LIM = CFG.safety

coord = st.floats(-250, 250)
alt = st.floats(-20, 120)
spd = st.floats(-30, 30)


def _state(pos, vel=(0.0, 0.0, 0.0), battery=70.0, t=10.0, armed=True) -> VehicleState:
    return VehicleState(
        t=t,
        pos=np.array(pos, dtype=float),
        vel=np.array(vel, dtype=float),
        quat=quat_from_euler(0, 0, 0),
        omega=np.zeros(3),
        armed=armed,
        on_ground=False,
        mode=FlightMode.GUIDED,
        battery_wh=battery,
        battery_capacity_wh=CFG.vehicle.battery_wh,
    )


@settings(max_examples=400, deadline=None)
@given(coord, coord, alt, spd, spd, spd)
def test_setpoints_always_inside_envelope(
    x: float, y: float, z: float, vx: float, vy: float, vz: float
) -> None:
    mon = SafetyMonitor(LIM, CFG.vehicle, WORLD)
    st_ = _state((0.0, 20.0, 20.0))
    dec = mon.filter(st_, Setpoint(pos=np.array([x, y, z]), vel=np.array([vx, vy, vz])), [])
    sp = dec.setpoint
    assert sp is not None
    assert Geofence(WORLD.fence).signed_distance(sp.pos) >= LIM.fence_margin - 0.5
    assert LIM.min_altitude - 1e-9 <= sp.pos[2] <= LIM.max_altitude + 1e-9
    assert math.hypot(sp.vel[0], sp.vel[1]) <= LIM.max_horizontal_speed + 1e-9
    assert abs(sp.vel[2]) <= LIM.max_vertical_speed + 1e-9


@settings(max_examples=300, deadline=None)
@given(st.floats(-40, 40), st.floats(-40, 40), st.floats(-15, 15), st.floats(0.0, 6.0))
def test_setpoints_keep_out_of_observed_tracks(dx: float, dy: float, dz: float, std: float) -> None:
    mon = SafetyMonitor(LIM, CFG.vehicle, WORLD)
    track_pos = np.array([0.0, 20.0, 30.0])
    tr = TrackEstimate(1, track_pos, np.zeros(3), np.eye(6) * std**2, True, 50, last_hit_t=10.0)
    st_ = _state((30.0, 20.0, 30.0))
    dec = mon.filter(st_, Setpoint(pos=track_pos + np.array([dx, dy, dz])), [tr])
    assert dec.setpoint is not None
    assert float(np.linalg.norm(dec.setpoint.pos - track_pos)) >= LIM.min_separation - 1e-6


def test_keep_out_grows_while_track_is_unseen() -> None:
    mon = SafetyMonitor(LIM, CFG.vehicle, WORLD)
    track_pos = np.array([0.0, 20.0, 30.0])
    tr = TrackEstimate(1, track_pos, np.zeros(3), np.eye(6), True, 50, last_hit_t=6.0)  # unseen for 4 s
    dec = mon.filter(_state((40.0, 20.0, 30.0)), Setpoint(pos=track_pos + np.array([20.0, 0, 0])), [tr])
    assert dec.setpoint is not None
    assert np.linalg.norm(dec.setpoint.pos - track_pos) > LIM.min_separation + 25.0


def test_closing_speed_enlarges_buffer() -> None:
    mon = SafetyMonitor(LIM, CFG.vehicle, WORLD)
    track_pos = np.array([0.0, 20.0, 30.0])
    tr = TrackEstimate(1, track_pos, np.array([4.0, 0, 0]), np.eye(6), True, 50, last_hit_t=10.0)
    st_ = _state((20.0, 20.0, 30.0), vel=(-5.0, 0.0, 0.0))  # 9 m/s closing
    dec = mon.filter(st_, Setpoint(pos=track_pos + np.array([18.0, 0, 0])), [tr])
    assert dec.setpoint is not None
    assert np.linalg.norm(dec.setpoint.pos - track_pos) > 18.0 + 5.0


def test_energy_reserve_latches_rtl_then_land() -> None:
    mon = SafetyMonitor(LIM, CFG.vehicle, WORLD)
    mon.check_vehicle(_state((50.0, 50.0, 30.0), battery=60.0), link_ok=True)
    assert mon.latched is SafetyRequest.NONE
    mon.check_vehicle(_state((50.0, 50.0, 30.0), battery=17.0), link_ok=True)
    assert mon.latched is SafetyRequest.RTL
    mon.check_vehicle(_state((50.0, 50.0, 30.0), battery=60.0), link_ok=True)  # good news does not unlatch
    assert mon.latched is SafetyRequest.RTL
    mon.check_vehicle(_state((50.0, 50.0, 30.0), battery=7.0), link_ok=True)
    assert mon.latched is SafetyRequest.LAND


def test_link_loss_latches_rtl_after_timeout() -> None:
    mon = SafetyMonitor(LIM, CFG.vehicle, WORLD)
    mon.check_vehicle(_state((0.0, 0.0, 20.0), t=10.0), link_ok=True)
    mon.check_vehicle(_state((0.0, 0.0, 20.0), t=10.0 + LIM.link_loss_timeout_s - 0.5), link_ok=False)
    assert mon.latched is SafetyRequest.NONE
    mon.check_vehicle(_state((0.0, 0.0, 20.0), t=10.0 + LIM.link_loss_timeout_s + 0.5), link_ok=False)
    assert mon.latched is SafetyRequest.RTL


def test_no_braking_when_it_can_stop_in_time() -> None:
    mon = SafetyMonitor(LIM, CFG.vehicle, WORLD)
    st_ = _state((-60.0, 0.0, 20.0), vel=(-8.0, 0.0, 0.0))
    dec = mon.filter(st_, Setpoint(pos=np.array([-62.0, 0.0, 20.0]), vel=np.array([-8.0, 0, 0])), [])
    assert dec.interventions == []


def test_fence_braking_before_crossing() -> None:
    mon = SafetyMonitor(LIM, CFG.vehicle, WORLD)
    # 9 m inside the fence at 8 m/s outbound: stopping takes 8 m + 1 m reaction margin
    st_ = _state((-91.0, 0.0, 20.0), vel=(-8.0, 0.0, 0.0))
    dec = mon.filter(st_, Setpoint(pos=np.array([-80.0, 0.0, 20.0]), vel=np.array([-8.0, 0, 0])), [])
    assert "fence_brake" in dec.interventions
    assert dec.setpoint is not None and np.allclose(dec.setpoint.vel[0:2], 0.0)


@given(coord, coord)
def test_project_inside_is_inside(x: float, y: float) -> None:
    f = Geofence(WORLD.fence)
    p = f.project_inside(np.array([x, y]), 8.0)
    assert f.signed_distance(p) >= 8.0 - 0.5
