"""ArduPilot vehicle over MAVLink (GUIDED mode), via pymavlink.

Implements :class:`kestrel.vehicle.base.Vehicle` against a real or SITL
ArduCopter. Integration-tested against ArduCopter SITL in CI
(``tests/test_sitl.py``).

Engineering notes, each from a real failure:

* **One reader.** Only the receive thread reads the socket. ``COMMAND_ACK``\\ s are
  handed to waiting callers through a condition variable. Two threads calling
  ``recv_match`` on the same connection lose messages to each other.
* **Origin on the ground.** Kestrel's world frame is "home = (0, 0, 0)". The EKF
  local origin is not necessarily home, so the offset is captured while landed,
  before takeoff. Capturing it in the air shifts every altitude by the takeoff
  height.
* **Frames in one place.** MAVLink is NED/FRD; Kestrel is ENU/FLU. All conversion
  happens in this file and nowhere else.
* **PX4 is not claimed.** PX4 OFFBOARD needs a pre-armed setpoint stream and
  different mode handling. It is detected and refused rather than half-supported.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

import numpy as np
from pymavlink import mavutil

from kestrel.geometry import Vec, enu_to_ned, ned_to_enu, rot_to_quat
from kestrel.types import FlightMode, Setpoint, VehicleState

M = mavutil.mavlink

# NED <- ENU and FRD <- FLU are both their own inverses
_R_ENU_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
_R_FRD_FLU = np.diag([1.0, -1.0, -1.0])

# use position, velocity, acceleration and yaw; ignore yaw rate
_TYPE_MASK = M.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE


class MavlinkError(RuntimeError):
    pass


def quat_ned_frd_to_enu_flu(q_ned_frd: Vec) -> Vec:
    w, x, y, z = q_ned_frd
    r_ned_frd = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )
    return rot_to_quat(_R_ENU_NED @ r_ned_frd @ _R_FRD_FLU)


def yaw_enu_to_ned(yaw_enu: float) -> float:
    return (math.pi / 2 - yaw_enu + math.pi) % (2 * math.pi) - math.pi


@dataclass
class _Telemetry:
    t: float = 0.0
    pos_ned: Vec | None = None
    vel_ned: Vec | None = None
    quat_enu: Vec | None = None
    omega_flu: Vec | None = None
    armed: bool = False
    custom_mode: int = -1
    battery_pct: float = -1.0
    landed: bool | None = None
    last_heartbeat: float = 0.0


class MavlinkVehicle:
    def __init__(
        self,
        url: str,
        battery_capacity_wh: float,
        connect_timeout_s: float = 30.0,
        telemetry_hz: float = 25.0,
    ) -> None:
        self.capacity = battery_capacity_wh
        self.conn = mavutil.mavlink_connection(
            url, source_system=245, source_component=M.MAV_COMP_ID_ONBOARD_COMPUTER, autoreconnect=True
        )
        hb = self.conn.wait_heartbeat(timeout=connect_timeout_s)
        if hb is None:
            raise MavlinkError(f"no heartbeat from {url} within {connect_timeout_s:.0f} s")
        if hb.autopilot != M.MAV_AUTOPILOT_ARDUPILOTMEGA:
            raise MavlinkError("only ArduPilot is supported (PX4 OFFBOARD is not implemented)")
        # take ids from the heartbeat itself: wait_heartbeat leaves target_component at 0
        self.sys = hb.get_srcSystem()
        self.comp = hb.get_srcComponent()
        self.modes = self.conn.mode_mapping() or {}
        self._tel = _Telemetry()
        self._lock = threading.Lock()
        self._acks: dict[int, int] = {}
        self._ack_cv = threading.Condition(self._lock)
        self._origin_ned: Vec | None = None
        self._running = True
        self._boot_wall = time.monotonic()
        self.statustext: list[str] = []
        self._rx = threading.Thread(target=self._rx_loop, name="mavlink-rx", daemon=True)
        self._rx.start()
        self._hb = threading.Thread(target=self._hb_loop, name="mavlink-hb", daemon=True)
        self._hb.start()
        interval_us = int(1e6 / telemetry_hz)
        for msg_id in (M.MAVLINK_MSG_ID_LOCAL_POSITION_NED, M.MAVLINK_MSG_ID_ATTITUDE_QUATERNION):
            self._command(M.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, interval_us, wait=False)
        for msg_id in (M.MAVLINK_MSG_ID_SYS_STATUS, M.MAVLINK_MSG_ID_EXTENDED_SYS_STATE):
            self._command(M.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 500_000, wait=False)

    # ------------------------------------------------------------------ threads
    def _rx_loop(self) -> None:
        while self._running:
            msg = self.conn.recv_match(blocking=True, timeout=0.5)
            if msg is None or msg.get_srcSystem() != self.sys:
                continue
            kind = msg.get_type()
            with self._lock:
                tel = self._tel
                if kind == "LOCAL_POSITION_NED":
                    tel.pos_ned = np.array([msg.x, msg.y, msg.z])
                    tel.vel_ned = np.array([msg.vx, msg.vy, msg.vz])
                    tel.t = msg.time_boot_ms / 1000.0
                elif kind == "ATTITUDE_QUATERNION":
                    tel.quat_enu = quat_ned_frd_to_enu_flu(np.array([msg.q1, msg.q2, msg.q3, msg.q4]))
                    tel.omega_flu = np.array([msg.rollspeed, -msg.pitchspeed, -msg.yawspeed])
                elif kind == "HEARTBEAT" and msg.get_srcComponent() == self.comp:
                    tel.armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
                    tel.custom_mode = msg.custom_mode
                    tel.last_heartbeat = time.monotonic()
                elif kind == "SYS_STATUS":
                    tel.battery_pct = float(msg.battery_remaining)
                elif kind == "EXTENDED_SYS_STATE":
                    tel.landed = msg.landed_state == M.MAV_LANDED_STATE_ON_GROUND
                elif kind == "COMMAND_ACK":
                    self._acks[msg.command] = msg.result
                    self._ack_cv.notify_all()
                elif kind == "STATUSTEXT":
                    self.statustext.append(msg.text)

    def _hb_loop(self) -> None:
        while self._running:
            self.conn.mav.heartbeat_send(M.MAV_TYPE_ONBOARD_CONTROLLER, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(1.0)

    def close(self) -> None:
        self._running = False
        self._rx.join(timeout=2.0)
        self._hb.join(timeout=2.0)
        self.conn.close()

    # ------------------------------------------------------------------ commands
    def _command(self, cmd: int, *params: float, wait: bool = True, timeout_s: float = 3.0) -> int | None:
        p = list(params) + [0.0] * (7 - len(params))
        with self._lock:
            self._acks.pop(cmd, None)
        self.conn.mav.command_long_send(self.sys, self.comp, cmd, 0, *p)
        if not wait:
            return None
        deadline = time.monotonic() + timeout_s
        with self._ack_cv:
            while cmd not in self._acks:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not self._ack_cv.wait(remaining):
                    return None
            return self._acks.pop(cmd)

    def _set_mode(self, name: str) -> bool:
        mode = self.modes.get(name)
        if mode is None:
            raise MavlinkError(f"autopilot has no {name} mode")
        result = self._command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)
        return bool(result == M.MAV_RESULT_ACCEPTED)

    def wait_ready(self, timeout_s: float = 90.0) -> None:
        """Block until a local position estimate is streaming (EKF has an origin)."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._tel.pos_ned is not None and self._tel.quat_enu is not None:
                    return
            time.sleep(0.2)
        raise MavlinkError("no local position estimate: is the EKF origin set (GPS fix / origin)?")

    # ------------------------------------------------------------------ Vehicle API
    def state(self) -> VehicleState:
        with self._lock:
            tel = self._tel
            if tel.pos_ned is None or tel.vel_ned is None or tel.quat_enu is None:
                raise MavlinkError("no navigation solution yet; call wait_ready() first")
            origin = self._origin_ned if self._origin_ned is not None else tel.pos_ned
            pos = ned_to_enu(tel.pos_ned - origin)
            vel = ned_to_enu(tel.vel_ned)
            mode = FlightMode.DISARMED
            if tel.armed:
                mode = FlightMode.LAND if tel.custom_mode == self.modes.get("LAND") else FlightMode.GUIDED
            landed = tel.landed if tel.landed is not None else (not tel.armed or pos[2] < 0.2)
            pct = tel.battery_pct if tel.battery_pct >= 0 else 100.0
            return VehicleState(
                t=tel.t,
                pos=pos,
                vel=vel,
                quat=tel.quat_enu.copy(),
                omega=tel.omega_flu.copy() if tel.omega_flu is not None else np.zeros(3),
                armed=tel.armed,
                on_ground=bool(landed),
                mode=mode,
                battery_wh=self.capacity * pct / 100.0,
                battery_capacity_wh=self.capacity,
            )

    def arm(self) -> bool:
        st = self.state()
        if not st.on_ground:
            return st.armed
        with self._lock:
            # origin captured on the ground, before any climb
            self._origin_ned = None if self._tel.pos_ned is None else self._tel.pos_ned.copy()
        if not self._set_mode("GUIDED"):
            return False
        if self._command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1) != M.MAV_RESULT_ACCEPTED:
            return False
        # the ACK can arrive before the heartbeat that reports the armed state
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            with self._lock:
                if self._tel.armed:
                    return True
            time.sleep(0.05)
        return False

    def takeoff(self, altitude: float) -> None:
        self._command(M.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, float("nan"), 0, 0, altitude)

    def set_setpoint(self, sp: Setpoint) -> None:
        with self._lock:
            origin = self._origin_ned if self._origin_ned is not None else np.zeros(3)
        p = enu_to_ned(np.asarray(sp.pos)) + origin
        v = enu_to_ned(np.asarray(sp.vel))
        a = enu_to_ned(np.asarray(sp.acc))
        boot_ms = int((time.monotonic() - self._boot_wall) * 1000) & 0xFFFFFFFF
        self.conn.mav.set_position_target_local_ned_send(
            boot_ms,
            self.sys,
            self.comp,
            M.MAV_FRAME_LOCAL_NED,
            _TYPE_MASK,
            *p,
            *v,
            *a,
            yaw_enu_to_ned(sp.yaw),
            0.0,
        )

    def land(self) -> None:
        self._set_mode("LAND")

    def disarm(self) -> bool:
        return bool(self._command(M.MAV_CMD_COMPONENT_ARM_DISARM, 0) == M.MAV_RESULT_ACCEPTED)
