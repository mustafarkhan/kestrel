# 0008. MAVLink adapter rules

- **One reader.** Only the receive thread calls `recv_match`; command
  acknowledgements are handed over through a condition variable. Two readers on
  one connection steal each other's messages.
- **Origin captured on the ground.** The local frame is re-zeroed at arming,
  while landed. Capturing it after takeoff shifts every altitude by the takeoff
  height.
- **Ids from the heartbeat.** `wait_heartbeat` leaves `target_component` at 0; the
  first SITL flight armed and disarmed in a loop because heartbeats from
  component 1 were filtered out and the vehicle never appeared armed.
- **Arm means armed.** `arm()` returns only once a heartbeat reports the armed
  state, not when the acknowledgement arrives.
- **PX4 refused, not half-supported.** OFFBOARD needs a pre-armed setpoint stream
  and different mode handling; until that is implemented and tested, the adapter
  rejects non-ArduPilot autopilots.
