# 0005. Rate-limit yaw and prioritise roll/pitch in the mixer

**Context.** Mid-survey the observer fell from 25 m to the ground. Instrumenting
the controller showed a limit cycle: after a heading reversal, roll and pitch
swung by about 80 degrees at 1.4 Hz with motors banging between zero and full
thrust. Near a 180 degree heading error the SO(3) attitude error is almost
degenerate and couples the yaw demand into roll and pitch; the mixer then gave up
collective thrust to chase attitude.

**Decision.**
- Slew-limit the yaw reference inside the attitude controller (90 deg/s), as
  autopilots do (ArduPilot `ATC_RATE_Y_MAX`).
- Mixer priority: roll/pitch torque, then collective thrust, then yaw.
- Retune attitude gains with the 30 ms motor lag in the loop.

**Measured.** A 180 degree turn in hover: 47.6 degrees of tilt and 4.0 s of
motor saturation before, 3.6 degrees after. Aggressive 30-40 m steps with yaw
reversal: peak tilt 54 degrees before, 35.2 degrees after (limit 35). Covered by
`tests/test_control.py::test_yaw_reversal_does_not_upset_attitude`.
