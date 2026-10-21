# 0006. Cascaded position-velocity loop with speed saturation

**Context.** A single PD law on position error turns a far-away setpoint into
whatever speed the gains produce. The safety monitor clipped the *velocity
feed-forward* to 8 m/s, but the vehicle still reached 13.8 m/s, because the
speed came from the position error, not the feed-forward.

**Decision.** Position error produces a velocity command, saturated to the
autopilot's own envelope, and velocity error produces acceleration, the same
structure ArduPilot and PX4 use (`WPNAV_SPEED`, `MPC_XY_VEL_MAX`). Goals derived
from perception additionally pass through a second-order reference filter with
speed and acceleration limits.

**Consequences.** Speed limits hold regardless of how far away a setpoint is.
Covered by `tests/test_control.py::test_large_step_respects_envelope`.
