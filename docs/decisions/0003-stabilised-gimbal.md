# 0003. Mount the camera on a stabilised gimbal

**Context.** With a rigid mount, the camera pitches with the airframe. Chasing
a target, the multirotor tilts up to 35 degrees, the camera points at the ground
or the zenith, the target leaves the frame and clutter takes its place. In one
run the target was in view in only 67 of 701 frames.

**Decision.** Model a 2-axis stabilised gimbal (roll/pitch decoupled from the
airframe, yaw following the vehicle) with a commanded tilt, a slew-rate limit and
residual jitter. During tracking the tilt points at the target's elevation.

**Measured.** With the gimbal, the target was in view for 82% of tracking time,
and 100% once guidance was also smoothed (0004).
