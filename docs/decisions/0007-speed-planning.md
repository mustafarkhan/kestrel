# 0007. Curvature-aware speed planning along paths

**Context.** On ArduCopter SITL a 15 m orbit took 90 s. The path follower used a
single cruise speed for the whole path, capped by its tightest corner (the 90
degree join onto the circle).

**Decision.** Per-point speed limit from the lateral-acceleration budget at the
local curvature, then a forward pass (acceleration limit) and a backward pass
(braking limit). The reference is integrated from that profile, so position,
velocity and acceleration stay consistent.

**Measured.** Orbit with entry: 90 s before, 22 s after. Writing the
lateral-acceleration test then exposed a bug in the curvature estimate (tangent
change divided by 1.5 segments instead of the 2 separating the tangents), which
under-read curvature by a third.
