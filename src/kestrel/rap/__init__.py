"""The recognised air picture: ground sensors, a SAPIENT fusion node, and its scoring.

Kestrel on its own is one sensor node. Front-line reporting and NATO's own data challenge
point the same way: no single sensor class sees every drone, so a picture is built by
fusing radar, RF, acoustic, cooperative identification and cameras, removing duplicate
tracks, fusing class evidence, and identifying friend from unknown. This package does that
in simulation, with every sensor speaking SAPIENT, so the fusion node can be pointed at
recorded SAPIENT traffic from real sensors without change.
"""
