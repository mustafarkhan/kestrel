# 0013: Two intruders at once

## What was measured

Allied studies of the drone threat plan for several drones at once, not one. The
benchmark can now fly any number of intruders per encounter (`kestrel bench
--intruders N`); the first intruder of each encounter is drawn exactly as in the
single-intruder sets, and the others come from a separate random stream. On the
published set (seeds 4000–4023), with two intruders:

| configuration | runs inside 12 m (entries) | closest | missions completed (returned early) |
|---|---|---|---|
| no avoidance | 20/24 (30) | 1.3 m | 24/24 (0) |
| one camera | 15/24 (26) | 1.0 m | 22/24 (2) |
| camera + acoustic cueing | 4/24 (5) | 3.7 m | 12/24 (12) |

With one intruder the same camera configuration had 1/24. Two intruders break it.

## Why

Every loss of well clear in 8 of the camera-only encounters (17 entries into the 12 m
sphere) was classified from the flight log by whether the aircraft that came close had
been in a camera's view, and tracked, in the 3 s before:

| in the 3 s before entry | entries |
|---|---|
| not in view (often while the camera was following the other drone) | 12 |
| in view but not detected: slightly below the observer, on the horizon, where the detector does not search | 1 |
| in view and tracked; the entry was marginal (10.8–11.7 m) | 4 |

A single forward camera looks at one thing. Following one drone at a standoff points
it away from the other, and the other is exactly as fast. That is a coverage problem,
not a tuning problem, and the acoustic row shows it: microphones that hear both
aircraft cut the runs inside 12 m from 15 to 4.

## The price

The acoustic network gives no altitude, so an unseen, cued aircraft is kept clear of as
a vertical column, and with two intruders over the work area there is nearly always one.
Half the missions spent their two-minute giving-way budget and came home early. That is
the conservative failure, the right one to have; every run landed at home. Better
altitude information (a second sensor type, or a camera ring that can confirm the
cue) is what would let the mission continue.

## Not addressed

* Intruders that sit on the horizon slightly below the observer: searching against
  ground clutter is out of scope for this detector ([0004](0004-separation.md)).
* Groups larger than two, and intruders that coordinate.
