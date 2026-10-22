# 0009: Birds, and telling them from drones

## Problem

Every source on counter-UAS sensing names birds as the main false-alarm problem. Radar
sees them; cameras see them. At the ranges where detection matters, a crow and a
quadcopter are both a dark blob a few pixels across. Until now Kestrel's sky had clouds
and no birds, so its false-alarm figure was optimistic in exactly the way a reviewer
from the field would spot first.

## What was added

**Birds in the simulation** (`sim/birds.py`). Medium birds, 0.5–1.0 m span (corvid to
gull), at 8–13 m/s and 10–60 m above ground, wandering on slowly varying turn and climb
rates, replaced at the edge of the area when they leave it. They fly intermittently: on
average 70% flapping at 3–6 Hz and 30% gliding, in bouts of a few seconds. The renderer
draws a body and two wings whose visible extent and height follow the wing-beat
(`perception/render.py`).

At first sight, birds are a harder detection problem than drones, not an easier one:
with a larger span they are detected further out. In the same sky, a 0.35 m drone was
detected in 85% of frames at 60 m and 13% at 100 m; birds in 83% at 100 m and about half
at 150 m.

**A classifier on wing-beat flicker** (`perception/classify.py`). A flapping wing sweeps
in and out of the silhouette, so the detected blob's integrated darkness jumps from frame
to frame. A multirotor's props blur into steady discs, so its darkness changes only by
sensor noise. The classifier:

1. takes the change in log flux between consecutive frames of a track;
2. divides it by the change sensor noise alone would produce at that brightness,
   `1.39 * sqrt(1/f1 + 1/f2)`, so that a steady target scores about 1 at any range;
3. takes the RMS over the last 40 frame pairs (4 s), with each change clipped at 6 so
   one cloud edge cannot decide;
4. maps it to a probability of *drone* with a logistic around 2.0, whose slope grows
   with the number of pairs. Below 8 pairs it gives no opinion (0.5).

A 10 Hz camera samples a 3–6 Hz wing-beat at or below the Nyquist rate, so a frequency
measurement would alias. The size of the changes survives aliasing; that is why the
feature is the RMS, not a spectral peak.

**How the rest of the stack uses it.** A track is *reported* as a drone (event log,
SAPIENT classification, "follow any drone" interrupts) only when the classifier puts it
at 0.6 or more. Every track, bird or drone, still gets the full separation keep-out: a
bird is a collision hazard too, and a drone misclassified as a bird must never lose
protection. So the classifier can only change what is reported, not what is avoided.

## Calibration and held-out check

The noise coefficient (1.57) and threshold (2.0) were fitted on a calibration set of
rendered sequences (seeds 200–649: drones, flapping birds and gliding birds at 20–140 m,
random headings, moving camera, noisy cloudy sky).

| 15-frame windows, calibration set | threshold 1.5 | **2.0** | 2.5 |
|---|---|---|---|
| drones kept | 91% | **100%** | 100% |
| flapping birds rejected | 84% | **82%** | 69% |
| gliding birds rejected | 29% | **5%** | 2% |

A further rule came from a closed-loop check: distant birds (integrated darkness below
about 100) were being called drones, because at that brightness sensor noise in the
frame-to-frame change (about 0.22 in log flux) is as large as the wing-beat modulation of
a bird seen side-on. Below that brightness the classifier now gives no opinion instead.

On a separate held-out set (seeds 5000–5239, 3 s sequences, never used for fitting),
with the reporting threshold of 0.6:

| class | sequences | reported as drone | called bird | no opinion |
|---|---|---|---|---|
| drones | 31 | 81% | 0% | 19% (too faint) |
| flapping birds | 22 | 27% | 73% | 0% |
| gliding birds | 32 | 91% | 3% | 6% |

No drone was called a bird. The threshold leans that way on purpose: missing a real drone
costs more than reporting a bird. Gliding birds are reported as drones about as often as
drones are, because a gliding bird has no wing-beat to see. That is a property of the
cue, not a tuning problem, and it is why the closed-loop result below is a reduction and
not an elimination.

A side effect worth recording: seen side-on, a bird's wing-beat is mostly up-and-down
motion and changes the silhouette's area less than when it flies towards or away from
the camera, so aspect matters as much as range.

## Closed-loop result

See the README results matrix (`birds, classifier off` against `birds, classifier on`)
and `docs/results/`.

## What would do better

* A higher frame rate (30–60 Hz) resolves the wing-beat frequency itself.
* Trajectory cues: birds soar, circle and change height in ways small multirotors do
  not. A track-level motion classifier would also catch gliders.
* A learned appearance classifier at close range, trained and tested on real footage
  (the first roadmap item).
