# 0002. Reject cloud gaps with a darker-than-clear-sky test

**Context.** The first closed-loop run produced thousands of short-lived tracks
and the observer chased a "target" to the geofence corner. Frames showed the
detector firing on blue sky seen through holes in clouds. Those gaps are at
infinity, so while the observer translates they keep a constant bearing, which
the tracker interprets as an aircraft pacing the observer. Following it moves the
camera onto more clouds: a runaway loop.

**Decision.** A candidate must be at least 8 grey levels darker than the 10th
percentile of clear sky in the same image rows. A backlit aircraft is darker than
clear sky; a cloud gap *is* clear sky. Also added: a minimum contrast gate, and a
stricter score to *start* a track than to update one.

**Measured.** On candidate blobs from 180 random frames: 98.6% of clutter
rejected, 83% of true detections kept. False alarms on random sky frames went
from about 15 per frame (first version) to 0 in 160 frames.

**Consequences.** Sunlit targets brighter than the sky are missed; recorded as
a limitation in `docs/safety.md`.
