# 0012: Do not stare at where an intruder used to be

## Problem

After the renderer correction ([0010](0010-area-true-rendering.md)) the one-camera
configuration lost well clear in 3 of 24 encounters on the diagnosis set (seeds
3000–3023), the closest at 2.1 m. Tracing each one:

| encounter | what happened at closest approach |
|---|---|
| mc009 | observer flying a survey lane west (heading 180°) with its camera facing east (yaw 0°); the intruder came head-on from the west, unseen until it had passed |
| mc015 | intruder approached from about 30° off the direction of travel, camera turned away |
| mc020 | intruder approached along the lane, camera turned away, never detected |

In each case the camera was pointed at the **last known position of the intruder seen a
minute or more earlier**. That was the contact watch from [0004](0004-separation.md):
for up to 180 s after a sighting, while within 70 m of it, point the camera there,
"so that a re-pass is seen". It was added for a real failure, an intruder turning back
on the observer. But after a few tens of seconds an unseen aircraft could be anywhere
within a radius far larger than the watch radius (8 m/s × 60 s = 480 m). Staring at its
old position bought nothing and left the direction of travel unwatched, which is where
own motion makes closing speeds highest.

## Fix

The watch now lasts 15 s after the last sighting: long enough for an aircraft that is
turning back to be caught (it is still within about 135 m of where it was), short enough
that the camera returns to the direction of travel and its sweep afterwards.

## Result on the diagnosis set

| one camera, seeds 3000–3023 | watch 180 s | watch 15 s |
|---|---|---|
| runs inside 12 m | 3/24 | 1/24 |
| closest approach | 2.1 m | 7.3 m |
| missions completed and landed | 24/24 | 24/24 |

These encounters were used to find the fault, so the published matrix is flown on a new
set (seeds 4000–4023) that was not looked at before these numbers were fixed.

## What would do better

A forward camera has to choose where to look. The structural fixes are more coverage
(the acoustic and camera-ring rows of the matrix) or an active search policy that
weighs the reachable set of every unseen contact against the direction of travel.
