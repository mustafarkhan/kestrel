# 0016: Night, and a thermal camera

## Why

The drone incursions that closed Copenhagen, Aalborg and Munich airports in September and
October 2025 were all reported after dark: Copenhagen on the evening of 22 September, Aalborg
from 21:44 on 24 September, Munich in the evening of 2 October and at 21:30 the next day
([Wikipedia, *2025 European drone sightings*](https://en.wikipedia.org/wiki/2025_European_drone_sightings)).
Thermal anti-drone tracking is a benchmark of its own (CST Anti-UAV, [research.md](../research.md),
reference 17). Every benchmark before this one was flown by day. A visible camera at night was never measured, so the protection the
results tables show was a daytime number.

## What was built

* **Night and thermal rendering** (`perception/render.py`): a palette per band and light.
  Visible at night is a moonless sky a few grey levels above black, against which an unlit
  drone has almost no contrast. Thermal is an 8-bit long-wave infrared image, white hot: clear
  sky is cold and coldest at the zenith, cloud bases warmer than clear sky, ground and sea
  warmer still, and a drone's motors, speed controllers and battery the warmest things in the
  frame; a bird's feathers keep it near ambient, so it shows, but less than a drone.
* **A bright-target detector mode** (`perception/detector.py`): a thermal frame is inverted
  and runs the same pipeline, so every gate keeps its meaning ("darker than clear sky" becomes
  "warmer than the coldest sky nearby").
* **A thermal camera** (`configs/thermal.yaml`): an uncooled LWIR core of the 320 × 256,
  50° class, the same 0.16° per pixel as the visible camera over a narrower view. Cores of
  this class list at about $2,240–3,305 radiometric.
* **A night airfield** (`scenarios/airfield_night.yaml`) and the same 24 encounters
  (seeds 4000–4023) flown with a visible camera at night, a thermal camera at night, a thermal
  camera with the ground acoustic network at night, and a thermal camera by day.

## Results

| | runs inside 12 m | closest approach | encounters detected 3 s before 30 m |
|---|---|---|---|
| one camera, day (for reference) | 1/24 | 7.7 m | 18/24 |
| one camera, night | 18/24 | 3.6 m | 0/24 |
| thermal camera, night | 3/24 | 1.4 m | 16/24 |
| thermal + acoustic, night | 0/24 | 12.5 m | 20/24 |
| thermal camera, day | 1/24 | 3.1 m | 15/24 |

* **A visible camera at night is no sensor at all.** It tracked no drone in 24 encounters, and
  the runs are identical to flying with avoidance switched off: 18 of 24 inside 12 m.
* **A thermal camera restores most of the daytime protection**: 3 of 24 runs inside 12 m at
  night against 18, with no false alarms on warm clouds.
* **Its narrower view is the cost.** The three runs inside 12 m at night are encounters 17, 18
  and 20. In 18 the intruder closed from the side while Kestrel was flying home, outside the
  50° frame for its whole final approach (1.4 m); in 20 an orbiting intruder was never detected before it came close,
  by day (3.1 m) or night (6.7 m); 17 was a marginal 11.6 m. The visible camera by day, with its
  75° view, kept all three outside 12 m (12.0 m, 12.9 m and 13.5 m).
* **Ground microphones close the gap**: thermal plus the acoustic network at night kept every
  run outside 12 m (closest 12.5 m) and detected 20 of 24 encounters, the same as the daytime
  visible camera with acoustic cueing. The cost is also the same: 4 missions ended early to
  give way to traffic.

## Choices

| choice | why |
|---|---|
| Same angular resolution as the visible camera, narrower view | a 640-pixel core at 75° would keep the view at four times the price; the 320 class is what a node of this cost carries. The narrower view is measured, not hidden |
| Invert and reuse the detector | one pipeline and one set of gates, already measured against cloud clutter, instead of a second detector with its own tuning |
| No thermal bird/drone classifier change | the wing-beat flicker cue is in a target's brightness, which a thermal image still carries; its performance at night is not measured separately |

## Not done

The thermal model is a palette, not a radiometric simulation: no atmospheric attenuation with
range or humidity, no sun or cloud reflections, no non-uniformity or fixed-pattern noise, no
thermal crossover at dawn and dusk when a target and its background are at the same
temperature. Real thermal footage is the check ([real-data.md](../real-data.md)).
