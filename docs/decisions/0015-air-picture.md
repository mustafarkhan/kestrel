# 0015: The recognised air picture

## Why

Research for v0.4 ([research.md, section 6](../research.md#6-2026-what-the-front-line-the-market-and-nato-ask-of-the-picture))
pointed one way from three directions. Front-line reporting: sensing and cueing, not
interceptors, are the bottleneck, every sensor class has a blind spot, and multi-sensor
fusion is the approach that works. The market: Europe is short of networked sensing and
cross-border C2, and buyers want open standards. NATO: the 2026 counter-UAS data challenge
asks for one local recognised air picture from radar, RF, EO/IR and acoustic data, with
duplicates removed, classification, and friend/hostile/unknown identification.

Kestrel was one sensor node reporting into a picture someone else would build. v0.4 builds
the picture, and lets it task Kestrel.

## What was built

* **Ground sensors as SAPIENT nodes** (`rap/sensors.py`): radar (radar equation, clutter
  masking at low altitude, track breaks, clutter tracks, a weak micro-Doppler opinion), RF
  direction finders (blind to radio-silent drones), a Remote ID receiver (cooperative, and
  spoofable), an acoustic grid (range by propulsion, no altitude).
* **A SAPIENT fusion node** (`rap/fusion.py`): association by sensor object id then gate,
  Kalman fusion with each report's own errors, duplicate merging, bounded class evidence,
  APP-6 standard identities with *hostile* reserved for an operator, and the choice of the
  track that most needs a look.
* **The loop** (`rap/loop.py`): the fusion node runs alongside a flight, reads Kestrel's own
  reports, and sends `LOOK_AT` tasks over the same link as every other tasking.
* **Scoring** (`rap/score.py`): GOSPA, coverage, duplicates, track continuity,
  classification and identification against ground truth.
* **Replay** (`rap/replay.py`, `kestrel fuse`): the same fusion on a recorded SAPIENT log, so
  real sensor data, such as the NATO challenge's, can be run without a scenario file.
* **Schema conformance**: Dstl has published the BSI Flex 335 protobuf files; every message
  kind Kestrel writes is now parsed against them in the test suite.

## Choices

| choice | why |
|---|---|
| Fuse sensor reports as measurements, with their stated errors | the sensors here report plots or fixes, so their errors are close to independent; the fused covariance is consistent (mean NEES 2.1–2.4 against 3 expected, slightly conservative). Track-level reports from real sensors' own trackers would call for covariance intersection |
| Class evidence per sensor node, mean log-odds, at most three looks | a radar's per-scan micro-Doppler scores on one object are correlated; summed naively, any sensor that reports often becomes certain on its own. With the bound, radar alone leaves most drones *unknown*, which is what fielded radars report |
| *Friend* needs Remote ID plus an independent sensor in the right place | Remote ID is not authenticated. Without corroboration, a broadcast is *assumed friend*; a serial seen in two places is never *friend* |
| *Hostile* only by an operator | identification is a decision with consequences; the picture shows suspect and unknown and leaves the decision to people |
| Kestrel closes to 30 m on a 3-D track, 45 m on a column | a radar or fused track places the aircraft in 3-D, so the camera can come to the follow distance where its classifier has enough pixels; a fix without altitude is a column to keep clear of |

## Faults found on the way

| fault | effect | fix |
|---|---|---|
| Class evidence summed over every report | a radar's correlated 0.3 scores made every bird *certain*; a camera that misread one drone flipped it to *bird* | per-node mean log-odds with at most three looks; camera weight 0.6 for a classifier whose errors are measured |
| Remote ID objects keyed by serial | a spoofer and the friend it copied shared one object id, so a phantom track inherited the friend's corroboration and was shown as *friend* | object ids per broadcaster; corroboration only from a precise sensor inside the gate of the broadcast position; the clone rule |
| Tasks sent for tracks outside the area | Kestrel flew to the fence chasing birds outside it and timed out | tasks only for tracks inside the area the airborne node may fly |
| Re-sent positions queued as new looks | stale positions piled up; Kestrel left the drone it was classifying to visit old ones | a re-sent position updates the look under way; a newer task replaces one still waiting |
| A look ended when the camera first saw the track | Kestrel left before its classifier had enough frames | a look ends when the picture can say what the track is |
| Field names taken from the ICD text | registration, status location and the look-at path did not parse against the published schema | aligned with Dstl's protobuf; conformance test |

## Results

The air-picture tables in the README (picture: seeds 7000–7023; with Kestrel: 8000–8023,
neither used while developing).
