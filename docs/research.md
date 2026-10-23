# Research and standards map

How Kestrel relates to published detect-and-avoid (DAA) standards, air-to-air
detection research, and lessons reported from the war in Ukraine. Each row says
whether Kestrel **implements**, **measures**, or **declares out of scope** the
item, so a reader can see where it stands against the field rather than take a
claim on trust. Sources are listed at the end; figures are quoted from them.

Scope note: Kestrel is an observer. It detects, tracks, keeps clear and reports.
It has no intercept or engagement function, and battlefield sources are used here
only for what they say about *sensing*.

## 1. How the field measures detection

| Item | What the source says | Kestrel |
|---|---|---|
| Encounter-level detection (Amazon Prime Air AOT benchmark) | An encounter counts as detected if the aircraft is tracked for **3 s before** it is within **300 m**; at most **one false alarm per two flight hours** [1] | **Measures** the same definition scaled to small drones: a correct track for 3 s before the intruder is within the 30 m standoff (`metrics.py`). False alarms are counted per flight hour as *confirmed* tracks that never match a real aircraft |
| Probability of track vs range | AirTrack reports "probability of track of more than 95% up to a range of 700 m" against manned aircraft, and states compliance with ASTM F3442 [2] | **Measures** detection probability by range on rendered frames (`tests/test_perception.py`). Targets here are 0.35 m drones, not general aviation, so ranges are tens of metres, not hundreds |
| Ego-motion compensation | AirTrack aligns frames to cancel the observer's motion before detection, because distant aircraft have low signal-to-noise [2] | **Partly**: the stabilised gimbal removes roll and pitch; detection is single-frame. Multi-frame aligned detection is roadmap |
| Synthetic data | SynthSwarm (2026) shows synthetic training transfers to real air-to-air data at about 87% of in-domain mAP50, and names the synthetic-to-real gap as open [3] | **Declares**: all Kestrel imagery is synthetic. Evaluation on real footage is the first roadmap item |
| Event cameras | SkyEV (ETH Zurich, July 2026): fusing event and RGB cameras gains 10.3 mAP over RGB alone for small fast drones, at 5.3 ms on Jetson Orin [4] | **Out of scope** (RGB only); listed as a sensing upgrade path |

## 2. How the field measures avoidance

| Item | What the source says | Kestrel |
|---|---|---|
| Well clear volume | ASTM F3442 recommends lateral and vertical well-clear boundaries of 2000 ft (610 m) and 250 ft (77 m) [5], for encounters with manned traffic | **Declares** its own: a 12 m sphere for drone-to-drone encounters, for which no standard exists. Stated in `metrics.py` and `docs/safety.md` |
| Loss of well clear (LoWC) | DAA evaluations count losses of well clear over large encounter sets [6] | **Measures** LoWC events and runs with any LoWC, over 24-encounter Monte Carlo sets |
| Risk ratio | DAA performance is judged against the same encounters flown without avoidance | **Measures**: `kestrel bench --no-avoidance` flies the identical encounters with contact handling, yielding and separation filtering off; `bench.risk_ratio()` divides the two |
| Well-clear definition choice | MIT Lincoln Laboratory evaluated four non-cooperative well-clear definitions over one million simulated encounters; modified tau performed best and lowers the required surveillance range [6] | **Roadmap**: Kestrel's buffer is distance-based (with closing speed and uncertainty terms); a tau-based definition is a candidate |
| Operational risk targets | A published SORA-based design uses a loss rate below 10⁻³ per flight hour and 90% detection [7] | **Not claimed**: a few hundred simulated encounters cannot demonstrate rates of that order |

## 3. What the battlefield adds

| Lesson | Source | Kestrel |
|---|---|---|
| Layered detection; never rely on one sensor class. Acoustic nodes are an early-warning and **cueing** layer for cameras and radar, not a replacement | US Army CALL report on acoustic detection in Ukraine [8] | **Implements** a ground acoustic network (`perception/acoustic.py`): triangulated fixes point the camera and widen the keep-out before visual contact. Benchmarked against camera-only and a five-camera ring |
| Acoustic performance | About 2° bearing accuracy per node; small drones heard at "hundreds to a few thousand metres"; about 9,500 nodes at roughly $400–500 each in Sky Fortress [8] | **Uses** 2° bearing noise, 250 m range for a 0.35 m drone, two-node minimum for a fix, 1 s latency. Altitude is treated as unknown |
| Acoustic limits | Wind, urban and industrial noise, quiet electric propulsion and deliberate masking shorten range [8] | **Declares**: none of these are modelled |
| RF-silent drones | Fibre-optic drones (fielded since spring 2024) have no radio signature, so RF detection and jamming do not apply [9] | **Consistent**: Kestrel detects optically and acoustically and does not rely on RF |
| GNSS denial | Ukrainian systems switch to optical navigation when spoofing is detected [10] | **Out of scope**: navigation is the autopilot's job. A GNSS-integrity check is roadmap |
| Mass and autonomy | The IISS expects more autonomy with one mission manager over several air vehicles, and notes the value of detection as early indication even without dissuasion [11] | **Consistent**: one operator instruction drives a fully autonomous mission; multi-vehicle tasking is roadmap |

## 4. What allied programmes ask for

| Need | Source | Kestrel |
|---|---|---|
| A common message standard for counter-UAS sensors, so systems from different nations and companies feed one picture | SAPIENT (Dstl, BSI Flex 335), trialled at NATO's counter-UAS Technical Interoperability Exercise with 17 sensor and 7 decision modules, in NATO standardisation since 2024 [12, 13]; the ECFR calls for common standards and certification with NATO and one command-and-control node fed by every flank detection station [14] | **Implements** a SAPIENT ICD v7 export: registration, status and detection reports from two nodes (airborne cameras, ground acoustic network), UTM coordinates with one-sigma errors (`interop/sapient.py`, `docs/interop.md`). Protobuf wire encoding and a conformance run are not done |
| Birds: radar and optical sensors raise false alarms on birds | ECFR [14]; the acoustic report [8] | **Implements** birds in the simulated sky (intermittent flapping and gliding, corvid-to-gull size) and a wing-beat flicker classifier on the track's brightness history (`perception/classify.py`). **Measures** bird tracks reported as drones with the classifier off and on. Gliding birds cannot be told apart by this cue |
| Several drones at once | ECFR [14] ("multi-shot" defence against simultaneous drones); IISS [11] | **Measures** a two-intruder benchmark set. Engagement is out of scope; Kestrel detects, tracks, keeps clear and reports |
| Sensors that can be tasked by the command node, not only report to it | SAPIENT task definitions, including the `LOOK_AT` command (ICD v7, 5.6.8–5.6.9) [12] | **Implements** a `LOOK_AT` task definition at registration and accepts and acknowledges such tasks, sending the observer to look ([0014](decisions/0014-sensor-node.md)). The Task message's own field names are not yet verified against the `.proto` files |
| Operating under jamming | ECFR: electronic countermeasures dominate the flank; RF-silent fibre-optic drones [9, 14] | **Implements** a bounded "keep working" link policy with store-and-forward reports, and **measures** it against return-on-link-loss on the same jammed encounters |
| Layered detection with a C2 node over it | ECFR [14]; NATO Eastern Sentry counter-drone training and the EU Eastern Flank Watch [15] | **Consistent**: the acoustic layer cues the camera and both report as separate SAPIENT nodes, so a fusion node can weigh them on their own records |

## 5. Roadmap, in order

1. Evaluate the detector on real air-to-air footage (public datasets, e.g. AOT [1]).
2. Multi-frame, ego-motion-compensated detection, as in AirTrack [2].
3. A tau-based well-clear definition compared against the current distance-based one [6].
4. Real flight with a cooperating target drone.
5. GNSS-integrity monitoring and multi-vehicle tasking.
6. SAPIENT protobuf encoding from the Dstl `.proto` files and a run against the SAPIENT test harness.
7. Thermal imagery for night operation; event cameras [4].
8. A Gazebo/ROS 2 bridge so the same autonomy runs against the photoreal simulators used in industry.

## Sources

1. Amazon Science, *New workshop, dataset will help promote safety in the skies* (Airborne Object Tracking dataset and challenge). https://www.amazon.science/blog/new-workshop-dataset-will-help-promote-safety-in-the-skies
2. *AirTrack: Onboard Deep Learning Framework for Long-Range Aircraft Detection and Tracking*, arXiv:2209.12849. https://arxiv.org/abs/2209.12849
3. Zheng et al., *SynthSwarm* (synthetic dataset for long-range UAV swarm detection), J. Eur. Opt. Soc.-Rapid Publ. 22, 49 (2026). https://jeos.edpsciences.org/articles/jeos/full_html/2026/01/jeos20260026/jeos20260026.html
4. *SkyEV: RGB-Event UAV detection and tracking dataset and baseline*, ETH Zurich, July 2026. https://www.alphaxiv.org/abs/2607.18747
5. IMAV 2024 paper citing ASTM F3442/F3442M-20 well-clear boundaries. https://www.imavs.org/papers/2024/26.pdf ; standard: ASTM F3442/F3442M-25, https://store.astm.org/f3442-25.html
6. Wu et al., *Detect-and-avoid closed-loop evaluation of noncooperative well clear definitions*, J. Air Transportation (2020), MIT Lincoln Laboratory. https://www.ll.mit.edu/r-d/publications/detect-and-avoid-closed-loop-evaluation-noncooperative-well-clear-definitions
7. As [5] (SORA-derived figures used in that paper).
8. US Army Center for Army Lessons Learned, *Listening to the sky: acoustic drone detection in Ukraine* (2026). https://api.army.mil/e2/c/downloads/2026/04/30/af450e75/no-26-1115-listening-to-the-sky-acoustic-drone-detection-in-ukraine-mar-26.pdf
9. Wikipedia, *Fiber optic drone*. https://en.wikipedia.org/wiki/Fiber_optic_drone
10. Razom, *Drone that cannot be jammed: how Ukrainian developers taught unmanned vehicles to recognize GPS spoofing*. https://razomua.media/en/news/politika/dron-yakoho-ne-zbyty-hlushinnyam-yak-ukrayinski-rozrobnyky-navchyly-bezpilotnyky-rozpiznavaty-pidminu-gps
11. IISS, *UAVs: ISR, deterrence and war*, Strategic Dossier (March 2026). https://www.iiss.org/globalassets/media-library---content--migration/files/publications---free-files/strategic-dossier/uavs-2026/iiss_uavs-isr-deterrence-and-war_032026.pdf
12. Dstl, *SAPIENT Interface Control Document*, version 7.0 (2023). https://assets.publishing.service.gov.uk/media/6419a2068fa8f547c68029d3/SAPIENT_Interface_Control_Document_v7_FINAL__fixed2_.pdf
13. *SAPIENT becomes NATO standard for counter-drone systems* (June 2026). https://letsdatascience.com/news/sapient-becomes-nato-standard-for-counter-drone-systems-199eb30d
14. ECFR, *Buzzkill: how to defend Europe against drones* (2026). https://ecfr.eu/publication/buzzkill-how-to-defend-europe-against-drones/
15. *NATO conducts counter-drone training mission over Baltics* (Eastern Sentry), Defence Industry Europe. https://defence-industry.eu/nato-conducts-counter-drone-training-mission-over-baltics-to-reinforce-eastern-flank-air-defence-and-deterrence/ ; European Parliamentary Research Service, *Eastern Flank Watch and the European drone wall* (October 2025). https://epthinktank.eu/2025/10/23/eastern-flank-watch-and-european-drone-wall/
