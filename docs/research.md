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
| A common message standard for counter-UAS sensors, so systems from different nations and companies feed one picture | SAPIENT (Dstl, BSI Flex 335), trialled at NATO's counter-UAS Technical Interoperability Exercise with 17 sensor and 7 decision modules, adopted by NATO through STANREC/AEDP-4869 [12, 13, 22]; the ECFR calls for common standards and certification with NATO and one command-and-control node fed by every flank detection station [14] | **Implements** BSI Flex 335 v2.0 messages (registration, status, detection, task, task acknowledgement) from Kestrel, the simulated ground sensors and the fusion node, and **tests** every kind against Dstl's published protobuf schema [23] (`tests/test_sapient_conformance.py`, `docs/interop.md`). A run against a SAPIENT test harness or a third-party fusion node is not done |
| Birds: radar and optical sensors raise false alarms on birds | ECFR [14]; the acoustic report [8] | **Implements** birds in the simulated sky (intermittent flapping and gliding, corvid-to-gull size) and a wing-beat flicker classifier on the track's brightness history (`perception/classify.py`). **Measures** bird tracks reported as drones with the classifier off and on. Gliding birds cannot be told apart by this cue |
| Several drones at once | ECFR [14] ("multi-shot" defence against simultaneous drones); IISS [11] | **Measures** a two-intruder benchmark set. Engagement is out of scope; Kestrel detects, tracks, keeps clear and reports |
| Sensors that can be tasked by the command node, not only report to it | SAPIENT task definitions, including the `LOOK_AT` command (ICD v7, 5.6.8–5.6.9) [12] | **Implements** a `LOOK_AT` task definition at registration and accepts and acknowledges such tasks, sending the observer to look ([0014](decisions/0014-sensor-node.md)). Since v0.4 the Task message, like every message Kestrel writes, is parsed against Dstl's BSI Flex 335 v2.0 protobuf schema in CI ([interop.md](interop.md)) |
| Operating under jamming | ECFR: electronic countermeasures dominate the flank; RF-silent fibre-optic drones [9, 14] | **Implements** a bounded "keep working" link policy with store-and-forward reports, and **measures** it against return-on-link-loss on the same jammed encounters |
| Layered detection with a C2 node over it | ECFR [14]; NATO Eastern Sentry counter-drone training and the EU Eastern Flank Watch [15] | **Consistent**: the acoustic layer cues the camera and both report as separate SAPIENT nodes, so a fusion node can weigh them on their own records |

## 5. Roadmap, in order

1. Deployment and field testing with our systems and defence-technology partners: real sensor data,
   multi-sensor network fusion over SAPIENT, and flights against a cooperating target drone. The fusion
   node already reads recorded SAPIENT traffic (`kestrel fuse`); the NATO counter-UAS data challenge
   data [20] is the first real multi-sensor set to run it on.
2. Night: thermal imagery and a thermal detector, benchmarked by day and night; the CST Anti-UAV
   thermal benchmark [17] as the real-data check.
3. Real-data evaluation of the detector on public air-to-air sets (Det-Fly, Drone-vs-Bird, DUT Anti-UAV
   [18]) and of the acoustic layer on the 24-microphone localisation set [19].
4. Multi-frame, ego-motion-compensated detection, as in AirTrack [2].
5. Authenticated friendly-force tracking in place of Remote ID for *friend*; the SAPIENT `follow` task.
6. A tau-based well-clear definition; GNSS-integrity monitoring.
7. A Gazebo/ROS 2 bridge so the same autonomy runs against the photoreal simulators used in industry.

## 6. 2026: what the front line, the market and NATO ask of the picture

Researched for v0.4 (November 2026). The pattern across all three is the same: sensing and the
picture are the gap, not effectors.

| Finding | Source | Kestrel |
|---|---|---|
| Sensors, not interceptors, are the bottleneck. Interceptor units say "we just need better radar"; missions start from radar tracks, acoustic spotters and the battlefield management system; networked systems are starting to hand tracks between sectors | Defense News, March 2026 [16]; TWZ, May 2026 [24] | **Implements** the fusion node that builds one picture from radar, RF, acoustic, Remote ID and Kestrel, and hands its most uncertain track to the airborne camera (`rap/`, [air-picture.md](air-picture.md)) |
| Every sensor class has a known blind spot: acoustic networks miss electric-motor variants, RF misses fibre-optic and GNSS/INS-guided drones, EO/IR needs cueing and good weather, radar struggles with low RCS and low altitude; multi-sensor fusion is "the only operationally validated approach" | Drone Warfare, April 2026 [28] | **Models** each blind spot (radio-silent drones, quiet propulsion, low-altitude clutter, birds on radar) and **measures** each sensor alone against all fused on the same situations |
| Decoys (Gerbera, Parody) make up about a third of mass raids, to exhaust interceptors and overload detection | Defense News [16] | **Partly**: the picture separates drone from bird and identifies friends; telling a decoy from an armed drone is not modelled |
| Europe's C-UAS demand: $5–7 billion by 2029 (about 58% a year), 4,000–5,000 systems; underserved: civil infrastructure, naval, and cross-border networking of sensors and C2; buyers want low unit cost, open standards, simple training and logistics | BCG, May 2026 [21] | **Consistent**: open SAPIENT interfaces end to end, a fusion node anyone can run on recorded traffic, a parts-cost airborne node |
| EU drone wall (EDDI): initial capability by end-2026, full by end-2027; the sensors, networking and who builds the recognised air picture are still open | European Security & Defence, March 2026 [25] | **Consistent**: the picture is built from heterogeneous SAPIENT nodes, the open question the initiative leaves |
| NATO C-UAS Data Challenge 2026: build one local recognised air picture from radar, RF, EO/IR, acoustic and cyber data from NATO exercises; remove duplicate tracks, fuse kinematics and attributes, classify (drone, bird, type), identify friend, hostile or unknown with white-listed identifiers; registration by 14 December 2026, submission by 21 December 2026, prizes up to €100,000 | NCIA [20] | **Implements** each task in simulation, scored with the same kinds of measure (GOSPA, duplicates, classification, identification). Running it on the challenge data needs registration by an eligible entity |
| SAPIENT v2.0 (BSI Flex 335 v2.0): hierarchical fusion nodes, mobile nodes, run-time taxonomies; the protobuf schema is published; SAPIENT-compliant radar processing and multi-sensor fusion were validated at NATO's LCI-X Crucible exercise in Romania, April 2026 | GOV.UK [22]; Dstl [23]; Defence Online, August 2026 [26] | **Implements** v2.0 messages, tested against the published schema; tasks use `destination_id` and `look_at.location_list` as the schema defines |
| EU direct Remote ID is mandatory for specific-category and class-marked drones since 1 January 2024 | EASA [27] | **Uses** it as cooperative identification, never as proof: *friend* needs an independent sensor to put the drone where it says it is, and a cloned serial is never a friend. Remote ID is not authenticated; that limit is stated |
| Tiny targets are still the hard case: on the CST Anti-UAV thermal benchmark (ICCV 2025 workshop; 220 sequences, 78,224 targets under 10 px, day and night) the best trackers reach about 36% mSA | Xie et al. [17] | **Roadmap** (thermal and real-data evaluation, items 2 and 3) |
| Lightweight detectors on public sets: YOLO11n with context-aware augmentation reaches 90.9% mAP50 on Det-Fly (air-to-air), 68.0% on Drone-vs-Bird | arXiv 2604.19999 [18] | **Roadmap**: Kestrel's detector is classical and untested on real footage |
| A public 24-microphone acoustic localisation set with GPS ground truth (DJI Air 3), released July 2026 | Western Sydney University [19] | **Roadmap**; its terms reserve text-and-data-mining rights, to be checked before use |

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
16. Defense News, *Novel interceptor drones bend air-defense economics in Ukraine's favor* (5 March 2026). https://www.defensenews.com/global/europe/2026/03/05/novel-interceptor-drones-bend-air-defense-economics-in-ukraines-favor/
17. Xie et al., *CST Anti-UAV: A Thermal Infrared Benchmark for Tiny UAV Tracking in Complex Scenes*, ICCV 2025 Workshops; arXiv:2507.23473. https://arxiv.org/html/2507.23473v2
18. *Optimizing Data Augmentation for Real-Time Small UAV Detection: A Lightweight Context-Aware Approach*, arXiv:2604.19999. https://arxiv.org/pdf/2604.19999
19. Rodriguez, Chevtchenko and Afshar, *UAV Acoustic Localization Dataset: 24-Channel Beamformed Recordings of a DJI Air 3 Drone with Synchronized GPS Flight Logs*, Western Sydney University (2026). https://doi.org/10.26183/07nm-4p35
20. NCIA, *NATO C-UAS Data Challenge 2026*, challenge description. https://www.nato.int/content/dam/nato/webready/documents/nfdi/nfdi-C-UAS-DataChallenge.pdf ; event page: https://www.nato.int/en/work-with-us/business-and-project-opportunities/events/2026/12/nato-c-uas-data-challenge-2026
21. BCG, *Countering the drone threat: Europe's C-UAS opportunity* (May 2026). https://www.bcg.com/assets/2026/white-paper-countering-the-drone-threat-europes-cuas-opportunity.pdf
22. GOV.UK, *New milestone as BSI publishes updated SAPIENT standard* (BSI Flex 335 v2.0, March 2024). https://www.gov.uk/government/news/new-milestone-as-bsi-publishes-updated-sapient-standard
23. Dstl, *SAPIENT Proto Files* (BSI Flex 335 v1.0 and v2.0 protobuf, Apache 2.0). https://github.com/dstl/SAPIENT-Proto-Files
24. TWZ, *Inside Ukraine's Interceptor Drone Innovations Swatting Down Thousands Of Russian Shaheds* (May 2026). https://www.twz.com/news-features/inside-ukraines-interceptor-drone-innovations-swatting-down-thousands-of-shahed-drones
25. European Security & Defence, *Europe's drone wall: ready, EDDI, go* (13 March 2026). https://euro-sd.com/2026/03/articles/exclusive/49854/europes-drone-wall-ready-eddi-go/
26. Defence Online, *Cambridge Pixel validates SAPIENT-compliant radar processing at NATO counter-UAS exercise in Romania* (7 August 2026). https://www.defenceonline.co.uk/2026/08/07/cambridge-pixel-validates-sapient-compliant-radar-processing-at-nato-counter-uas-exercise-in-romania/
27. EASA, *Remote identification will become mandatory for drones across Europe*. https://www.easa.europa.eu/en/document-library/general-publications/remote-identification-will-become-mandatory-drones-across
28. Drone Warfare, *Countering the Shahed-136: Detection, Intercept, Cost* (16 April 2026). https://drone-warfare.com/counter-uas/countering-the-shahed-136/
