# The recognised air picture

Kestrel on its own is one sensor node. A counter-drone picture is built from many: radar,
radio-frequency direction finders, microphones, cooperative identification and cameras, each
blind to something the others see. This part of Kestrel builds that picture in simulation
with every sensor speaking SAPIENT, fuses it into one recognised air picture (RAP), and lets
the picture task the airborne node to look where the ground sensors cannot decide.

```text
 radar ──┐
 RF DF ──┤  SAPIENT    ┌──────────────── fusion node ────────────────┐
 Remote ID├──reports──▶│ associate → fuse → de-duplicate → classify   │──▶ picture (JSON lines)
 acoustic ┤            │ → identify (friend / neutral / suspect /     │
 Kestrel ─┘            │   unknown; hostile only by an operator)      │
    ▲                  └──────────── most needed look ────────────────┘
    └────────────── SAPIENT LOOK_AT task (same link as every other tasking) ◀──┘
```

Why this, and why now, is in [research.md](research.md#6-2026-what-the-front-line-the-market-and-nato-ask-of-the-picture):
the front line reports that sensing and cueing, not interceptors, are the bottleneck and that
only multi-sensor fusion works in practice; the EU's drone wall leaves open who builds the
picture; and NATO's 2026 counter-UAS data challenge asks for exactly this fusion.

## The sensors, and what each one misses

All parameters are in `SensorNetConfig` (`src/kestrel/config.py`); the models are in
`src/kestrel/rap/sensors.py`.

| sensor | sees | misses | model |
|---|---|---|---|
| **Radar** (mast, 1 s scan) | drones and birds alike | low targets beyond 400 m (clutter, trees, buildings); targets below 0.5° elevation; tracks break after 3 missed scans and restart with a new id; raises clutter tracks (30 per hour) | radar equation with a Swerling I target: SNR ∝ RCS / R⁴, 13 dB for 0.01 m² at 1.5 km, P(detect) = Pfa^(1/(1+SNR)); 3 m range, 0.5° azimuth, 1° elevation noise; m-of-n (2 of 3) track start |
| **Radar micro-Doppler** | rotors look more drone-like than wing-beats | gliding birds; fixed-wing props | a noisy per-scan score (σ 0.15) around 0.7 for multirotors, 0.55 for fixed-wing, 0.3 for flapping and 0.5 for gliding birds |
| **RF direction finders** | drones with a control or video link | fibre-optic and autonomous (GNSS/INS) drones: no emissions to hear | 3° bearings from each node, triangulated as the acoustic network is; an emitter keeps one id (its fingerprint) |
| **Remote ID receiver** | what a compliant drone *says* about itself (EU direct Remote ID, mandatory since 1 January 2024) | anything that does not broadcast; a spoofer's false position | GNSS-grade position (3 m horizontal, 5 m vertical), velocity, and the serial |
| **Acoustic grid** | engines far (×2 range), electric multirotors near (400 m), quiet ones barely (×0.4) | altitude; anything quiet or masked | as Kestrel's own network: 2° bearings, triangulated fixes, no tracks |
| **Kestrel** | what its camera sees, with its wing-beat classifier | everything out of view; gliding birds look like drones to it | the full closed-loop simulation |

Birds have radar cross-sections drawn between 0.003 and 0.02 m²: as large as a small drone's,
which is why radar alone raises bird alarms.

## The fusion node

`src/kestrel/rap/fusion.py`. It reads SAPIENT messages only, so the same code runs on recorded
traffic (`kestrel fuse`).

* **Association.** A report from a tracking sensor (radar, RF, Remote ID, camera) first follows
  its own object id to the picture track it fed before. Otherwise it joins the nearest track
  within a 99% chi-square gate on position, using the report's stated one-sigma errors. A
  track never takes two live object ids from the same tracking sensor: the sensor says they
  are two objects.
* **Kinematics.** A constant-velocity Kalman filter (3 m/s² process noise) with each report's
  errors as measurement noise. Acoustic and RF fixes carry no altitude; they are sent at the
  middle of the altitude band with a vertical error spanning it, so they constrain only the
  horizontal position. The fused covariance is tested for consistency: on a radar-tracked
  drone the mean normalised estimation error squared is 2.1–2.4 against 3 expected, so the
  stated errors are slightly conservative.
* **Duplicates.** Two tracks that stay inside each other's 95% gate for 2 s, and that no
  tracking sensor says are different objects, are merged into the older one.
* **Class.** Drone-versus-bird evidence is a log-likelihood ratio. Each sensor node's opinion
  is the mean log-odds of its reports, weighted by trust (radar 0.5, acoustic 0.6, Kestrel's
  camera 0.6, RF 1.0) and credited with at most three independent looks: one sensor's reports
  on one object are correlated, so a sensor that talks a lot cannot become certain on its
  own. A Remote ID broadcast makes the class *UAV*. UAV above p = 0.8, bird below 0.2,
  unknown between.
* **Identity** (NATO standard identities, APP-6):
  * *friend*: Remote ID serial on our whitelist **and** a precise non-cooperative sensor
    (horizontal error under 30 m) puts the aircraft within the 95% gate of the broadcast
    position in the last 5 s;
  * *assumed friend*: a whitelisted serial nobody has corroborated;
  * *suspect*: an unidentified drone inside, or within 60 s of, a protected area;
  * a **cloned serial** (one whitelisted serial broadcast from two places at once): Remote ID
    cannot say which copy is genuine, so none is shown as friend; a corroborated copy is
    assumed friend and the others are suspect, until an operator decides;
  * *neutral*: a compliant civil drone (non-whitelisted serial) or a bird;
  * *unknown*: everything else; *pending* until a track is confirmed;
  * ***hostile is never set automatically***. It is an operator's decision, recorded with
    `FusionNode.declare`.
* **Tasking.** Every second the node picks the confirmed track that most needs a closer look:
  not friend or neutral, class not settled (or suspect), not seen by a camera in the last
  15 s, inside the area the airborne node may fly; the one that would reach a protected area
  soonest goes first. It sends Kestrel's camera node a SAPIENT `LOOK_AT` task with the track's
  position and errors, and re-sends the newer position every 3 s so the look follows a moving
  aircraft. A look ends when Kestrel's camera has reported the track and the picture can say
  what it is, when the track is gone, or after 75 s.

What Kestrel does with a task: a location with a measured altitude (z error under 15 m, so a
radar or fused track) is approached to 30 m, the follow distance, so the camera can classify
it; a column (an acoustic or RF fix) keeps the 45 m standoff. A re-sent position for the look
under way updates it; a newer task replaces one still waiting. Every safety limit applies
throughout.

## Scoring

`src/kestrel/rap/score.py`, against ground truth every 0.5 s, inside the protected area:

* **GOSPA** (Rahmathullah, García-Fernández and Svensson, 2017), cut-off 25 m, p = 2,
  α = 2: localisation error of matched tracks plus a fixed cost for every missed object and
  every false track. Horizontal distance, because fixes declare no altitude.
* **Coverage**: share of drone-seconds with a matching picture track (all drones; hostile
  drones separately).
* **Duplicates**: extra picture tracks on one object, per object-second.
* **Track identities per drone**: 1 is unbroken.
* **Classification**: share of matched drone-seconds shown as UAV, and as bird (the dangerous
  error); share of matched bird-seconds shown as UAV (the false alarm).
* **Identification**: share of friendly-drone seconds shown as friend; seconds a hostile was
  shown as friend; seconds a friend was shown where no friendly drone is (a phantom).
* **Track to UAV**: for hostile drones, time from first picture track to first shown as UAV.

## Results

Filled in from `docs/results/picture/` and `docs/results/loop/`; see the README tables.

## Limits

* The sensor models are deliberately simple and their parameters are typical published
  values, not a particular product's. Terrain masking is a range-and-height rule, not a
  terrain model. Radar micro-Doppler is a score, not a spectrum.
* Track reports from a sensor's own tracker are fused as if independent; the per-sensor
  class evidence is bounded for that reason, the kinematics are not (the consistency test
  passes with these sensors, which report plots rather than smoothed tracks).
* Two drones flying in close formation can be merged.
* **Remote ID is not authenticated.** A hostile broadcasting one of our serials from its own
  true position, while our drone with that serial is not flying, is corroborated by radar and
  shown as friend. The clone rule only helps while ours is also up. A fielded system needs
  authenticated friendly-force tracking from its own ground control, not a broadcast anyone
  can copy.
* No real sensor data yet. `kestrel fuse` is the entry point for it.
