# Field test plan: from bench to first tasked flight

Kestrel has not flown. This is the order in which to change that, with what each stage must
show before the next one starts. A stage that fails is repeated after the fix; no stage is
skipped.

Flying is the operator's responsibility: authorisation for the site and category of
operation, insurance, a safety pilot with the transmitter in hand on every flight, and a
cooperating target drone flown by a second pilot. Nothing here replaces the operator's
own risk assessment. Kestrel only observes and reports: no stage involves closing on an
uncooperative aircraft or acting against one.

## Stage 0: bench, no aircraft (an afternoon)

Everything except the airframe, on the companion computer that will fly.

```bash
deploy/pki.sh pki fusion kestrel-1                       # one CA for the site, a certificate per device
kestrel fusion --site sites/my_site.yaml --listen 0.0.0.0:5020 \
    --tls-ca pki/ca.crt --tls-cert pki/fusion.crt --tls-key pki/fusion.key --out runs/rap.jsonl
kestrel fly "take off to 20 m, survey the north field at 25 m, then return home" \
    --connect bench --camera bench --scenario sites/my_site.yaml --sapient <fusion-host>:5020 \
    --tls-ca pki/ca.crt --tls-cert pki/kestrel-1.crt --tls-key pki/kestrel-1.key --yes
```

| check | pass |
|---|---|
| the site file is surveyed: origin, fence corners, protected areas | fence drawn on a map matches the ground; the autopilot's own fence is at least as tight |
| `kestrel preflight` on the bench vehicle | no check fails; every warning is understood and written down |
| bench mission with the fusion node on a second machine | mission completes; the picture shows the node `OK`; a flight log is written |
| a certificate from a different CA | the fusion node refuses the connection |
| the fusion node is stopped mid-mission | Kestrel applies the link-loss action in the config and says so in the log |
| the real camera in place of `bench` (`--camera 0` or the pipeline) on the ground | preflight reports its size and rate; frames are not black; detections on a clear sky are few |

## Stage 1: camera calibration and recorded footage

On the ground, with the camera that will fly.

1. Intrinsics: checkerboard calibration; put the measured field of view and resolution in
   the config. The detector's bearings are only as good as this.
2. Record the target drone against the sky at measured distances (10 m steps from 20 m to
   100 m, laser rangefinder), by day, and at night for a thermal camera. Label the boxes.
3. `kestrel evaluate` on the recording ([real-data.md](real-data.md)).

| check | pass |
|---|---|
| recall by distance | within reach of the simulated figures (every frame to 45 m, about half at 60 m for a 0.35 m drone), or the config's detection assumptions are replaced by the measured ones and the benchmarks re-run |
| size-to-range | `size_gain` and `size_offset` refitted on the recording; range error within the monitor's 15% floor at 30 to 60 m |
| false detections on empty sky, cloud, treeline | counted and published; if high, the horizon mask and thresholds are retuned before flight |

## Stage 2: hardware in the loop

The real flight controller on the bench, propellers off, running ArduCopter's simulation
on the board, connected to the companion computer by the cable that will fly.

| check | pass |
|---|---|
| `kestrel preflight` | no check fails; the autopilot failsafes and its fence are reported on, not off |
| a full mission | completes; setpoint rate holds on the companion computer with the camera running (CPU headroom recorded) |
| switch out of GUIDED on the transmitter during the mission | Kestrel stands down at once and sends nothing more |
| unplug the companion computer in flight (simulated) | the autopilot's own failsafe returns and lands |
| unplug the camera | Kestrel reports the sensor failed and returns home |

## Stage 3: first flights, no traffic

Open site, light wind, the safety pilot ready to take over. Low and short first.

1. Take-off to 10 m, hover 30 s, land.
2. A 40 m square at 15 m.
3. The survey mission at 25 m.
4. Fence: a mission whose path would cross the fence is refused at planning; a look tasked
   next to the fence is clamped at the fence margin (the intervention is in the log). The
   autopilot's own fence is the backstop.
5. Low battery (threshold raised for the test) and link loss (the radio or the fusion link
   switched off): return and land.

| check | pass |
|---|---|
| position tracking | within 2 m of the planned path in light wind |
| takeover | every takeover by the pilot is clean; the log shows Kestrel silent afterwards |
| failsafes | each one triggers once, as configured, with the reason in the log |
| the simulator's vehicle model | step responses compared with the flight log; the model's mass, drag and rate limits corrected and the benchmarks re-run if they differ |

## Stage 4: perception in flight, target at a distance

The cooperating target flies a fixed pattern at least 60 m away at a different altitude,
its position logged (its own GNSS log is the ground truth). Kestrel flies a plain survey
and the target stays far enough away that no avoidance is expected: this stage measures
what the camera sees, not what Kestrel does about it.

| check | pass |
|---|---|
| detection range and track continuity | measured against the target's GNSS log and compared with the benchmark tables |
| range error | within the 15% floor the separation filter assumes; otherwise the floor is raised |
| false tracks per hour | measured over real cloud, birds and terrain; compared with the simulated rate |
| bird or drone | every bird track logged with its class; a gliding bird reported as a drone is expected and counted |

## Stage 5: separation, vertically offset first

The same encounter geometries as the benchmark (head-on, crossing, overtaking, orbit),
flown slowly, with the target **15 m above** Kestrel's altitude so that a failure of the
separation filter is not a collision. The safety pilot aborts on any doubt.

| check | pass |
|---|---|
| horizontal miss distance | never inside 12 m horizontally in at least 20 encounters; each intervention's reason in the log |
| blind geometries | approaches from outside the camera's view are flown deliberately, with the acoustic network or the fusion node's traffic as the only cue, and the result is published either way |

Only with every Stage 5 encounter passed is the vertical offset reduced, in steps, and
never below what the operator's risk assessment allows.

## Stage 6: the sensor node with real sensors

1. A partner's recorded SAPIENT traffic through `kestrel fuse`, then streamed with
   `kestrel send` to a live fusion node: registration, fusion and health without an aircraft.
2. The partner's sensors live on the site network over mutual TLS; the target flies;
   Kestrel stays on the ground. The picture is compared with the target's GNSS log:
   coverage, duplicates, class, track continuity, the same scores as the benchmark.
3. Kestrel airborne and tasked: the fusion node sends `LOOK_AT`; Kestrel classifies and
   reports; the operator sees the track change on the TAK display. The target is at a
   different altitude throughout, and traffic from the fusion node is on.
4. Link jamming, if the site and its radio licence allow it; otherwise the link is switched
   off: store-and-forward and the two-minute bound are checked against the log.
5. A Remote ID spoof with a second transmitter: a whitelisted serial from a second position
   must not be shown as friend.

| check | pass |
|---|---|
| a third-party sensor registers and is fused | no schema or session errors on either side; rejected messages have a reason |
| first track to *UAV* with tasking | compared with the benchmark's median and 90th percentile |
| own-aircraft recognition | the radar's track of Kestrel is shown as friend, is never tasked against, and is absent from Kestrel's own traffic |
| *hostile* | set only by the operator's `declare`; present in the picture and on the display; never set otherwise |

## What is recorded, every flight

The flight log directory (`--log-dir`), the autopilot's dataflash log, the fusion node's
picture (`--out`), the target's GNSS log, wind, light and cloud, and the commit of the
software that flew. A result without all of them is not published.

## After the trials

The measured numbers replace the simulated ones in the README where they exist, side by
side with the simulation's. A limitation is removed from the README only with the log that
removed it.
