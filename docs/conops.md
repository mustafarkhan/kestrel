# Concept of operations: an airborne sensor node

**Role.** Kestrel is an airborne sensor node for layered counter-drone defence and for
detect-and-avoid: it finds, tracks, identifies and reports. It never engages. It makes
the shared air picture better in the places fixed sensors do not reach, and hands every
decision about what to do with a contact to the people and systems responsible for it.

## Where it fits

Counter-drone defence is built in layers: detect, track, identify, decide, defeat.
Fixed radars, cameras, radio-frequency detectors and microphone grids carry most of
the first three. Their gaps are well known:

* **Low altitude and terrain**: small drones fly below radar coverage and behind trees,
  buildings and quays.
* **Identification**: a radar return or an acoustic bearing says *something is there*,
  not *what it is*. Birds are the classic false alarm.
* **Altitude**: acoustic networks, the cheapest wide-area layer, give no height.
* **Temporary sites**: events, convoys, ports of call, exercises, where nothing is
  installed.

A small drone that can be sent to a reported position, confirm and classify the contact,
keep it in view, and pass a clean three-dimensional track to the command node fills
exactly those gaps. That is Kestrel's job.

## The loop

```text
 ground sensors ──cue──▶ fusion / C2 node ──LOOK_AT task──▶ Kestrel
      ▲                        ▲                              │
      │                        └────── SAPIENT reports ◀──────┤ detect, classify,
      └── (also cue Kestrel directly) ────────────────────────┘ track, keep clear
```

1. **Stand by.** Kestrel waits on station near the protected site, camera scanning
   (`stand by at the berth for 30 minutes`).
2. **Cue.** A ground sensor reports something, either directly (acoustic fixes) or as a
   SAPIENT `LOOK_AT` task from the command node, or the operator says
   `investigate (40, 90)`.
3. **Investigate.** Kestrel closes to a standoff from the reported position at its
   working altitude, camera tilted up so a target above is seen against sky.
4. **Confirm and classify.** A camera track near the report confirms it (an aircraft is
   there); the bird/drone classifier adds the class as evidence accumulates.
5. **Report and follow.** Detections go to the fusion node as SAPIENT detection reports
   with position, velocity, one-sigma errors and class. If tasked, Kestrel follows the
   contact at a standoff so the picture stays current.
6. **Return to station.** Then home when the standby period or the battery ends.

Every step runs inside the same safety monitor: separation from every contact (birds
included), geofence, altitude band and energy are enforced on board whatever the
mission or the tasking says.

## Across the domains

| domain | what Kestrel contributes | what it depends on |
|---|---|---|
| **Air** | confirms and classifies contacts other sensors only half-see; keeps them in view; triangulated range with a second observer | line of sight; daylight optics (no thermal yet) |
| **Land and infrastructure** | fills gaps between fixed sites; covers temporary sites | a station to stand by at; battery endurance |
| **Maritime** | the same loop over water (harbours, berthed ships, offshore installations); sea rendered as the background below the horizon | shore or deck launch; sea-state effects on flight are not modelled |
| **Electromagnetic** | keeps working when the ground link is jammed (configurable): continues the task on board for a bounded time, buffers reports and sends them with their original timestamps when the link returns | navigation still needs GNSS; visual navigation is roadmap |
| **Information** | SAPIENT ICD v7 out (registration, status, detections) and in (`LOOK_AT` tasks, acknowledged) | conformance testing against a real fusion node is not done |
| **Space** | nothing directly; GNSS and satellite backhaul are dependencies to design around | |

## Rules it keeps

* **No engagement.** No effector, no target designation for strikes. Reports describe
  airspace for its protection.
* **Every contact is kept clear of.** The classifier changes what is reported and
  followed, never what is avoided.
* **A jammed link is bounded autonomy, not unbounded.** With `link_loss_action:
  continue`, the observer keeps its task for at most `link_loss_autonomy_s` (2 minutes by
  default) and then returns home on its own. With the civil default, `rtl`, it returns
  after 5 s.
* **Bounded giving way.** Two minutes per mission, 30 s on the way home; separation
  itself is never bounded.

## Measured

See the sensor-node table in the README and `docs/results/matrix.json`. Cost and
coverage are below.

## Cost and coverage

**Parts for one node** (retail listings and launch announcements, October 2026; a parts
cost, not a product price: no engineering, certification, ground station or support):

| item | example | price |
|---|---|---|
| airframe, autopilot, GPS, telemetry, motors, ESCs | Holybro X500 V2 PX4 development kit (Pixhawk 6C or 6X) [1] | £645–785 incl. VAT |
| companion computer | NVIDIA Jetson Orin Nano Super developer kit, 67 TOPS [2] | $249 (launch price) |
| camera | Raspberry Pi High Quality Camera, 12.3 MP [3] | $50 |
| battery, lens, gimbal, mounts | estimate, not sourced | about €300 |
| **total** | | **about €1,350–1,500** |

A ground acoustic node of the kind fielded in Ukraine costs about $400–500 [4]; the
nine-node grid in the benchmark would be about $4,000.

**For scale.** The European Council on Foreign Relations puts a Geran-type drone at
about €10,000, and missiles fired at such drones at several hundred thousand to a
million euros each [5]. A sensor node replaces no interceptor. What it changes is the
question asked before one is used: *is that a drone, and where exactly is it?*

**Coverage from one station.** In the benchmark (see the README), a node standing by at
the hangar with the microphone grid tracked every cued intruder across the
300 m × 300 m area, a median 23 s after the first report and within 26 s in 90% of runs,
launch included; without the microphones it tracked 17 of 24, with a 90th percentile
of 145 s after the intruder crossed the fence. Coverage scales with the cueing layer,
not with more cameras.

Sources:
1. Flying Tech, *Holybro X500 V2 Quadcopter PX4 Development Kit*. https://www.flyingtech.co.uk/product/holybro-x500-v2-quadcopter-px4-development-kit/
2. Phoronix, *NVIDIA Launches $249 "Gen AI Supercomputer" With Jetson Orin Nano Super Developer Kit*. https://www.phoronix.com/news/NVIDIA-Jetson-Orin-Nano-Super
3. Digital Camera World, *Raspberry Pi launches 12.3MP camera module for just $50*. https://www.digitalcameraworld.com/uk/news/raspberry-pi-launches-123mp-camera-module-for-just-dollar50
4. US Army Center for Army Lessons Learned, *Listening to the sky* (2026), see docs/research.md [8].
5. ECFR, *Buzzkill: how to defend Europe against drones* (2026). https://ecfr.eu/publication/buzzkill-how-to-defend-europe-against-drones/

## Not done

Real flight, real footage, thermal imagery, visual navigation under GNSS denial,
conformance against a SAPIENT fusion node, and launch and recovery from a moving deck.
