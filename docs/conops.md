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

COST_SECTION

## Not done

Real flight, real footage, thermal imagery, visual navigation under GNSS denial,
conformance against a SAPIENT fusion node, and launch and recovery from a moving deck.
