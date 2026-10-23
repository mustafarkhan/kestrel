# Safety case (simulation scope)

The design question is not "will the LLM behave?" but "what happens when it does
not?". Kestrel answers with three independent barriers, each simple enough to
test exhaustively, so that no single component, learned or not, can make the
aircraft do something unsafe.

| Barrier | When | What it guarantees | Where it is tested |
|---|---|---|---|
| **1. Schema** | planning | the language layer can only express tasks from a fixed vocabulary; anything else fails validation | `tests/test_mission.py` (invalid, invented sites, garbage) |
| **2. Static validation** | before takeoff | every path stays inside the geofence margin, altitudes in band, speeds within limits, standoff above minimum separation, energy within the usable battery | `tests/test_mission.py::test_validator_rejects_with_reason` |
| **3. Runtime monitor** | every tick | setpoints filtered against fence, altitude, kinematic limits and separation; latched return or landing on energy, link loss, or breach | `tests/test_safety.py` (property-based), `tests/test_closed_loop.py` |

Emergency commands (`stop`, `abort`, `land now`) are matched by regular
expressions before any model runs, so they carry no model latency and cannot be
misparsed.

## Separation from non-cooperative aircraft

The monitor keeps every setpoint outside a sphere around each confirmed track,
and inside that sphere commands an escape velocity (horizontal where possible)
at least as fast as the intruder's speed towards the observer:

```
need = min_separation
     + max( k * sigma_eff,  v_intruder_max * time_since_seen )
     + v_close^2 / (2 a_max) + v_close * t_reaction
```

* `sigma_eff` is the track's position uncertainty, floored at a fraction of range:
  monocular range comes from an assumed target size, so it cannot be better than
  that assumption however confident the filter is.
* The **reachable-set** term grows while a track is not being observed: an aircraft
  you cannot see may have flown towards you at top speed.
* The **stopping-distance** term scales with the current closing speed: a static
  buffer is useless when two aircraft close at 8 m/s.

Each term was added because a simulated encounter violated separation without it
(see [decisions/0004-separation.md](decisions/0004-separation.md)).

**Every contact is kept clear of, whatever it is.** Birds get the same buffer as
drones: a bird is a collision hazard too, and a drone the classifier mistakes for a
bird must not lose protection. The classifier changes what is *reported* and
*followed*, never what is *avoided*. Acoustic cues the camera cannot yet confirm are
kept clear of as vertical columns, because sound gives no altitude.

**A lost link is bounded.** By default the observer returns home 5 s after the ground link
is lost. The sensor-node policy (`link_loss_action: continue`) keeps the task going on
board for at most `link_loss_autonomy_s` (2 minutes), then returns home on its own; every
on-board limit (separation, geofence, altitude, energy) applies throughout. Commands and
ground-sensor cues do not arrive while the link is down.

**Teammates are traffic.** With several observers, each keeps the others' datalink
positions clear like any aircraft, and the safety monitor keeps clear of every fused
track, including ones labelled friendly, because a label can be wrong.

**Giving way is bounded; separation is not.** The executive gives way to traffic
crossing its planned path, for at most two minutes per mission and 30 s on the way
home, so persistent traffic cannot hold the aircraft out until the battery decides.
The monitor's separation filter does not depend on either budget.

## What is *not* covered

Stated plainly, because these are the limits an operator must know:

* **Unseen aircraft.** In the default configuration there is one forward camera. An
  aircraft that never enters its field of view cannot be avoided by sight; the
  acoustic network and the five-camera ring configuration are measured as mitigations. The executive mitigates this by keeping the
  camera on the last known contact while nearby, but an unknown aircraft arriving
  from behind is not detected. A rear camera, radar, or a Remote ID / ADS-B feed
  would close this gap.
* **Bright targets.** The detector assumes a backlit (darker-than-sky) target. A
  sunlit target brighter than the sky is missed.
* **Below the horizon.** Targets against ground clutter are excluded by design.
* **Encounter geometry.** If an intruder is first detected at short range while
  both aircraft close fast, there may not be room to stop. The benchmark reports
  these cases; it does not hide them.
* **Intruder model.** Simulated intruders fly figure-eights, orbits and waypoint
  loops at 2-6 m/s with accelerations up to 3 m/s^2, one or two at a time. Faster or
  more aggressive aircraft, and larger groups, are not evaluated.
* **Birds.** Gliding birds look like drones to the flicker classifier and are
  reported as such; birds are modelled as medium-sized (0.5-1 m span).
* **Simulation only.** The flight dynamics, camera and target models are
  simplified. The MAVLink path is tested against ArduCopter SITL, not a real
  airframe. Nothing here is flight-qualified.
