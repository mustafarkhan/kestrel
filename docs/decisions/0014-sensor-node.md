# 0014: From observer to sensor node

## Why

Feedback on v0.2 asked what Kestrel is *for*. The answer chosen is the one the European
analyses of the drone threat point to: fixed sensors leave gaps (low altitude, terrain,
identification, altitude from sound, temporary sites), and a small drone that is sent
to a reported position, confirms and classifies the contact and reports it into the
shared picture fills them. The role and its limits are in [conops.md](../conops.md).

## What was built

* **Standby and investigate.** A `standby` task (wait on station, camera scanning) and
  `on_cue: investigate`: a ground-sensor report, an operator's `investigate (x, y)` or a
  SAPIENT `LOOK_AT` task sends the observer to a standoff from the reported position.
  A camera track near the report confirms it; the classifier adds the class.
* **C2 tasking over SAPIENT.** The camera node registers a `LOOK_AT` task definition and
  acknowledges tasks it receives.
* **Jammed links.** `link_loss_action: continue` keeps the task going on board for a
  bounded time, holds SAPIENT reports and sends them with their original timestamps
  when the link returns. Cues and operator commands, which come over the same link, do
  not arrive while it is down.
* **Two observers.** Both cameras feed one tracker at the fusion node; contacts and
  investigations are shared; teammates are cooperative traffic for the safety monitor.
* **Sea.** A sea surface below the horizon and a harbour scenario.

## Faults found on the way

| fault | effect | fix |
|---|---|---|
| The investigation goal took its altitude from the current position, and the reference filter carried the takeoff climb into it | the observer climbed to 43 m while investigating, so a target at 30 m was below the horizon (not searched) and never confirmed | hold the mission altitude for the whole investigation |
| The standby clock ran only while on station, and every resume met a fresh cue | an aircraft that was never seen kept the observer investigating until the time limit, without landing | the standby period runs from its start, excursions included; no new investigation once a task's time is up |
| Confirmation required the classifier to call the track a drone | with the classifier on, a drone at the 45 m standoff was often too faint for an opinion, and confirmation waited for the follow at 30 m (168 s instead of 46 s in a test encounter) | confirmation means *an aircraft is there* (any track not called a bird); *it is a drone* is reported separately as the evidence arrives, and both times are measured |
| A teammate's track was matched as friendly only by position | its range, estimated from a drone-sized assumption, was wrong by the size ratio, so teammates were tracked as intruders and given way to | also match by bearing from another observer; the safety monitor keeps clear of every fused track whatever its label, since a label can be wrong |

## Results

The sensor-node table in the README (seeds 6000–6023, never used while developing).
