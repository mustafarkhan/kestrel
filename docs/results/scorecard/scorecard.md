# Air-picture scorecard: Kestrel fusion node, simulated ground sensors

Threat library 1.0, seeds 9000 to 9004 (5 per threat). Measures and file formats: `docs/scorecard.md`. Simulated sensors and simulated sky.

## Mixed raid (`mixed-raid`)

One radio-controlled, one engine-driven and one silent low drone at once, among patrols and birds.

| measure | better | result |
|---|---|---|
| **hostile drone-time tracked** | higher | 76% |
| entry to first track (median) | lower | 0 s |
| drones never tracked | lower | 0 |
| **tracked drone-time shown as UAV** | higher | 95% |
| tracked drone-time shown as bird | lower | 0% |
| first track to UAV, hostile (median) | lower | 1 s |
| **bird-time shown as UAV** | lower | 2% |
| false tracks at a time (mean) | lower | 0.28 |
| duplicate tracks per object | lower | 0% |
| track identities per drone | lower | 1.5 |
| friend-time shown as friend | higher | 100% |
| hostile shown as friend | lower | 0 s |
| friend shown where none is | lower | 0 s |
| GOSPA | lower | 31.4 m |

## Silent and low (`silent-low`)

Four quiet drones with no radio link, 8 to 20 m up, from all sides: nothing for RF or acoustic sensors to hear, little for a radar to see.

| measure | better | result |
|---|---|---|
| **hostile drone-time tracked** | higher | 65% |
| **entry to first track (median)** | lower | 46 s |
| **drones never tracked** | lower | 0 |
| tracked drone-time shown as UAV | higher | 81% |
| tracked drone-time shown as bird | lower | 0% |
| first track to UAV, hostile (median) | lower | 31 s |
| bird-time shown as UAV | lower | 0% |
| false tracks at a time (mean) | lower | 0.31 |
| duplicate tracks per object | lower | 0% |
| track identities per drone | lower | 2.11 |
| friend-time shown as friend | higher | 100% |
| hostile shown as friend | lower | 0 s |
| friend shown where none is | lower | 0 s |
| GOSPA | lower | 34.5 m |

## One-way attack (`one-way-attack`)

Three fast fixed-wing drones straight at the protected site: how early they are tracked and called drones.

| measure | better | result |
|---|---|---|
| **hostile drone-time tracked** | higher | 100% |
| **entry to first track (median)** | lower | 0 s |
| drones never tracked | lower | 0 |
| tracked drone-time shown as UAV | higher | 100% |
| tracked drone-time shown as bird | lower | 0% |
| **first track to UAV, hostile (median)** | lower | 0 s |
| bird-time shown as UAV | lower | 0% |
| false tracks at a time (mean) | lower | 0.29 |
| duplicate tracks per object | lower | 0% |
| track identities per drone | lower | 1.1 |
| friend-time shown as friend | higher | 100% |
| hostile shown as friend | lower | 0 s |
| friend shown where none is | lower | 0 s |
| GOSPA | lower | 28.9 m |

## Saturation (`saturation`)

Twelve drones from one sector within a minute: track capacity, duplicates and track continuity.

| measure | better | result |
|---|---|---|
| **hostile drone-time tracked** | higher | 100% |
| entry to first track (median) | lower | 0 s |
| drones never tracked | lower | 0 |
| tracked drone-time shown as UAV | higher | 100% |
| tracked drone-time shown as bird | lower | 0% |
| first track to UAV, hostile (median) | lower | 0 s |
| bird-time shown as UAV | lower | 4% |
| false tracks at a time (mean) | lower | 0.3 |
| **duplicate tracks per object** | lower | 0% |
| **track identities per drone** | lower | 1.47 |
| friend-time shown as friend | higher | 100% |
| hostile shown as friend | lower | 0 s |
| friend shown where none is | lower | 0 s |
| GOSPA | lower | 31.2 m |

## Decoys (`decoys`)

Eight radar-bright decoys on one side while two silent low drones come from the other.

| measure | better | result |
|---|---|---|
| **hostile drone-time tracked** | higher | 77% |
| entry to first track (median) | lower | 0 s |
| **drones never tracked** | lower | 0 |
| **tracked drone-time shown as UAV** | higher | 96% |
| tracked drone-time shown as bird | lower | 0% |
| first track to UAV, hostile (median) | lower | 0 s |
| bird-time shown as UAV | lower | 0% |
| false tracks at a time (mean) | lower | 0.27 |
| duplicate tracks per object | lower | 0% |
| track identities per drone | lower | 1.46 |
| friend-time shown as friend | higher | 100% |
| hostile shown as friend | lower | 0 s |
| friend shown where none is | lower | 0 s |
| GOSPA | lower | 32.9 m |

## Remote ID spoof (`remote-id-spoof`)

A hostile broadcasts a friendly serial with a false position.

| measure | better | result |
|---|---|---|
| hostile drone-time tracked | higher | 84% |
| entry to first track (median) | lower | 0 s |
| drones never tracked | lower | 0 |
| tracked drone-time shown as UAV | higher | 97% |
| tracked drone-time shown as bird | lower | 0% |
| first track to UAV, hostile (median) | lower | 0 s |
| bird-time shown as UAV | lower | 2% |
| false tracks at a time (mean) | lower | 1.04 |
| duplicate tracks per object | lower | 0% |
| track identities per drone | lower | 1.69 |
| **friend-time shown as friend** | higher | 57% |
| **hostile shown as friend** | lower | 0 s |
| **friend shown where none is** | lower | 6 s |
| GOSPA | lower | 35.2 m |

## Remote ID impersonation (`remote-id-impersonation`)

A hostile broadcasts a grounded friend's serial from its own true position.

| measure | better | result |
|---|---|---|
| hostile drone-time tracked | higher | 100% |
| entry to first track (median) | lower | 0 s |
| drones never tracked | lower | 0 |
| tracked drone-time shown as UAV | higher | 100% |
| tracked drone-time shown as bird | lower | 0% |
| first track to UAV, hostile (median) | lower | 0 s |
| bird-time shown as UAV | lower | 0% |
| false tracks at a time (mean) | lower | 0.27 |
| duplicate tracks per object | lower | 0% |
| track identities per drone | lower | 1.2 |
| **friend-time shown as friend** | higher | 100% |
| **hostile shown as friend** | lower | 2335 s |
| friend shown where none is | lower | 2320 s |
| GOSPA | lower | 28.8 m |

## Birds only (`birds-only`)

Forty birds and no hostile drone: every drone alarm is false.

| measure | better | result |
|---|---|---|
| entry to first track (median) | lower | 0 s |
| drones never tracked | lower | 0 |
| tracked drone-time shown as UAV | higher | 100% |
| tracked drone-time shown as bird | lower | 0% |
| **bird-time shown as UAV** | lower | 2% |
| **false tracks at a time (mean)** | lower | 0.61 |
| duplicate tracks per object | lower | 0% |
| track identities per drone | lower | 1.47 |
| friend-time shown as friend | higher | 100% |
| hostile shown as friend | lower | 0 s |
| friend shown where none is | lower | 0 s |
| GOSPA | lower | 49.9 m |

## Radar outage (`radar-outage`)

The mixed raid with the radar silent for 200 s: what the other sensors hold, and how the picture recovers.

| measure | better | result |
|---|---|---|
| **hostile drone-time tracked** | higher | 69% |
| entry to first track (median) | lower | 1 s |
| drones never tracked | lower | 0 |
| **tracked drone-time shown as UAV** | higher | 96% |
| tracked drone-time shown as bird | lower | 0% |
| first track to UAV, hostile (median) | lower | 0 s |
| bird-time shown as UAV | lower | 0% |
| false tracks at a time (mean) | lower | 0.28 |
| duplicate tracks per object | lower | 0% |
| **track identities per drone** | lower | 1.6 |
| friend-time shown as friend | higher | 100% |
| hostile shown as friend | lower | 0 s |
| friend shown where none is | lower | 0 s |
| GOSPA | lower | 38.1 m |

Measures in bold are the ones the threat is aimed at.
