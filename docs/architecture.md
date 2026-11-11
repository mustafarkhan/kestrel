# Architecture

The role this architecture serves, and its limits, are in [conops.md](conops.md).

Kestrel is an autonomy stack for a small multirotor that is *told* what to do in
plain language and *sees* other aircraft with a single camera. Everything runs in
one Python process at a fixed rate; the autopilot (simulated, or ArduPilot over
MAVLink) only ever receives position / velocity / acceleration setpoints.

```mermaid
flowchart LR
    subgraph language [Language layer]
        T[operator text] --> E{emergency?}
        E -- "stop / abort / land now" --> X[abort]
        E -- no --> L[LLM planner<br/>Ollama, JSON schema]
        L -- invalid twice / offline --> R[rule parser]
        L --> M[Mission]
        R --> M
        M --> V[static validator<br/>fence, altitude, energy, standoff]
    end
    subgraph perception [Perception, 10 Hz]
        C[camera frame] --> D[sky-target detector]
        D --> K[GNN + EKF tracker<br/>+ bird/drone flicker classifier]
        A[ground microphones, 1 Hz] --> Q[triangulated cues]
    end
    subgraph decide [Autonomy, 20 Hz]
        V --> EX[mission executive<br/>behaviours, contact pre-emption]
        K --> EX
        X --> EX
        EX --> S[runtime safety monitor]
        K --> S
        Q --> EX
        Q --> S
    end
    S --> AP[autopilot<br/>sim or ArduPilot GUIDED]
    K --> SAP[SAPIENT reports<br/>to a C-UAS fusion node]
    Q --> SAP
    AP --> NAV[navigation state] --> EX
    NAV --> S
```

## Layers

| Module | Responsibility | Key idea |
|---|---|---|
| `mission/parser.py` | emergency phrases; rule-based parsing | stop/abort/land never wait for a model |
| `mission/llm.py` | language → `Mission` via local LLM | schema-constrained output, re-validated, one repair round, deterministic fallback |
| `mission/validator.py` | pre-flight checks | rejects with reasons; only structural normalisation |
| `mission/geometry.py` | task → concrete path | the plan that is checked is the plan that is flown |
| `mission/executive.py` | behaviours, pre-emption, resume, aborts | contacts interrupt and the interrupted task resumes from where it was |
| `guidance/trajectory.py` | paths, speed planning, reference filter | curvature-aware speed profile; consistent pos/vel/acc |
| `guidance/observer.py` | standoff observation of a target | look up against sky; orbit for parallax |
| `perception/render.py` | synthetic camera frames | detector runs on pixels, so false alarms are emergent |
| `perception/detector.py` | small dark targets against sky | black top-hat, robust threshold, horizon mask, darker-than-sky test |
| `perception/tracker.py` | multi-target tracking | bearing + log-range EKF, GNN association, M-of-N confirmation |
| `perception/classify.py` | bird or drone | wing-beat flicker in the track's brightness, against sensor noise |
| `perception/acoustic.py` | ground microphone network | bearings triangulated into horizontal fixes; altitude unknown, so a column |
| `interop/sapient.py` | reports to other systems | SAPIENT (BSI Flex 335 v2.0) registration, status, detection, task acknowledgement; UTM coordinates; checked against Dstl's schema |
| `interop/wire.py` | SAPIENT on a real link | protobuf from Dstl's schema, mandatory-field validation, length framing, TCP client and server, mutual TLS 1.3; a node id is bound to its connection |
| `interop/cot.py` | the picture on a TAK display | Cursor on Target events over UDP, identity as the standard affiliation |
| `rap/service.py` | the fusion node as a live service | the same `FusionNode` behind the SAPIENT server; clock and site checks, sensor health, operator declarations |
| `rap/tasking.py` | which track gets the next look | shared by the simulation loop and the live service |
| `live.py` | the real-time flight loop | camera frames, tasks and traffic from the link, link-loss policy, pilot takeover, stale-position and dead-camera handling, flight recorder |
| `preflight.py` | checks before arming | autopilot, navigation, GNSS, site, battery, mission, camera, link, clock, failsafes, disk; a failed check stops the flight |
| `perception/source.py` | where frames come from | an OpenCV camera, or the renderer on the bench |
| `vehicle/bench_vehicle.py` | the simulator behind the real loop | the whole flight software exercised without an aircraft |
| `rap/sensors.py` | ground radar, RF, Remote ID, acoustic | each a SAPIENT node with its real counterpart's blind spots |
| `rap/fusion.py` | the recognised air picture | association, Kalman fusion, duplicate merging, bounded class evidence, standard identities, choosing the next look |
| `rap/loop.py` | the picture in a flight | ground sensors and Kestrel feed it; it tasks Kestrel with `LOOK_AT` |
| `rap/score.py`, `rap/replay.py` | scoring; recorded traffic | GOSPA, duplicates, class and identity against truth; `kestrel fuse` on a SAPIENT log |
| `sim/birds.py` | birds in the simulated sky | intermittent flapping and gliding, wandering paths |
| `team.py` | several observers in one simulation | one fused tracker (triangulation), shared contacts, teammates as cooperative traffic |
| `safety/monitor.py` | run-time assurance | every setpoint filtered; latched RTL/LAND |
| `safety/energy.py` | battery planning model | shared by validator and monitor |
| `control/` | cascaded position/velocity + SE(3) attitude, mixer | what the simulated autopilot runs |
| `vehicle/` | `Vehicle` protocol: simulator or ArduPilot | identical autonomy code for both |

## Frames and clocks

* World frame ENU, metres, origin at home; body FLU. MAVLink NED/FRD is converted
  only in `vehicle/mavlink_vehicle.py`.
* The simulator steps physics at 400 Hz, autonomy at 20 Hz, the camera at 10 Hz.
  Detections are delivered after the configured latency and fused at capture time.
* Frames are rendered from the *true* pose but processed with the *estimated*
  pose, so navigation error turns into bearing error exactly as on an aircraft.
* With a real autopilot, the mission clock is the autopilot's boot time, so the
  loop also behaves correctly under SITL `--speedup`.

## Data contracts

* **`Mission`** (pydantic): the only thing the language layer can produce. Its
  JSON Schema is what the LLM is constrained to.
* **`Setpoint`**: position, velocity and acceleration feed-forward plus yaw; maps
  1:1 onto `SET_POSITION_TARGET_LOCAL_NED`.
* **`VehicleState`**: the navigation solution (an estimate, never truth).
* **`Detection` / `TrackEstimate`**: pixel detections in, world-frame tracks with
  covariance out, plus the probability the track is a drone rather than a bird. A
  learned detector can replace the classical one behind the same `Detection` type.
* **SAPIENT messages**: what leaves the system for a counter-UAS fusion node
  ([interop.md](interop.md)).
