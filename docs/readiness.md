# Readiness: what is closed, what is open, and who can close it

Kestrel is software. v0.6 takes it from "runs in simulation" to "ready to be integrated and
tested with real sensors and a real airframe". It is **not** a fielded or certified product,
and no amount of software makes it one: the open items below need hardware, flights, data
or an authority. This page is the register of both lists, so nobody has to guess which
claims are measured and which are still to be earned.

Maturity, stated the way a buyer's engineer would: components and their integration are
validated in simulation and against an autopilot in the loop (ArduCopter SITL in CI),
which is technology readiness level 4. Level 5 needs the same on representative hardware;
level 6 needs it flying in a relevant environment.

## Closed in v0.6

Each row is a way the v0.5 code would have failed outside simulation. "Checked by" names
the test that fails if it comes back.

| # | Gap found by the audit | What was done | Checked by |
|---|---|---|---|
| 1 | SAPIENT messages existed only as JSON; no real fusion node or sensor could be connected | protobuf encoding from Dstl's schema, 4-byte length framing, TCP client (node side) and server (fusion side) | `test_wire.py` |
| 2 | **Messages were not conformant.** Protobuf accepts a message without the fields the standard marks mandatory; a real receiver rejects it. Registrations lacked `capabilities` and `config_data`, ground-sensor modes lacked `task`, status reports lacked `info` | a validator reads the schema's `is_mandatory` options; every message Kestrel writes is checked; the four omissions fixed | `test_wire.py::test_messages_carry_the_mandatory_fields_of_the_standard`, `test_sapient_conformance.py` |
| 3 | The fusion node ran only inside the simulator or on a finished log | `kestrel fusion`: a live service with the same `FusionNode` the benchmarks measure | `test_fusion_service.py` (real TCP, compared against the direct result) |
| 4 | Anyone who could reach the port could inject tracks, impersonate a sensor, or task the aircraft | mutual TLS 1.3 against a private CA on both endpoints; a non-local listener or link is refused without it unless `--insecure` is given; a node id is bound to the connection that registered it | `test_wire.py::test_mutual_tls_admits_only_certificates_from_our_ca`, `::test_the_server_refuses_what_would_corrupt_the_picture` |
| 5 | A malformed or hostile peer could crash or exhaust the service | frame size limit before allocation, undecodable payloads close only that connection, connection cap, bounded queues | `test_wire.py` (framing fuzzed with Hypothesis) |
| 6 | Replayed or badly timed reports would be fused as current | reports more than 2 s off the fusion clock are dropped and counted; positions outside the site are dropped | `test_fusion_service.py` |
| 7 | A dead sensor looked the same as an empty sky | per-node health in every picture: `OK`, `STALE`, `DISCONNECTED` | `test_fusion_service.py` |
| 8 | **The ground sensors never saw Kestrel**, so nothing tested what happens when the radar tracks our own aircraft: it would have been tasked against itself and told to avoid itself | Kestrel is now in the ground sensors' truth; the fusion node recognises its track from the node's own status reports, shows it as friend, never tasks against it, and leaves it out of that node's traffic. Loop benchmark re-flown | `test_air_picture.py::test_the_fusion_node_recognises_its_own_airborne_node` |
| 9 | The picture's traffic reached Kestrel by a function call | sent as SAPIENT detection reports over the link and decoded by the node, in simulation and live | `test_air_picture.py`, `test_fusion_service.py`, `test_live.py` |
| 10 | **The real flight loop had no camera, no SAPIENT, no link monitoring and no log**: `kestrel fly` flew a mission blind and deaf. Everything measured in simulation was absent from the code that would fly | `live.run_live` now runs the detector and tracker on camera frames, reports and takes tasks over the fusion link, applies the link-loss policy, and records the flight | `test_live.py` (the real loop on a bench vehicle), `test_preflight.py` (from the command line, two processes) |
| 11 | The loop would fight a safety pilot for control | leaving GUIDED while armed makes the loop stand down at once and send nothing more | `test_live.py::test_a_pilot_taking_control_ends_the_mission_loop` |
| 12 | A silent autopilot kept receiving setpoints computed from a stale position | no fresh position: nothing is sent; after 10 s one landing request | `test_live.py::test_a_silent_autopilot_gets_no_setpoints_then_one_landing_request` |
| 13 | A failed camera would have been read as "no traffic" | no frames for 2 s in the air: `SYSTEM_ERROR` to the fusion node and return home | `test_live.py::test_a_dead_camera_is_a_failed_sensor_not_an_empty_sky` |
| 14 | **The fence was centred on wherever the aircraft took off**, not on the surveyed site | the vehicle's GNSS position places it on the site before arming; a site file for somewhere else cannot be flown; home is the take-off point | `test_live.py::test_the_site_frame_follows_the_vehicle_not_the_other_way_round` |
| 15 | A task from the fusion node could name any location | tasks outside the fence, with unreadable or non-finite locations, or with a control other than start are rejected with a `TaskAck` reason | `test_live.py::test_tasks_and_traffic_from_the_fusion_node` |
| 16 | Nothing checked the aircraft before arming | `kestrel preflight`: autopilot, navigation, GNSS, site, battery, mission, camera (size, rate, not black), fusion link, clock, autopilot failsafes, disk; a failed check stops the flight; arming needs the operator to type `FLY` | `test_preflight.py` |
| 17 | *Hostile* could not be set at all, so the rule "only an operator" had no operator | `declare <track> <IDENTITY>` on the fusion console; every use is kept in the picture | `test_fusion_service.py::test_only_an_operator_makes_a_track_hostile_and_displays_are_told` |
| 18 | The picture could only be read as JSON | Cursor on Target over UDP for ATAK/WinTAK (`--cot`) | same test |
| 19 | The installed package did not work outside the repository, and nothing built it | CI builds the wheel, installs it in a clean environment and uses it; builds and runs the container | CI job `package` |
| 20 | A long flight grew memory without bound (every message kept) | the live exporter streams and forgets; the fusion node forgets the sensor objects of dropped tracks | by construction (`keep=False`), `test_live.py` |
| 21 | **One not-a-number coordinate from one sensor corrupted every track in the picture** (protobuf carries NaN and infinity like any double; association spread it), and an infinite one raised out of the fusion node | every report is checked before it touches a track: position, time, errors, velocity and confidence must be finite and plausible; a refused report is counted by reason in the picture | `test_fusion_service.py::test_one_sensor_sending_nonsense_cannot_corrupt_the_picture` |
| 22 | A sensor flooding new objects could stall the picture (duplicate merging is quadratic in tracks) | a track limit (2,000); object ids and serials are cut to 128 characters | same test |
| 23 | **A typo in a site or config file weakened the safety monitor in the air**: a fence listed out of order (a bow-tie has no inside), a negative separation, an inverted altitude band and a reserve above 100% were all accepted | fences must be finite, simple polygons with a real area; the origin must be a real position; every safety limit and camera parameter is range-checked when the file is read | `test_config.py` |

## Open: cannot be closed by software

| # | Open item | Why it matters | Who closes it |
|---|---|---|---|
| A | **No flight on a real airframe.** The MAVLink path is tested against ArduCopter SITL only | SITL has perfect sensors and no vibration, wind shear, compass error or radio dropouts | flight trials with a safety pilot, following [field-test-plan.md](field-test-plan.md) |
| B | **The detector has never seen a real frame in flight.** Its range, false-alarm rate and size estimate come from rendered pixels | real optics, exposure, compression, rolling shutter and clutter will move every number; the size-to-range calibration (`size_gain`, `size_offset`) is specific to the simulated camera | camera calibration on the chosen sensor; `kestrel evaluate` on recorded footage ([real-data.md](real-data.md)); then flight |
| C | **No real sensor has been connected.** Ground-sensor models are generic | real radars report tracks, not plots; their errors are correlated and their latencies differ | integration with partners' sensors over SAPIENT; `kestrel send` and `kestrel fuse` on their recordings first |
| D | **No run against the SAPIENT test harness or a third-party fusion node** | schema conformance is necessary, not sufficient: behaviour over a session (acks, timing, re-registration) is what the harness checks | Dstl's harness or a partner's decision-making module |
| E | **Remote ID is not authenticated** | a hostile broadcasting one of our serials while our drone is on the ground is corroborated by radar and shown as friend | authenticated friendly-force tracking from our own ground control; the node's own status reports already do this for Kestrel itself (item 8) |
| F | **GNSS denial.** Kestrel takes its position from the autopilot | jammed or spoofed GNSS degrades every bearing-to-position conversion and the fence itself; the autopilot's EKF failsafe is the only protection today | an alternative navigation source on the airframe; out of scope for this software |
| G | **The MAVLink link is unsigned** | anyone on the telemetry radio's channel can command the autopilot | MAVLink 2 signing configured on the autopilot and radios; a wired companion link reduces the exposure |
| H | **No certification, approval or qualification of any kind** | detect-and-avoid results are measured against ASTM F3442's metrics as a yardstick, not as compliance; no SORA, no airworthiness, no EMC or environmental qualification, no security accreditation | the operator's authorisation process with the competent authority |
| I | **Export control and procurement rules have not been assessed** | counter-UAS sensing may fall under dual-use or national lists depending on the hardware it is fitted to | legal review before any transfer outside the developing organisation |
| J | **The operator interface is a console and a data feed** | no map display of Kestrel's own; human factors of the *hostile* decision are untested | a TAK plug-in or the partner's C2 display, with operators in the loop |
| K | **One fusion node, one airborne node.** No redundancy, no hierarchy | a single process is a single point of failure | a second fusion node (SAPIENT hierarchical fusion) and more than one observer on the link |
| L | **Weather.** Rain, fog, icing and strong wind are not modelled | they decide whether the aircraft and the camera work at all | flight trials |

## Open: software still to do

Smaller, and honest about being unfinished:

* A certified sensor that understates its errors (but stays finite and positive) is believed:
  it splits tracks and degrades the picture without being refused. Per-sensor error floors
  from measured performance are the fix, once real sensors are connected.

* `follow`, `move_to` and `patrol` SAPIENT commands are not accepted; `CONTROL_STOP` is
  acknowledged as rejected rather than cancelling a look.
* The fusion console is a terminal; operator commands are not authenticated beyond access
  to that terminal.
* No certificate revocation: retiring a device means a new CA (`deploy/pki.sh`).
* Camera attitude assumes a fixed mount or a self-stabilised gimbal held at one pitch; the
  loop does not command a gimbal over MAVLink.
* PX4 is refused, not supported.
* Fusion throughput is measured on one site (about 2,000 reports per second on one core
  against the benchmark's 26 per second); it is not profiled for hundreds of tracks.

## What "ready for integration testing" means here

1. A partner's SAPIENT sensor can connect to `kestrel fusion` over mutual TLS, and its
   reports are validated, fused and shown, or rejected with a reason.
2. A recorded SAPIENT log can be fused offline (`kestrel fuse`) or streamed to a live
   fusion node as if the sensors were present (`kestrel send`).
3. The airborne node can be flown on ArduCopter with a camera and the fusion link, behind
   pre-flight checks, with a pilot able to take over at any moment, leaving a flight log.
4. All of the above runs on a bench without an aircraft (`kestrel fly --connect bench`).
5. The steps from bench to first flight, with pass criteria, are written down
   ([field-test-plan.md](field-test-plan.md)).
