# Changelog

## 0.5.0 (2026-11-10)

Night, thermal, and real-data evaluation.

- Visible and long-wave infrared rendering by day and night; a bright-target (white hot)
  detector mode; a thermal camera config (320 × 256, 50°) and a night airfield.
- Four new benchmark sets on the same encounters (seeds 4000–4023): a visible camera at
  night, a thermal camera at night, with and without the ground acoustic network, and a
  thermal camera by day ([0016](docs/decisions/0016-night-and-thermal.md)).
- `kestrel evaluate`: the detector on recorded footage in Anti-UAV, YOLO or MOT layout,
  recall by target size, false detections per frame ([docs/real-data.md](docs/real-data.md)).

## 0.4.0 (2026-11-10)

The recognised air picture.

- Simulated radar, RF direction finders, Remote ID receiver and acoustic grid as SAPIENT
  nodes; a SAPIENT fusion node with association, Kalman fusion, duplicate merging, bounded
  class evidence and APP-6 identities (*hostile* only by an operator).
- The fusion node tasks Kestrel with `LOOK_AT` and shares its 3-D tracks with Kestrel's
  separation filter.
- SAPIENT aligned with Dstl's published BSI Flex 335 v2.0 protobuf schema and tested against it.
- `kestrel picture`, `kestrel picture --loop`, `kestrel fuse`
  ([0015](docs/decisions/0015-air-picture.md), [docs/air-picture.md](docs/air-picture.md)).

## 0.2.0–0.3.0 (October 2026)

Birds and the bird/drone classifier, acoustic cueing, SAPIENT reports and tasking, two
observers, the sensor-node mission and jammed-link autonomy: decision records 0009–0014 and
pull requests #2–#4.

## 0.1.0 (2026-10-22)

First release.

- Language-tasked missions: local LLM planner (Ollama, schema-constrained) with a
  deterministic rule-parser fallback and regex emergency commands.
- Static plan validation: geofence, altitude band, speed, separation standoff,
  battery energy.
- Air-to-air perception on rendered frames: small-target sky detector, multi-target
  tracker (bearing + log-range EKF, GNN association, M-of-N confirmation).
- Mission executive with contact pre-emption, resume, and contact watch.
- Runtime safety monitor: geofence, altitude, kinematics, range-aware separation with
  closing-speed and reachable-set terms, latched energy and link-loss failsafes.
- Quadrotor simulator: rigid-body dynamics, motor lag, wind, cascaded position and
  SE(3) attitude control, stabilised gimbal.
- ArduPilot over MAVLink (GUIDED), integration-tested on ArduCopter 4.6.3 SITL in CI.
- Monte Carlo benchmark, report plots, demo animation and a project video rendered from the
  repository's own outputs (`docs/video/make_video.py`).
