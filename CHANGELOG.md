# Changelog

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
