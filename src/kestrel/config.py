"""Typed configuration.

Every tunable lives here as a pydantic model with a sensible default, so a run is
fully described by one YAML file (see ``configs/default.yaml``) and nothing is
hidden as a magic number inside an algorithm.
"""

from __future__ import annotations

from pathlib import Path
from typing import Annotated, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

XY = tuple[float, float]
XYZ = tuple[float, float, float]


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


# --------------------------------------------------------------------------- vehicle


class VehicleParams(_Strict):
    """Physical parameters of the observer quadrotor (X configuration)."""

    mass: float = 1.6
    inertia: XYZ = (0.021, 0.021, 0.038)
    arm_length: float = 0.20
    max_thrust_per_motor: float = 9.0  # N  -> thrust-to-weight ~2.3
    yaw_moment_coeff: float = 0.016  # rotor drag torque / thrust [m]
    motor_time_constant: float = 0.03  # s, first-order thrust lag
    linear_drag: XYZ = (0.25, 0.25, 0.35)  # N per m/s of air-relative speed, body axes
    rotor_radius: float = 0.12
    propulsive_efficiency: float = 0.55  # electrical -> ideal induced power
    battery_wh: float = 77.0  # 4S 5200 mAh
    camera_offset: XYZ = (0.10, 0.0, 0.0)


class ControlGains(_Strict):
    """Cascaded position/velocity loop feeding a geometric SE(3) attitude controller."""

    kp: XYZ = (1.35, 1.35, 1.6)  # position error -> velocity command [1/s]
    kv: XYZ = (4.5, 4.5, 5.0)  # velocity error -> acceleration [1/s]
    max_speed_xy: float = 10.0  # autopilot's own envelope (cf. WPNAV_SPEED / MPC_XY_VEL_MAX)
    max_speed_z: float = 3.0
    ki: XYZ = (0.4, 0.4, 0.8)
    integral_limit: float = 2.0
    k_rot: XYZ = (3.0, 3.0, 1.2)  # tuned with the 30 ms motor lag in the loop
    k_omega: XYZ = (0.40, 0.40, 0.15)
    max_tilt_deg: float = 35.0
    max_yaw_rate_deg_s: float = 90.0  # yaw reference slew limit (cf. ATC_RATE_Y_MAX)


# --------------------------------------------------------------------------- sensing


class CameraParams(_Strict):
    width: int = 480
    height: int = 360
    hfov_deg: float = 75.0
    pitch_up_deg: float = 8.0  # default tilt, up, so targets sit against sky
    gimbal: bool = True  # 2-axis stabilised (roll/pitch); yaw follows the airframe
    gimbal_min_deg: float = -30.0
    gimbal_max_deg: float = 45.0
    gimbal_rate_deg_s: float = 90.0
    gimbal_jitter_deg: float = 0.15  # residual stabilisation error
    rate_hz: float = 10.0
    # Camera layout: yaw of each camera relative to the nose. [0] is one forward camera;
    # [0, 72, 144, -144, -72] is a five-camera ring with full 360 degree coverage at 75 deg hfov.
    yaw_offsets_deg: tuple[float, ...] = (0.0,)
    noise_sigma: float = 2.0  # grey levels (0..255)
    latency_s: float = 0.05


class AcousticParams(_Strict):
    """Ground acoustic cueing network (off by default). Sources in perception/acoustic.py."""

    enabled: bool = False
    node_spacing_m: float = 100.0  # grid over the operating area
    range_m: float = 250.0  # small quadcopter, conservative end of "hundreds to a few thousand m"
    bearing_sigma_deg: float = 2.0  # single-node azimuth accuracy reported for fielded dishes
    detection_probability: float = 0.8  # per node, per update
    min_nodes: int = 2  # bearings needed to triangulate a fix
    rate_hz: float = 1.0
    latency_s: float = 1.0  # bearings are collected and correlated before a fix is published
    cue_memory_s: float = 3.0  # how long a fix is used after it was heard


class StateNoise(_Strict):
    """Noise on the navigation solution the autonomy sees (not the truth)."""

    position_sigma: float = 0.15
    velocity_sigma: float = 0.05
    tilt_sigma_deg: float = 0.3
    yaw_sigma_deg: float = 0.5


class DetectorParams(_Strict):
    tophat_size: int = 11  # px, must exceed the largest expected target
    threshold_k: float = 10.0  # robust z-score threshold on the top-hat response
    min_contrast: float = 30.0  # grey levels; clouds rarely produce dark spots this deep
    sky_darkness_min: float = 8.0  # target must be this much darker than clear sky nearby
    size_gain: float = 4.28  # apparent diameter = gain * blob spread + offset
    size_offset: float = -1.16
    min_area_px: int = 2
    max_area_px: int = 400
    max_aspect: float = 4.0
    # Search region ends this far above the true horizon (negative: below it). -1 deg lets the
    # detector see same-altitude traffic, which sits on the horizon; the background there is haze
    # at kilometre range. Real skyline clutter (treelines) further down is not modelled.
    horizon_margin_deg: float = -1.0


class TrackerParams(_Strict):
    accel_noise: float = 4.0  # m/s^2: small UAS turn hard; lower values lag manoeuvres by metres
    bearing_sigma_deg: float = 0.6  # dominated by navigation attitude error, not pixels
    log_range_sigma: float = 0.35  # inflated over the ~0.15 per-frame error: errors are correlated
    target_size_m: float = 0.35  # assumed characteristic size of a small UAS
    gate_chi2: float = 16.27  # 3 DoF, p = 0.999
    init_score: float = 14.0  # weaker detections may update tracks but not start them
    confirm_hits: int = 4
    confirm_window: int = 6
    max_coast_s: float = 5.0  # lost tracks linger with growing uncertainty: separation keeps respecting them
    max_position_std: float = 25.0
    # Classify tracks as bird or drone from wing-beat flicker (perception/classify.py). Off by
    # default so the bird-free benchmarks are unchanged; configs/birds.yaml turns it on.
    classify_birds: bool = False


# --------------------------------------------------------------------------- world


class PointSite(_Strict):
    kind: Literal["point"] = "point"
    xy: XY


class CircleSite(_Strict):
    kind: Literal["circle"] = "circle"
    center: XY
    radius: float = Field(gt=0)


class RectSite(_Strict):
    kind: Literal["rect"] = "rect"
    min_xy: XY
    max_xy: XY

    @model_validator(mode="after")
    def _ordered(self) -> RectSite:
        if not (self.min_xy[0] < self.max_xy[0] and self.min_xy[1] < self.max_xy[1]):
            raise ValueError("rect min_xy must be strictly below max_xy")
        return self


Site = Annotated[PointSite | CircleSite | RectSite, Field(discriminator="kind")]


class WindParams(_Strict):
    mean: XYZ = (2.0, -1.0, 0.0)
    gust_sigma: float = 1.0
    gust_time_constant: float = 3.0


class WorldConfig(_Strict):
    fence: list[XY] = Field(
        default_factory=lambda: [(-150.0, -150.0), (150.0, -150.0), (150.0, 150.0), (-150.0, 150.0)]
    )
    home: XYZ = (0.0, 0.0, 0.0)
    # Geodetic position (lat deg, lon deg, alt m above the geoid) of the local ENU origin, used
    # for reports to other systems. Default: ArduPilot SITL's default home.
    origin_lla: XYZ = (-35.363261, 149.165230, 584.0)
    surface: Literal["land", "sea"] = "land"  # what lies below the horizon in the camera
    sites: dict[str, Site] = Field(default_factory=dict)
    wind: WindParams = WindParams()

    @model_validator(mode="after")
    def _fence_ok(self) -> WorldConfig:
        if len(self.fence) < 3:
            raise ValueError("fence needs at least three vertices")
        return self


# --------------------------------------------------------------------------- safety


class SafetyLimits(_Strict):
    min_altitude: float = 5.0  # AGL, enforced once airborne (not during takeoff/landing)
    max_altitude: float = 60.0
    max_horizontal_speed: float = 8.0
    max_vertical_speed: float = 2.5
    max_acceleration: float = 4.0
    fence_margin: float = 8.0
    min_separation: float = 12.0  # to any tracked aircraft
    separation_margin_sigmas: float = 2.0
    range_uncertainty_floor: float = 0.15  # size-based range is never better than this fraction
    intruder_max_speed: float = 8.0  # for the reachable-set keep-out around unobserved tracks
    reaction_time_s: float = 0.8  # camera latency + 4-frame track confirmation + autonomy + autopilot
    energy_reserve_fraction: float = 0.20
    traffic_contingency_fraction: float = 1.0  # extra plan energy held for traffic delays
    rtl_speed: float = 6.0
    link_loss_timeout_s: float = 5.0
    # What a lost ground link means. "rtl": return home after link_loss_timeout_s (the civil
    # default). "continue": keep flying the mission autonomously for up to link_loss_autonomy_s,
    # buffering reports, because in counter-UAS work the link is jammed exactly when the
    # sensor is needed; separation, geofence and energy limits are enforced on board either way.
    link_loss_action: Literal["rtl", "continue"] = "rtl"
    link_loss_autonomy_s: float = 120.0
    land_speed: float = 0.8


# --------------------------------------------------------------------------- scenario


class IntruderConfig(_Strict):
    pattern: Literal["lemniscate", "waypoints", "orbit", "straight"] = "lemniscate"
    center: XYZ = (40.0, 60.0, 25.0)
    size: XY = (45.0, 25.0)  # pattern half-extent (x, y)
    speed: float = 5.0
    waypoints: list[XYZ] = Field(default_factory=list)
    appear_at_s: float = 0.0  # time the intruder reaches its pattern
    # A late intruder flies in from this far outside its pattern start (straight, level, at its
    # speed) instead of appearing out of nowhere next to the observer.
    ingress_m: float = 150.0
    span_m: float = 0.35


class BirdConfig(_Strict):
    """Birds in the operating area: the main source of false alarms for drone detection.

    Defaults describe medium birds (corvids, pigeons, gulls) whose size at range is close
    to a small drone's. Many birds fly intermittently, alternating flapping and gliding;
    a gliding bird has no wing-beat to give it away.
    """

    count: int = 0  # birds present at any time (a bird that leaves is replaced)
    span_m: XY = (0.5, 1.0)
    speed: XY = (8.0, 13.0)
    altitude: XY = (10.0, 60.0)  # AGL
    flap_hz: XY = (3.0, 6.0)
    glide_fraction: float = 0.3  # share of time spent gliding
    mean_bout_s: float = 2.5  # mean length of a flapping bout


class LinkLossEvent(_Strict):
    start_s: float
    duration_s: float


class Scenario(_Strict):
    name: str = "default"
    seed: int = 7
    max_duration_s: float = 420.0
    world: WorldConfig = WorldConfig()
    intruders: list[IntruderConfig] = Field(default_factory=list)
    birds: BirdConfig = BirdConfig()
    link_loss: list[LinkLossEvent] = Field(default_factory=list)


class Config(_Strict):
    vehicle: VehicleParams = VehicleParams()
    control: ControlGains = ControlGains()
    camera: CameraParams = CameraParams()
    state_noise: StateNoise = StateNoise()
    acoustic: AcousticParams = AcousticParams()
    detector: DetectorParams = DetectorParams()
    tracker: TrackerParams = TrackerParams()
    safety: SafetyLimits = SafetyLimits()
    physics_dt: float = 0.0025
    autonomy_rate_hz: float = 20.0


def load_yaml(path: str | Path) -> dict[str, object]:
    with open(path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: top level must be a mapping")
    return data


def load_config(path: str | Path | None = None) -> Config:
    return Config() if path is None else Config.model_validate(load_yaml(path))


def load_scenario(path: str | Path) -> Scenario:
    return Scenario.model_validate(load_yaml(path))
