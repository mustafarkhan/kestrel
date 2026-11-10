"""Building and scoring a picture from the ground sensors alone (no airborne node).

``flank_scenario(seed)`` draws one benchmark situation over a 1.6 km protected area of the
kind the front-line and allied sources describe: mixed hostile drones that each defeat a
different sensor, our own drones broadcasting whitelisted Remote ID, a compliant civil
drone, sometimes a spoofer broadcasting one of our serials from a false position, and
birds. ``run_picture`` flies it with any subset of the sensors and scores the result.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from kestrel.config import (
    BirdConfig,
    CircleSite,
    GroundAcousticConfig,
    IntruderConfig,
    PointSite,
    RadarConfig,
    RemoteIdConfig,
    RfConfig,
    Scenario,
    SensorNetConfig,
    WorldConfig,
)
from kestrel.geofence import Geofence
from kestrel.interop.sapient import SapientExporter, SapientOptions
from kestrel.rap.fusion import FusionNode, Message, RapTrack
from kestrel.rap.score import PictureScore
from kestrel.rap.sensors import SensorNetwork
from kestrel.rap.truth import Truth, TruthObject

HALF_M = 800.0
WHITELIST = ["KST-FRIEND-0001", "KST-FRIEND-0002"]
ALTITUDE_BAND = (0.0, 150.0)


def _edge_point(rng: np.random.Generator, alt: float) -> tuple[float, float, float]:
    a = rng.uniform(-math.pi, math.pi)
    return (1.3 * HALF_M * math.cos(a), 1.3 * HALF_M * math.sin(a), alt)


def _heading_to(start: tuple[float, float, float], target: tuple[float, float]) -> tuple[float, float]:
    return (target[0] - start[0], target[1] - start[1])


def flank_scenario(seed: int, spoofer: bool | None = None) -> Scenario:
    rng = np.random.default_rng(seed)
    depot = (0.0, 0.0)
    substation = (float(rng.uniform(250, 450)), float(rng.uniform(-450, -250)))
    drones: list[IntruderConfig] = []

    # 1. Radio-controlled quadcopter: RF hears it, radar sees it, acoustic hears it nearby.
    start = _edge_point(rng, float(rng.uniform(30, 80)))
    goal = substation if rng.random() < 0.5 else depot
    drones.append(
        IntruderConfig(
            pattern="waypoints",
            center=start,
            speed=float(rng.uniform(6, 12)),
            waypoints=[start, (goal[0], goal[1], start[2]), (goal[0] + 150.0, goal[1] + 100.0, start[2])],
            appear_at_s=float(rng.uniform(0, 120)),
            ingress_m=0.0,
            rf_link=True,
            rf_protocol="FHSS 2.4 GHz control, 5.8 GHz video",
            propulsion="electric",
            rcs_m2=0.01,
            radar_class_cue=0.7,
        )
    )
    # 2. Autonomous fixed-wing with a piston engine (one-way attack type): no radio link,
    #    loud, larger radar cross-section, fast and straight at the depot.
    start = _edge_point(rng, float(rng.uniform(60, 120)))
    drones.append(
        IntruderConfig(
            pattern="straight",
            center=start,
            size=_heading_to(start, depot),
            speed=float(rng.uniform(25, 40)),
            appear_at_s=float(rng.uniform(60, 300)),
            ingress_m=0.0,
            rf_link=False,
            propulsion="combustion",
            rcs_m2=0.05,
            radar_class_cue=0.55,
        )
    )
    # 3. Fibre-optic or autonomous quiet multirotor, low: no RF, little sound, often under radar.
    start = _edge_point(rng, float(rng.uniform(8, 20)))
    drones.append(
        IntruderConfig(
            pattern="waypoints",
            center=start,
            speed=float(rng.uniform(5, 10)),
            waypoints=[
                start,
                (depot[0] + rng.uniform(-100, 100), depot[1] + rng.uniform(-100, 100), start[2]),
                (0.5 * start[0] + rng.uniform(-200, 200), 0.5 * start[1] + rng.uniform(-200, 200), start[2]),
            ],
            appear_at_s=float(rng.uniform(0, 200)),
            ingress_m=0.0,
            rf_link=False,
            propulsion="quiet",
            rcs_m2=0.008,
            radar_class_cue=0.65,
        )
    )
    # 4-5. Our own drones on patrol, broadcasting whitelisted Remote ID.
    for k, serial in enumerate(WHITELIST):
        c = (float(rng.uniform(-400, 400)), float(rng.uniform(-400, 400)), float(rng.uniform(40, 70)))
        drones.append(
            IntruderConfig(
                pattern="orbit",
                center=c,
                size=(float(rng.uniform(80, 200)), 0.0),
                speed=float(rng.uniform(5, 9)),
                appear_at_s=float(30 * k),
                ingress_m=0.0,
                role="friendly",
                rf_link=True,
                rf_protocol="friendly C2 link",
                remote_id=serial,
                rcs_m2=0.012,
            )
        )
    # 6. A compliant civil drone crossing, broadcasting its own serial.
    start = _edge_point(rng, float(rng.uniform(40, 100)))
    drones.append(
        IntruderConfig(
            pattern="straight",
            center=start,
            size=_heading_to(start, (rng.uniform(-300, 300), rng.uniform(-300, 300))),
            speed=float(rng.uniform(5, 10)),
            appear_at_s=float(rng.uniform(0, 300)),
            ingress_m=0.0,
            role="civil",
            rf_link=True,
            rf_protocol="consumer video link",
            remote_id=f"CIV-{seed:05d}",
            rcs_m2=0.01,
        )
    )
    # 7. Sometimes a spoofer: a hostile radio-controlled drone broadcasting one of our serials
    #    with a position 150-300 m from where it really is.
    if spoofer if spoofer is not None else rng.random() < 0.5:
        start = _edge_point(rng, float(rng.uniform(30, 70)))
        off = rng.uniform(150, 300)
        a = rng.uniform(-math.pi, math.pi)
        drones.append(
            IntruderConfig(
                pattern="waypoints",
                center=start,
                speed=float(rng.uniform(6, 10)),
                waypoints=[
                    start,
                    (depot[0] + 50.0, depot[1] - 50.0, start[2]),
                    (-0.6 * start[1], 0.6 * start[0], start[2]),
                ],
                appear_at_s=float(rng.uniform(0, 200)),
                ingress_m=0.0,
                rf_link=True,
                rf_protocol="FHSS 2.4 GHz control",
                remote_id=WHITELIST[0],
                remote_id_offset_m=(off * math.cos(a), off * math.sin(a), 0.0),
                rcs_m2=0.01,
            )
        )
    world = WorldConfig(
        fence=[(-HALF_M, -HALF_M), (HALF_M, -HALF_M), (HALF_M, HALF_M), (-HALF_M, HALF_M)],
        home=(0.0, 0.0, 0.0),
        origin_lla=(60.30, 25.05, 20.0),
        sites={"depot": CircleSite(center=depot, radius=150.0), "substation": PointSite(xy=substation)},
    )
    sensors = SensorNetConfig(
        radar=RadarConfig(),
        rf=RfConfig(nodes=[(-600.0, -600.0), (600.0, -600.0), (600.0, 600.0), (-600.0, 600.0)]),
        remote_id=RemoteIdConfig(range_m=1200.0),
        acoustic=GroundAcousticConfig(),
        whitelist=list(WHITELIST),
        protected=dict(world.sites),
    )
    return Scenario(
        name=f"flank-{seed}",
        seed=seed,
        max_duration_s=600.0,
        world=world,
        intruders=drones,
        birds=BirdConfig(count=12, altitude=(10.0, 80.0)),
        sensors=sensors,
    )


@dataclass
class PictureRun:
    scenario: Scenario
    fusion: FusionNode
    score: PictureScore
    messages: list[Message]
    frames: list[tuple[float, list[TruthObject], list[RapTrack]]] = field(default_factory=list)


def make_geo(scenario: Scenario, seed: int) -> SapientExporter:
    return SapientExporter(
        SapientOptions(origin_lla=scenario.world.origin_lla, seed=seed, altitude_band_m=ALTITUDE_BAND)
    )


def run_picture(
    scenario: Scenario,
    include: set[str] | None = None,
    whitelist: bool = True,
    dt: float = 0.5,
    keep_messages: bool = False,
    record_every_s: float = 0.0,
) -> PictureRun:
    """Fly the scenario's truth past the ground sensors; fuse and score every ``dt``.

    ``record_every_s`` > 0 keeps the truth and the picture at that interval, for replay.
    """
    assert scenario.sensors is not None, "scenario has no sensor network"
    net_cfg = scenario.sensors
    geo = make_geo(scenario, scenario.seed)
    truth = Truth(scenario)
    net = SensorNetwork(net_cfg, scenario.world, geo, scenario.seed, include, ALTITUDE_BAND)
    fusion = FusionNode(geo, scenario.world, net_cfg.whitelist if whitelist else [], net_cfg.protected)
    score = PictureScore()
    fence = Geofence(scenario.world.fence)
    messages: list[Message] = []
    frames: list[tuple[float, list[TruthObject], list[RapTrack]]] = []
    next_record = 0.0
    t = 0.0
    while t <= scenario.max_duration_s:
        objects = truth.objects(t)
        for msg in net.step(t, objects):
            fusion.ingest(msg)
            if keep_messages:
                messages.append(msg)
        fusion.step(t)
        picture = fusion.picture(t)
        score.add(t, dt, objects, picture, fusion, fence)
        if record_every_s > 0.0 and t >= next_record - 1e-9:
            frames.append((t, objects, picture))
            next_record += record_every_s
        t = round(t + dt, 6)
    return PictureRun(scenario, fusion, score, messages, frames)
