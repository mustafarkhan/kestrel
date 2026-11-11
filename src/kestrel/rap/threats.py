"""A library of named, versioned threat situations for testing an air picture.

Each threat is one way a counter-drone sensor network is beaten in practice, drawn from the
front-line and incident reporting in ``docs/research.md``: drones that emit nothing, raids
that saturate the tracker, decoys, Remote ID that lies, birds, and a sensor going dark. A
threat is a function of a seed, so the same situation can be given to any system, any number
of times, and two systems can be compared on exactly the same sky.

The library has a version. A threat's definition never changes within a version: a change
to any of them is a new version, and results quote the version they were measured with.

Everything here is a **situation to detect**, flown past sensors. Nothing in it attacks a
system's software or links.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from kestrel.config import BirdConfig, IntruderConfig, Scenario
from kestrel.rap.picture import HALF_M, WHITELIST, flank_scenario

LIBRARY_VERSION = "1.0"


@dataclass(frozen=True)
class Threat:
    name: str
    title: str
    tests: str  # what a system has to do to handle it
    watch: tuple[str, ...]  # the scorecard measures that matter most for it
    build: Callable[[int], Scenario]


def _edge(
    rng: np.random.Generator, alt: float, sector: tuple[float, float] = (-math.pi, math.pi)
) -> tuple[float, float, float]:
    a = rng.uniform(*sector)
    return (1.3 * HALF_M * math.cos(a), 1.3 * HALF_M * math.sin(a), alt)


def _base(seed: int, name: str) -> Scenario:
    """The benchmark site and sensors with only the two friendly patrols and the civil drone."""
    sc = flank_scenario(seed, spoofer=False)
    keep = [d for d in sc.intruders if d.role != "hostile"]
    return sc.model_copy(update={"name": f"{name}-{seed}", "intruders": keep})


def _with(sc: Scenario, drones: list[IntruderConfig], **changes: object) -> Scenario:
    return sc.model_copy(update={"intruders": [*sc.intruders, *drones], **changes})


def _rng(seed: int, salt: int) -> np.random.Generator:
    return np.random.default_rng([seed, salt])


def mixed_raid(seed: int) -> Scenario:
    sc = flank_scenario(seed, spoofer=False)
    return sc.model_copy(update={"name": f"mixed-raid-{seed}"})


def silent_low(seed: int) -> Scenario:
    """Four quiet multirotors with no radio link (fibre-optic or autonomous), low, from all sides."""
    rng = _rng(seed, 1)
    sc = _base(seed, "silent-low")
    depot = (0.0, 0.0)
    drones = []
    for k in range(4):
        start = _edge(
            rng, float(rng.uniform(8, 20)), (-math.pi + k * math.pi / 2, -math.pi / 2 + k * math.pi / 2)
        )
        aim = (depot[0] + float(rng.uniform(-80, 80)), depot[1] + float(rng.uniform(-80, 80)), start[2])
        drones.append(
            IntruderConfig(
                pattern="waypoints",
                center=start,
                waypoints=[start, aim, (-0.4 * start[0], -0.4 * start[1], start[2])],
                speed=float(rng.uniform(6, 12)),
                appear_at_s=float(rng.uniform(0, 240)),
                ingress_m=0.0,
                rf_link=False,
                propulsion="quiet",
                rcs_m2=0.008,
                radar_class_cue=0.65,
            )
        )
    return _with(sc, drones)


def one_way_attack(seed: int) -> Scenario:
    """Three fast fixed-wing drones with piston engines, straight at the depot, minutes apart."""
    rng = _rng(seed, 2)
    sc = _base(seed, "one-way-attack")
    drones = []
    for k in range(3):
        start = _edge(rng, float(rng.uniform(60, 120)))
        aim = (float(rng.uniform(-60, 60)), float(rng.uniform(-60, 60)))
        drones.append(
            IntruderConfig(
                pattern="straight",
                center=start,
                size=(aim[0] - start[0], aim[1] - start[1]),
                speed=float(rng.uniform(28, 45)),
                appear_at_s=float(60 + 150 * k + rng.uniform(0, 60)),
                ingress_m=0.0,
                rf_link=False,
                propulsion="combustion",
                rcs_m2=0.05,
                radar_class_cue=0.55,
            )
        )
    return _with(sc, drones)


def saturation(seed: int) -> Scenario:
    """Twelve radio-controlled quadcopters from one sector within a minute."""
    rng = _rng(seed, 3)
    sc = _base(seed, "saturation")
    centre = float(rng.uniform(-math.pi, math.pi))
    t0 = float(rng.uniform(60, 180))
    drones = []
    for _ in range(12):
        start = _edge(rng, float(rng.uniform(30, 90)), (centre - 0.5, centre + 0.5))
        aim = (float(rng.uniform(-200, 200)), float(rng.uniform(-200, 200)), start[2])
        drones.append(
            IntruderConfig(
                pattern="waypoints",
                center=start,
                waypoints=[start, aim, (aim[0] + float(rng.uniform(-150, 150)), aim[1] + 150.0, start[2])],
                speed=float(rng.uniform(8, 14)),
                appear_at_s=t0 + float(rng.uniform(0, 60)),
                ingress_m=0.0,
                rf_link=True,
                rf_protocol="FHSS 2.4 GHz control",
                rcs_m2=0.01,
                radar_class_cue=0.7,
            )
        )
    return _with(sc, drones)


def decoys(seed: int) -> Scenario:
    """Eight slow, radar-bright decoys with radio links, and behind them two silent low attackers."""
    rng = _rng(seed, 4)
    sc = _base(seed, "decoys")
    drones = []
    for _ in range(8):
        start = _edge(rng, float(rng.uniform(60, 110)), (0.0, math.pi))
        end = (float(rng.uniform(-500, 500)), float(rng.uniform(-100, 400)))
        drones.append(
            IntruderConfig(
                pattern="straight",
                center=start,
                size=(end[0] - start[0], end[1] - start[1]),
                speed=float(rng.uniform(12, 18)),
                appear_at_s=float(rng.uniform(40, 160)),
                ingress_m=0.0,
                rf_link=True,
                rf_protocol="analogue video 5.8 GHz",
                propulsion="electric",
                rcs_m2=0.2,  # a reflector fitted to look like something larger
                radar_class_cue=0.6,
            )
        )
    for _ in range(2):
        start = _edge(rng, float(rng.uniform(8, 18)), (-math.pi, 0.0))
        aim = (float(rng.uniform(-60, 60)), float(rng.uniform(-60, 60)), start[2])
        drones.append(
            IntruderConfig(
                pattern="waypoints",
                center=start,
                waypoints=[start, aim, (aim[0], aim[1] + 120.0, start[2])],
                speed=float(rng.uniform(7, 11)),
                appear_at_s=float(rng.uniform(100, 200)),
                ingress_m=0.0,
                rf_link=False,
                propulsion="quiet",
                rcs_m2=0.008,
                radar_class_cue=0.65,
            )
        )
    return _with(sc, drones)


def remote_id_spoof(seed: int) -> Scenario:
    sc = flank_scenario(seed, spoofer=True)
    return sc.model_copy(update={"name": f"remote-id-spoof-{seed}"})


def remote_id_impersonation(seed: int) -> Scenario:
    """A hostile broadcasts one of our serials **from its own true position**, while our drone
    with that serial is on the ground. Remote ID is not authenticated: any independent sensor
    confirms the broadcast, because the broadcast is true about everything except who it is."""
    rng = _rng(seed, 6)
    sc = _base(seed, "remote-id-impersonation")
    grounded = WHITELIST[0]
    flying = [d for d in sc.intruders if d.remote_id != grounded]
    start = _edge(rng, float(rng.uniform(30, 70)))
    hostile = IntruderConfig(
        pattern="waypoints",
        center=start,
        waypoints=[start, (40.0, -40.0, start[2]), (-0.5 * start[1], 0.5 * start[0], start[2])],
        speed=float(rng.uniform(6, 10)),
        appear_at_s=float(rng.uniform(0, 120)),
        ingress_m=0.0,
        rf_link=True,
        rf_protocol="FHSS 2.4 GHz control",
        remote_id=grounded,
        rcs_m2=0.01,
    )
    return sc.model_copy(update={"intruders": [*flying, hostile]})


def birds_only(seed: int) -> Scenario:
    """Forty birds, our own patrols and a civil drone; nothing hostile. Every alarm is a false one."""
    sc = _base(seed, "birds-only")
    return sc.model_copy(update={"birds": BirdConfig(count=40, altitude=(10.0, 100.0))})


def radar_outage(seed: int) -> Scenario:
    """The mixed raid with the radar silent from 200 s to 400 s (down, jammed or cut off)."""
    sc = flank_scenario(seed, spoofer=False)
    assert sc.sensors is not None
    sensors = sc.sensors.model_copy(update={"outages": {"radar": [(200.0, 400.0)]}})
    return sc.model_copy(update={"name": f"radar-outage-{seed}", "sensors": sensors})


THREATS: dict[str, Threat] = {
    t.name: t
    for t in (
        Threat(
            "mixed-raid",
            "Mixed raid",
            "one radio-controlled, one engine-driven and one silent low drone at once, among patrols and "
            "birds",
            ("hostile_coverage", "drone_as_uav", "bird_time_as_uav"),
            mixed_raid,
        ),
        Threat(
            "silent-low",
            "Silent and low",
            "four quiet drones with no radio link, 8 to 20 m up, from all sides: nothing for RF or acoustic "
            "sensors to hear, little for a radar to see",
            ("hostile_coverage", "first_track_s", "drones_never_tracked"),
            silent_low,
        ),
        Threat(
            "one-way-attack",
            "One-way attack",
            "three fast fixed-wing drones straight at the protected site: how early they are tracked "
            "and called drones",
            ("hostile_coverage", "first_track_s", "track_to_uav_s"),
            one_way_attack,
        ),
        Threat(
            "saturation",
            "Saturation",
            "twelve drones from one sector within a minute: track capacity, duplicates and track continuity",
            ("hostile_coverage", "duplicate_ratio", "ids_per_drone"),
            saturation,
        ),
        Threat(
            "decoys",
            "Decoys",
            "eight radar-bright decoys on one side while two silent low drones come from the other",
            ("hostile_coverage", "drones_never_tracked", "drone_as_uav"),
            decoys,
        ),
        Threat(
            "remote-id-spoof",
            "Remote ID spoof",
            "a hostile broadcasts a friendly serial with a false position",
            ("phantom_friend_s", "hostile_as_friend_s", "friend_identified"),
            remote_id_spoof,
        ),
        Threat(
            "remote-id-impersonation",
            "Remote ID impersonation",
            "a hostile broadcasts a grounded friend's serial from its own true position",
            ("hostile_as_friend_s", "friend_identified"),
            remote_id_impersonation,
        ),
        Threat(
            "birds-only",
            "Birds only",
            "forty birds and no hostile drone: every drone alarm is false",
            ("bird_time_as_uav", "false_mean"),
            birds_only,
        ),
        Threat(
            "radar-outage",
            "Radar outage",
            "the mixed raid with the radar silent for 200 s: what the other sensors hold, and how the "
            "picture recovers",
            ("hostile_coverage", "ids_per_drone", "drone_as_uav"),
            radar_outage,
        ),
    )
}


def build(name: str, seed: int) -> Scenario:
    if name not in THREATS:
        raise KeyError(f"unknown threat {name!r}; known: {', '.join(THREATS)}")
    return THREATS[name].build(seed)
