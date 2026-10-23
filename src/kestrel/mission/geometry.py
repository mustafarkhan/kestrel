"""Resolve mission tasks into concrete geometry (shared by validator and executive).

Keeping this in one place means the plan that is *checked* before flight is the
same plan that is *flown*: no second interpretation of "orbit the tower".
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from kestrel.config import CircleSite, PointSite, RectSite, SafetyLimits, WorldConfig
from kestrel.geometry import Vec
from kestrel.guidance.trajectory import lawnmower, orbit_points
from kestrel.mission.schema import (
    GotoTask,
    HoldTask,
    LandTask,
    Location,
    Mission,
    OrbitTask,
    ReturnHomeTask,
    SearchTask,
    StandbyTask,
    SurveyTask,
    TakeoffTask,
    Task,
    TrackTask,
)

DEFAULT_ALTITUDE = 20.0


class GeometryError(ValueError):
    pass


def resolve_xy(loc: Location, world: WorldConfig) -> Vec:
    if isinstance(loc, str):
        if loc in ("home", "launch"):
            return np.array(world.home[:2], dtype=float)
        site = world.sites.get(loc)
        if site is None:
            raise GeometryError(f"unknown site '{loc}'")
        if isinstance(site, PointSite):
            return np.array(site.xy, dtype=float)
        if isinstance(site, CircleSite):
            return np.array(site.center, dtype=float)
        mid: Vec = (np.array(site.min_xy) + np.array(site.max_xy)) / 2.0
        return mid
    return np.array(loc, dtype=float)


def survey_bounds(area: str, world: WorldConfig) -> tuple[tuple[float, float], tuple[float, float]]:
    site = world.sites.get(area)
    if isinstance(site, RectSite):
        return site.min_xy, site.max_xy
    if isinstance(site, CircleSite):
        c, r = site.center, site.radius
        h = r / math.sqrt(2)
        return (c[0] - h, c[1] - h), (c[0] + h, c[1] + h)
    raise GeometryError(f"survey area '{area}' must be a rect or circle site")


@dataclass(frozen=True)
class TaskGeometry:
    """Nominal geometry of one task, starting from ``start``."""

    task: Task
    points: Vec  # (N, 3) waypoints in flight order (may be empty for hold/track/land)
    speed: float
    dwell_s: float  # time spent without translating (hold, search, track)
    altitude: float


def task_speed(task: Task, mission: Mission, limits: SafetyLimits) -> float:
    cruise = min(limits.max_horizontal_speed * 0.75, mission.max_speed or 1e9)
    v = getattr(task, "speed", None)
    return float(min(v, limits.max_horizontal_speed)) if v else float(cruise)


def task_geometry(
    task: Task, start: Vec, altitude: float, mission: Mission, world: WorldConfig, limits: SafetyLimits
) -> TaskGeometry:
    speed = task_speed(task, mission, limits)
    home = np.array(world.home, dtype=float)
    if isinstance(task, TakeoffTask):
        alt = task.altitude
        return TaskGeometry(task, np.array([[start[0], start[1], alt]]), limits.max_vertical_speed, 0.0, alt)
    alt = getattr(task, "altitude", None) or altitude
    if isinstance(task, GotoTask):
        xy = resolve_xy(task.at, world)
        return TaskGeometry(task, np.array([[xy[0], xy[1], alt]]), speed, 0.0, alt)
    if isinstance(task, OrbitTask):
        c = resolve_xy(task.at, world)
        start_angle = math.atan2(start[1] - c[1], start[0] - c[0])
        pts = orbit_points(np.array([c[0], c[1], alt]), task.radius, task.laps, start_angle, task.clockwise)
        return TaskGeometry(task, pts, speed, 0.0, alt)
    if isinstance(task, SurveyTask):
        lo, hi = survey_bounds(task.area, world)
        return TaskGeometry(task, lawnmower(lo, hi, alt, task.spacing), speed, 0.0, alt)
    if isinstance(task, SearchTask):
        c = resolve_xy(task.at, world)
        pts = orbit_points(np.array([c[0], c[1], alt]), task.radius, 1.0, 0.0, False)
        return TaskGeometry(task, pts, min(speed, 3.0), task.duration_s, alt)
    if isinstance(task, HoldTask):
        return TaskGeometry(task, np.empty((0, 3)), speed, task.duration_s, alt)
    if isinstance(task, StandbyTask):
        xy = resolve_xy(task.at, world)
        return TaskGeometry(task, np.array([[xy[0], xy[1], alt]]), speed, task.duration_s, alt)
    if isinstance(task, TrackTask):
        return TaskGeometry(task, np.empty((0, 3)), speed, task.duration_s, alt)
    if isinstance(task, ReturnHomeTask):
        return TaskGeometry(task, np.array([[home[0], home[1], alt]]), limits.rtl_speed, 0.0, alt)
    if isinstance(task, LandTask):
        return TaskGeometry(task, np.empty((0, 3)), limits.land_speed, alt / limits.land_speed, 0.0)
    raise GeometryError(f"unsupported task {task!r}")
