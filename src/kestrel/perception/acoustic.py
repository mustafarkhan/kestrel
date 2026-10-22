"""Ground acoustic sensor network that cues the observer before visual contact.

Modelled on the passive acoustic networks fielded in Ukraine (e.g. Sky Fortress),
as described in the US Army Center for Army Lessons Learned report *Listening to
the sky* (2026): cheap microphone nodes estimate the bearing of a drone's sound,
send compact bearing records rather than audio, and the network triangulates a
position when several nodes hear the same aircraft. The report's lesson is that
such nodes are an early-warning and *cueing* layer for cameras and radar, not a
replacement for them; Kestrel uses them exactly that way.

What the model includes, with the source of each number in ``AcousticParams``:

* a bearing error per node (about 2 deg for a single dish, per the report);
* a detection range for a small drone and a per-update detection probability;
* a fix only when at least two nodes hear the aircraft, by least-squares
  intersection of bearing lines, with a covariance from the geometry;
* fix latency (bearings are collected and correlated before a fix is published);
* **no altitude**: sound gives direction on the ground plane. Consumers must treat a
  fix as a vertical column, which is the conservative reading.

Not modelled: wind, terrain and urban noise, multipath, and deliberate acoustic
masking, all named as limitations in the report.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from kestrel.config import AcousticParams, WorldConfig
from kestrel.geometry import Vec


@dataclass(frozen=True)
class AcousticFix:
    t: float  # time the sound was heard
    xy: Vec  # triangulated horizontal position (world ENU)
    cov_xy: Vec  # 2x2 covariance of the fix
    nodes: int  # number of nodes that contributed


def node_grid(world: WorldConfig, spacing: float) -> Vec:
    """Nodes on a square grid over the geofence bounding box."""
    f = np.asarray(world.fence, dtype=float)
    lo, hi = f.min(axis=0), f.max(axis=0)
    xs = np.arange(lo[0] + spacing / 2, hi[0], spacing)
    ys = np.arange(lo[1] + spacing / 2, hi[1], spacing)
    return np.array([(x, y) for x in xs for y in ys], dtype=float)


class AcousticNetwork:
    def __init__(self, params: AcousticParams, world: WorldConfig, rng: np.random.Generator) -> None:
        self.p = params
        self.nodes = node_grid(world, params.node_spacing_m)
        self.rng = rng
        self.sigma = math.radians(params.bearing_sigma_deg)

    def sense(self, t: float, intruders: list[tuple[int, Vec, Vec]]) -> list[AcousticFix]:
        fixes: list[AcousticFix] = []
        for _, pos, _ in intruders:
            d = pos[0:2] - self.nodes
            rng = np.linalg.norm(d, axis=1)
            heard = (rng < self.p.range_m) & (self.rng.random(len(self.nodes)) < self.p.detection_probability)
            if heard.sum() < self.p.min_nodes:
                continue
            noise = self.rng.normal(0.0, self.sigma, int(heard.sum()))
            bearings = np.arctan2(d[heard, 1], d[heard, 0]) + noise  # from each node towards the sound
            fix = self._triangulate(self.nodes[heard], bearings, rng[heard])
            if fix is not None:
                xy, cov = fix
                fixes.append(AcousticFix(t, xy, cov, int(heard.sum())))
        return fixes

    def _triangulate(self, nodes: Vec, bearings: Vec, ranges: Vec) -> tuple[Vec, Vec] | None:
        """Weighted least-squares intersection of bearing lines.

        Each bearing defines a line through its node; the residual is the perpendicular
        distance, with standard deviation ``range * sigma``. Ranges come from truth only to
        weight the lines, which a real network approximates from the first-pass fix.
        """
        normals = np.stack([-np.sin(bearings), np.cos(bearings)], axis=1)
        w = 1.0 / np.maximum(ranges * self.sigma, 1.0) ** 2
        a = (normals * w[:, None]).T @ normals
        b = (normals * w[:, None]).T @ np.sum(normals * nodes, axis=1)
        if np.linalg.cond(a) > 1e6:  # bearing lines nearly parallel: no usable fix
            return None
        xy = np.linalg.solve(a, b)
        return xy, np.linalg.inv(a)
