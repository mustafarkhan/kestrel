"""Polygonal geofence with signed distance and projection to the safe interior."""

from __future__ import annotations

import math

import numpy as np

from kestrel.geometry import Vec


class Geofence:
    def __init__(self, vertices: list[tuple[float, float]]) -> None:
        v = np.asarray(vertices, dtype=float)
        if len(v) < 3:
            raise ValueError("geofence needs at least three vertices")
        area2 = float(np.sum(v[:, 0] * np.roll(v[:, 1], -1) - np.roll(v[:, 0], -1) * v[:, 1]))
        self.vertices = v if area2 > 0 else v[::-1]  # store counter-clockwise

    def contains(self, xy: Vec) -> bool:
        x, y = float(xy[0]), float(xy[1])
        inside = False
        v = self.vertices
        j = len(v) - 1
        for i in range(len(v)):
            xi, yi = v[i]
            xj, yj = v[j]
            if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
                inside = not inside
            j = i
        return inside

    def _nearest_on_boundary(self, xy: Vec) -> tuple[float, Vec]:
        best_d, best_p = math.inf, np.zeros(2)
        v = self.vertices
        for i in range(len(v)):
            a, b = v[i], v[(i + 1) % len(v)]
            ab = b - a
            t = float(np.clip((xy - a) @ ab / (ab @ ab), 0.0, 1.0))
            p = a + t * ab
            d = float(np.linalg.norm(xy - p))
            if d < best_d:
                best_d, best_p = d, p
        return best_d, best_p

    def signed_distance(self, xy: Vec) -> float:
        """Positive inside the fence, negative outside, metres to the boundary."""
        xy2 = np.asarray(xy[:2], dtype=float)
        d, _ = self._nearest_on_boundary(xy2)
        return d if self.contains(xy2) else -d

    def project_inside(self, xy: Vec, margin: float) -> Vec:
        """Closest point that is at least ``margin`` inside the fence.

        Iterative push along the boundary normal; exact for convex fences and
        conservative for mildly concave ones.
        """
        p = np.asarray(xy[:2], dtype=float).copy()
        centroid = self.vertices.mean(axis=0)
        for _ in range(12):
            sd = self.signed_distance(p)
            if sd >= margin - 1e-6:
                return p
            _, q = self._nearest_on_boundary(p)
            inward = q - p if sd < 0 else p - q
            norm = float(np.linalg.norm(inward))
            if norm < 1e-9:  # exactly on the boundary: head for the centroid
                inward = (centroid - p) / max(float(np.linalg.norm(centroid - p)), 1e-9)
            else:
                inward = inward / norm
            p = q + inward * margin if sd < 0 else p + inward * (margin - sd)
        return p
