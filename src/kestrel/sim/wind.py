"""Wind: constant mean plus first-order Gauss-Markov gusts (a Dryden-like spectrum)."""

from __future__ import annotations

import math

import numpy as np

from kestrel.config import WindParams
from kestrel.geometry import Vec


class Wind:
    def __init__(self, params: WindParams, rng: np.random.Generator) -> None:
        self.mean = np.array(params.mean, dtype=float)
        self.sigma = params.gust_sigma
        self.tau = params.gust_time_constant
        self.rng = rng
        self.gust = np.zeros(3)

    def step(self, dt: float) -> Vec:
        if self.sigma > 0.0:
            a = math.exp(-dt / self.tau)
            self.gust = a * self.gust + self.sigma * math.sqrt(1 - a * a) * self.rng.standard_normal(3)
            self.gust[2] *= 0.3  # vertical gusts are weaker near the ground
        return self.mean + self.gust
