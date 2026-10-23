"""Bird or drone? Wing-beat flicker in a track's brightness history.

At the ranges where both are a few pixels across, shape says little; motion over time
says more. A flapping wing sweeps in and out of the silhouette several times a second,
so the blob's integrated darkness (the detector's ``flux``) jumps from frame to frame.
A multirotor's spinning props blur into steady discs, so its flux changes only by
sensor noise and slowly with range. At a 10 Hz frame rate a 3–6 Hz wing-beat is near or
above the Nyquist rate, so the method measures the *size* of frame-to-frame changes,
which survives aliasing, rather than a frequency.

Each consecutive pair of detections gives a change in log flux. Sensor noise alone makes
that change roughly ``c * sqrt(1/f1 + 1/f2)`` for fluxes ``f1, f2`` (fainter blobs are
noisier); ``c`` was fitted on drones in simulation. Dividing by it gives a normalised
flicker that is about 1 for a steady target whatever its range. The classifier's score
is the RMS of the normalised changes over the recent track history, clipped so that one
cloud edge cannot decide the class.

Calibration (``docs/decisions/0009-birds.md``): on 15-frame windows from a separate
calibration set, a threshold of 2.0 kept every drone and rejected 82% of flapping birds
and 5% of gliding ones. A gliding bird has no wing-beat to see, so the classifier cannot
reject it; that is a property of the cue, not a tuning problem.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

NOISE_C = 1.57  # log-flux noise coefficient, fitted on simulated drones
THRESHOLD = 2.0  # normalised RMS flicker at which bird and drone are equally likely
SLOPE = 3.0  # logistic slope per unit of flicker, at the reference sample count
REF_PAIRS = 14
MIN_PAIRS = 8  # fewer than this: no opinion
WINDOW = 40  # pairs kept
MAX_GAP_S = 0.15  # only consecutive frames form a pair
CLIP = 6.0
# Below this integrated darkness, sensor noise in the log-flux change (about 0.22 at 100)
# is as large as the wing-beat modulation of a bird seen side-on, so the cue cannot be
# resolved and the classifier withholds an opinion rather than calling a faint bird a drone.
MIN_FLUX = 100.0


@dataclass
class FlickerClassifier:
    samples: list[tuple[float, float]] = field(default_factory=list)  # (t, flux)
    norm: list[float] = field(default_factory=list)  # normalised log-flux changes
    recent_flux: list[float] = field(default_factory=list)

    def add(self, t: float, flux: float) -> None:
        if flux <= 0.0:
            return
        self.recent_flux.append(flux)
        if len(self.recent_flux) > WINDOW:
            self.recent_flux.pop(0)
        if self.samples and 0.0 < t - self.samples[-1][0] <= MAX_GAP_S:
            f0 = self.samples[-1][1]
            change = math.log(flux / f0) / (NOISE_C * math.sqrt(1.0 / flux + 1.0 / f0))
            self.norm.append(max(-CLIP, min(CLIP, change)))
            if len(self.norm) > WINDOW:
                self.norm.pop(0)
        self.samples.append((t, flux))
        if len(self.samples) > 2:
            self.samples.pop(0)

    @property
    def pairs(self) -> int:
        return len(self.norm)

    def flicker(self) -> float | None:
        if len(self.norm) < MIN_PAIRS:
            return None
        return math.sqrt(sum(x * x for x in self.norm) / len(self.norm))

    def p_uav(self) -> float:
        """Probability the object is a drone. 0.5 (no opinion) until there is enough history,
        and while the object is too faint for a wing-beat to be resolved."""
        f = self.flicker()
        if f is None or sorted(self.recent_flux)[len(self.recent_flux) // 2] < MIN_FLUX:
            return 0.5
        k = SLOPE * math.sqrt(len(self.norm) / REF_PAIRS)
        return 1.0 / (1.0 + math.exp(k * (f - THRESHOLD)))
