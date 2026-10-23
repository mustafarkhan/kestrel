"""Small airborne target detector for sky backgrounds.

Classical pipeline used in sense-and-avoid research for targets only a few
pixels wide (where CNN detectors are weakest):

1. luma conversion;
2. morphological *black top-hat* (grey closing minus image), which keeps dark
   blobs smaller than the structuring element and removes smooth sky and large
   cloud structures;
3. a horizon mask from the camera attitude. It reaches 1 deg below the true horizon,
   so same-altitude traffic (the collision-course case, sitting on the horizon) is
   searched; the background there is haze at kilometre range. Clutter further down
   (treelines, buildings) is a different and much harder problem this detector
   does not claim to solve;
4. a robust z-score threshold (median / MAD), so the threshold adapts to how
   cluttered the current sky is;
5. connected components with area, aspect and minimum-contrast gates;
6. a *darker-than-clear-sky* test: a backlit aircraft is darker than clear sky
   at the same elevation, whereas a gap between clouds *is* clear sky. Without
   this, cloud gaps (which sit at infinity, so keep a constant bearing while the
   observer moves) get tracked as an aircraft pacing the observer. The cost is
   that a sunlit target brighter than the sky is missed: a stated limitation.

A learned detector (e.g. YOLO on Jetson) can replace this behind the same
:class:`Detection` interface; the tracker does not care where detections come from.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from kestrel.config import DetectorParams
from kestrel.perception.camera import Camera, CameraPose


@dataclass(frozen=True)
class Detection:
    u: float
    v: float
    size_px: float  # blur-corrected apparent diameter
    score: float  # peak robust z-score
    bbox: tuple[int, int, int, int]  # x0, y0, x1, y1 (inclusive-exclusive)
    flux: float = 0.0  # integrated top-hat response: silhouette area times contrast


def to_gray(rgb: np.ndarray) -> np.ndarray:
    f = rgb.astype(np.float32)
    out: np.ndarray = f[..., 0] * 0.299 + f[..., 1] * 0.587 + f[..., 2] * 0.114
    return out


class SkyTargetDetector:
    def __init__(self, camera: Camera, params: DetectorParams) -> None:
        self.cam = camera
        self.p = params
        self._rows = np.arange(camera.height)[:, None]
        self._margin_px = camera.fy * math.tan(math.radians(params.horizon_margin_deg))
        self.last_response: np.ndarray | None = None

    def sky_mask(self, pose: CameraPose) -> np.ndarray:
        w = self.cam.width
        h0 = self.cam.horizon_row(pose, 0.0)
        h1 = self.cam.horizon_row(pose, float(w - 1))
        if not (math.isfinite(h0) and math.isfinite(h1)):
            # camera looking straight up or down: all sky or all ground
            return np.full((self.cam.height, w), h0 == math.inf or h1 == math.inf)
        cols = np.linspace(h0, h1, w)[None, :] - self._margin_px
        # which side is sky? test the pixel straight above the centre of the horizon line
        mid = 0.5 * (h0 + h1)
        ray = self.cam.pixel_ray_world(pose, (w - 1) / 2, mid - 10.0)
        above_is_sky = ray[2] > 0
        return self._rows < cols if above_is_sky else self._rows > cols + 2 * self._margin_px

    def detect(self, rgb: np.ndarray, pose: CameraPose) -> list[Detection]:
        p = self.p
        gray = to_gray(rgb)
        closed = ndimage.grey_closing(gray, size=(p.tophat_size, p.tophat_size))
        resp = closed - gray
        mask = self.sky_mask(pose)
        # do not trust the top-hat near the image border or the horizon line
        mask = ndimage.binary_erosion(mask, iterations=p.tophat_size // 2, border_value=0)
        self.last_response = np.where(mask, resp, 0.0)
        if mask.sum() < 100:
            return []
        vals = resp[mask]
        med = float(np.median(vals))
        mad = float(np.median(np.abs(vals - med))) * 1.4826 + 0.5
        z = (resp - med) / mad
        hits = (z > p.threshold_k) & mask & (resp > 6.0)
        if not hits.any():
            return []
        labels, _ = ndimage.label(hits, structure=np.ones((3, 3)))
        out: list[Detection] = []
        for idx, sl in enumerate(ndimage.find_objects(labels), start=1):
            if sl is None:
                continue
            region = labels[sl] == idx
            area = int(region.sum())
            if not (p.min_area_px <= area <= p.max_area_px):
                continue
            hgt, wid = region.shape
            if max(hgt, wid) / max(1, min(hgt, wid)) > p.max_aspect:
                continue
            r = resp[sl] * region
            peak = float(r.max())
            if peak < p.min_contrast:
                continue
            if self._darkness(gray, mask, sl) < p.sky_darkness_min:
                continue
            size_px = self._size_from_moments(resp, sl, peak)
            ys, xs = np.nonzero(region)
            wts = r[ys, xs]
            out.append(
                Detection(
                    u=float(sl[1].start + (xs * wts).sum() / wts.sum()),
                    v=float(sl[0].start + (ys * wts).sum() / wts.sum()),
                    size_px=size_px,
                    score=float(z[sl][region].max()),
                    bbox=(sl[1].start, sl[0].start, sl[1].stop, sl[0].stop),
                    flux=float(wts.sum()),
                )
            )
        out.sort(key=lambda d: -d.score)
        return out[:20]

    @staticmethod
    def _darkness(gray: np.ndarray, mask: np.ndarray, sl: tuple[slice, slice]) -> float:
        """How much darker the blob is than clear sky in the same rows (grey levels)."""
        y0 = max(sl[0].start - 12, 0)
        y1 = sl[0].stop + 12
        band = gray[y0:y1][mask[y0:y1]]
        if band.size < 50:
            return 0.0
        clear_sky = float(np.percentile(band, 10))
        return clear_sky - float(gray[sl].min())

    def _size_from_moments(self, resp: np.ndarray, sl: tuple[slice, slice], peak: float) -> float:
        """Apparent diameter from the second moment of the top-hat response.

        Contrast-invariant (unlike integrated darkness), so it transfers across
        lighting. The linear map from blob spread to diameter absorbs optical blur
        and is calibrated in simulation (``size_gain``/``size_offset``); on a real
        camera it must be recalibrated, which ``tests/test_perception.py`` documents.
        """
        y0 = max(sl[0].start - 2, 0)
        x0 = max(sl[1].start - 2, 0)
        sub = resp[y0 : sl[0].stop + 2, x0 : sl[1].stop + 2]
        w = np.clip(sub - 0.2 * peak, 0.0, None)
        total = float(w.sum())
        if total <= 0.0:
            return 1.0
        ys, xs = np.mgrid[0 : sub.shape[0], 0 : sub.shape[1]]
        mx = float((xs * w).sum()) / total
        my = float((ys * w).sum()) / total
        spread = math.sqrt(float((((xs - mx) ** 2 + (ys - my) ** 2) * w).sum()) / total)
        return max(self.p.size_gain * spread + self.p.size_offset, 0.5)
