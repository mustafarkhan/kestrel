"""Demo animation: onboard camera with detections next to a live top-down map."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from kestrel.runtime import RunResult

_SURFACE = (252, 252, 251)
_INK = (11, 11, 11)
_INK2 = (82, 81, 78)
_GRID = (220, 219, 214)
_OBS = (42, 120, 214)
_INT = (235, 104, 52)
_TRK = (27, 175, 122)

CAM_W, CAM_H = 480, 360
MAP = 360
HEADER = 34


def _font(size: int) -> ImageFont.ImageFont | ImageFont.FreeTypeFont:
    for name in ("DejaVuSans.ttf", "Arial.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def make_gif(
    r: RunResult,
    path: Path,
    t_start: float | None = None,
    t_end: float | None = None,
    fps: int = 10,
    width_px: int = 680,
) -> int:
    frames = [
        f for f in r.log.frames if (t_start is None or f.t >= t_start) and (t_end is None or f.t <= t_end)
    ]
    if not frames:
        return 0
    a = r.log.arrays()
    t_log = a["t"]
    fence = np.array(r.scenario.world.fence)
    lo, hi = fence.min(axis=0) - 5, fence.max(axis=0) + 5
    scale = (MAP - 20) / float(max(hi - lo))

    def to_map(xy: np.ndarray) -> tuple[float, float]:
        return CAM_W + 10 + (xy[0] - lo[0]) * scale, HEADER + MAP - 10 - (xy[1] - lo[1]) * scale

    f_big, f_small = _font(15), _font(12)
    images = []
    for fr in frames:
        k = int(np.searchsorted(t_log, fr.t))
        k = min(k, len(t_log) - 1)
        canvas = Image.new("RGB", (CAM_W + MAP + 10, HEADER + max(CAM_H, MAP)), _SURFACE)
        d = ImageDraw.Draw(canvas)
        cam = Image.fromarray(fr.image).resize((CAM_W, CAM_H), Image.Resampling.NEAREST)
        canvas.paste(cam, (0, HEADER))
        sx = CAM_W / r.config.camera.width  # detections are in full-resolution pixels
        for det in fr.detections:
            x0, y0, x1, y1 = det.bbox
            d.rectangle(
                [x0 * sx - 6, HEADER + y0 * sx - 6, x1 * sx + 6, HEADER + y1 * sx + 6], outline=_INT, width=2
            )
        for _, u, v, confirmed in fr.tracks_px:
            cx, cy = u * sx, HEADER + v * sx
            if confirmed and 0 <= cx < CAM_W and HEADER <= cy < HEADER + CAM_H:
                d.line([cx - 14, cy, cx - 6, cy], fill=_TRK, width=2)
                d.line([cx + 6, cy, cx + 14, cy], fill=_TRK, width=2)
                d.line([cx, cy - 14, cx, cy - 6], fill=_TRK, width=2)
                d.line([cx, cy + 6, cx, cy + 14], fill=_TRK, width=2)

        # map panel
        pts = [to_map(p) for p in fence] + [to_map(fence[0])]
        d.line(pts, fill=_GRID, width=2)
        tg = a["target"][: k + 1]
        tg = tg[~np.isnan(tg[:, 0])]
        if len(tg) > 1:
            d.line([to_map(p) for p in tg[-600:]], fill=_INT, width=2)
        obs = a["pos"][: k + 1]
        if len(obs) > 1:
            d.line([to_map(p) for p in obs[::2]], fill=_OBS, width=3)
        ox, oy = to_map(obs[-1])
        d.ellipse([ox - 5, oy - 5, ox + 5, oy + 5], fill=_OBS)
        if len(tg):
            tx, ty = to_map(tg[-1])
            d.ellipse([tx - 4, ty - 4, tx + 4, ty + 4], fill=_INT)
        trk = a["track"][k]
        if not np.isnan(trk[0]):
            x, y = to_map(trk)
            d.ellipse([x - 6, y - 6, x + 6, y + 6], outline=_TRK, width=2)

        sep = (
            float(np.linalg.norm(a["pos"][k] - a["target"][k]))
            if not np.isnan(a["target"][k, 0])
            else float("nan")
        )
        title = f"t = {fr.t:5.1f} s   behaviour: {fr.behaviour}"
        if sep == sep:
            title += f"   range to intruder: {sep:4.1f} m"
        d.text((10, 8), title, fill=_INK, font=f_big)
        d.text((CAM_W + 16, HEADER + 6), "top-down", fill=_INK2, font=f_small)
        d.text(
            (10, HEADER + CAM_H - 20),
            "onboard camera  (box: detection, cross: confirmed track)",
            fill=(250, 250, 250),
            font=f_small,
        )
        size = (width_px, round(width_px * canvas.height / canvas.width))
        images.append(canvas.resize(size, Image.Resampling.LANCZOS))
    # One shared palette with the accent colours reserved, so thin coloured lines survive quantisation.
    sample = images[len(images) // 2].copy()
    sw = ImageDraw.Draw(sample)
    for i, col in enumerate((_OBS, _INT, _TRK, _INK, _INK2, _SURFACE)):
        sw.rectangle([i * 24, 0, i * 24 + 23, 23], fill=col)
    palette = sample.quantize(colors=64, method=Image.Quantize.MEDIANCUT)
    frames_p = [im.quantize(palette=palette, dither=Image.Dither.NONE) for im in images]
    frames_p[0].save(
        path, save_all=True, append_images=frames_p[1:], duration=int(1000 / fps), loop=0, optimize=True
    )
    return len(frames_p)
