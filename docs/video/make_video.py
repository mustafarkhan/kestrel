"""Render the project video from the repository's own outputs.

Everything on screen comes from this repository: a closed-loop simulation run
here, the plan validator's real output, the committed benchmark results and the
committed ArduCopter SITL flight log. Nothing is drawn by hand.

    python docs/video/make_video.py --out docs/video/kestrel_demo.mp4

Takes about 6 minutes on a 2-core machine (2.5 of them simulating).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from kestrel.config import load_config, load_scenario
from kestrel.runtime import Frame, RunResult, plan_and_validate, run

ROOT = Path(__file__).resolve().parents[2]
W, H, FPS = 1280, 720, 30

BG = (26, 26, 25)
PANEL = (36, 36, 34)
LINE = (62, 62, 58)
INK = (240, 239, 236)
INK2 = (195, 194, 183)
MUTED = (138, 137, 127)
BLUE = (57, 135, 229)
ORANGE = (217, 89, 38)
AQUA = (25, 158, 112)
RED = (208, 59, 59)
GREEN = (12, 163, 12)

INSTRUCTION = (
    "Take off to 25 m, survey the north field at 25 m with 30 m spacing, and if you see a drone, "
    "follow it from 30 m for 60 seconds, then return home"
)
REJECTED = "survey the lake at 20 m with 6 m spacing then hover for 25 minutes"


# --------------------------------------------------------------------------- fonts


def _font(names: list[str], size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for n in names:
        try:
            return ImageFont.truetype(n, size)
        except OSError:
            continue
    return ImageFont.load_default()


def sans(size: int, weight: str = "Regular") -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    return _font(
        [f"/usr/share/fonts/opentype/inter/Inter-{weight}.otf", f"Inter-{weight}.otf", "DejaVuSans.ttf"], size
    )


def mono(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    return _font(["DejaVuSansMono.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"], size)


# --------------------------------------------------------------------------- helpers


def blank() -> Image.Image:
    return Image.new("RGB", (W, H), BG)


def fade(img: Image.Image, k: float) -> Image.Image:
    if k >= 1.0:
        return img
    return Image.blend(Image.new("RGB", img.size, BG), img, max(0.0, k))


def faded(frames: list[Image.Image], n_in: int = 9, n_out: int = 9) -> Iterator[Image.Image]:
    n = len(frames)
    for i, f in enumerate(frames):
        yield fade(f, min(1.0, (i + 1) / n_in, (n - i) / n_out))


def hold(render: Callable[[float], Image.Image], seconds: float) -> list[Image.Image]:
    n = int(seconds * FPS)
    return [render(i / FPS) for i in range(n)]


def wrap(
    draw: ImageDraw.ImageDraw, text: str, font: ImageFont.FreeTypeFont | ImageFont.ImageFont, width: int
) -> list[str]:
    words, lines, cur = text.split(), [], ""
    for w in words:
        trial = f"{cur} {w}".strip()
        if draw.textlength(trial, font=font) <= width:
            cur = trial
        else:
            lines.append(cur)
            cur = w
    if cur:
        lines.append(cur)
    return lines


def footer(
    d: ImageDraw.ImageDraw, text: str = "Kestrel · rendered from the repository's own outputs"
) -> None:
    d.text((40, H - 34), text, fill=MUTED, font=sans(15))


# --------------------------------------------------------------------------- text slides


def slide_title() -> list[Image.Image]:
    def r(t: float) -> Image.Image:
        im = blank()
        d = ImageDraw.Draw(im)
        d.text((96, 228), "Kestrel", fill=INK, font=sans(92, "Bold"))
        d.text((100, 344), "a language-tasked aerial observer for small drones", fill=INK2, font=sans(34))
        d.line([(100, 414), (100 + 380 * min(1, t / 1.2), 414)], fill=BLUE, width=4)
        for i, line in enumerate(
            [
                "plain-English missions  ·  air-to-air detection and tracking  ·  runtime safety monitor",
                "simulation-first  ·  ArduPilot over MAVLink, flown on ArduCopter SITL",
            ]
        ):
            d.text((100, 444 + 34 * i), line, fill=MUTED, font=sans(22))
        footer(d, "Simulation and SITL only. Not flown on real hardware.")
        return im

    return list(faded(hold(r, 4.5)))


def slide_instruction() -> list[Image.Image]:
    f = sans(40, "Medium")

    def r(t: float) -> Image.Image:
        im = blank()
        d = ImageDraw.Draw(im)
        d.text((96, 120), "1  The operator says", fill=BLUE, font=sans(26, "SemiBold"))
        quoted = "“" + INSTRUCTION + "”"
        shown = quoted[: int(t * 38)]
        lines = wrap(d, quoted, f, W - 220)
        y, used = 200, 0
        for line in lines:
            take = max(0, min(len(line), len(shown) + 1 - used))
            d.text((100, y), line[:take], fill=INK, font=f)
            used += len(line) + 1
            y += 60
        if int(t * 2) % 2 == 0 and len(shown) < len(quoted):
            d.rectangle([100, y - 54, 104, y - 14], fill=INK2)
        d.text((100, 560), "No waypoints, no ground-station clicks.", fill=MUTED, font=sans(24))
        footer(d)
        return im

    return list(faded(hold(r, 6.5)))


def _plan_slide(title: str, lines: list[str], caption: str, seconds: float) -> list[Image.Image]:
    f = mono(19)

    def colour(line: str) -> tuple[int, int, int]:
        if line.startswith("ERROR") or "REJECTED" in line:
            return RED
        if "ACCEPTED" in line:
            return GREEN
        if line.startswith("warning"):
            return INK2
        return INK

    def r(t: float) -> Image.Image:
        im = blank()
        d = ImageDraw.Draw(im)
        d.text((96, 92), title, fill=BLUE, font=sans(26, "SemiBold"))
        d.rounded_rectangle([90, 150, W - 90, 520], radius=10, fill=PANEL, outline=LINE)
        n = int(t / 0.35) + 1
        y = 172
        for line in lines[:n]:
            for part in wrap(d, line, f, W - 230):
                d.text((116, y), part, fill=colour(line), font=f)
                y += 30
        for i, cl in enumerate(wrap(d, caption, sans(26), W - 200)):
            d.text((96, 560 + 36 * i), cl, fill=INK2, font=sans(26))
        footer(d)
        return im

    return list(faded(hold(r, seconds)))


def slide_plan(cfg_path: Path, scen_path: Path) -> list[Image.Image]:
    cfg, sc = load_config(cfg_path), load_scenario(scen_path)
    plan, rep = plan_and_validate(INSTRUCTION, cfg, sc)
    assert rep is not None
    lines = [f'$ kestrel plan "{INSTRUCTION[:58]}…"', f"planner: {plan.source}", *rep.summary().splitlines()]
    return _plan_slide(
        "2  It becomes a checked plan",
        lines,
        "The language layer can only write a schema-checked plan. A local LLM (Ollama) or the "
        "rule parser fills it; neither ever touches the controls.",
        7.5,
    )


def slide_rejected(cfg_path: Path, scen_path: Path) -> list[Image.Image]:
    cfg, sc = load_config(cfg_path), load_scenario(scen_path)
    plan, rep = plan_and_validate(REJECTED, cfg, sc)
    assert rep is not None
    lines = [f'$ kestrel plan "{REJECTED}"', f"planner: {plan.source}", *rep.summary().splitlines()]
    return _plan_slide(
        "3  Unsafe plans never fly",
        lines,
        "Geofence, altitude band, speed, separation standoff and battery energy are checked before "
        "takeoff. Emergency words (stop, abort, land now) bypass planning entirely.",
        7.0,
    )


# --------------------------------------------------------------------------- flight


@dataclass
class Segment:
    t0: float
    t1: float
    speed: float


CAM_X, CAM_Y, CAM_W, CAM_H = 24, 84, 768, 576
MAP_X, MAP_Y, MAP_S = 816, 84, 440
VIEW_LO = np.array([-100.0, -38.0])
VIEW_SPAN = 200.0


def _to_map(xy: np.ndarray, s: float) -> tuple[float, float]:
    return ((xy[0] - VIEW_LO[0]) / VIEW_SPAN * s, s - (xy[1] - VIEW_LO[1]) / VIEW_SPAN * s)


def _caption(beh: str, t: float, events: dict[str, float]) -> str:
    t_int, t_res = events.get("interrupt", 1e9), events.get("resume", 1e9)
    if beh == "takeoff":
        return "Take off to 25 m"
    if beh == "survey" and t < t_int:
        return "Survey the north field: lawnmower lanes, camera looking ahead"
    if t_int <= t < t_int + 6:
        return "Contact: intruder detected in the camera, track confirmed, survey interrupted"
    if beh == "track":
        return "Shadow from 30 m: below it so it is seen against sky, slow orbit so range becomes observable"
    if beh == "survey" and t >= t_res:
        return "Contact time over: the survey resumes where it left off, camera still watching the contact"
    if beh == "return_home":
        return "Return home"
    if beh == "land":
        return "Land and disarm"
    return beh


def _draw_map(res: RunResult, a: dict[str, np.ndarray], k: int, yaw: float) -> Image.Image:
    ss = 2
    s = MAP_S * ss
    im = Image.new("RGB", (s, s), PANEL)
    d = ImageDraw.Draw(im, "RGBA")
    w = res.scenario.world
    for gx in range(-100, 101, 50):
        x, _ = _to_map(np.array([gx, 0.0]), s)
        d.line([(x, 0), (x, s)], fill=(*LINE, 255), width=1)
    for gy in range(0, 161, 50):
        _, y = _to_map(np.array([0.0, gy]), s)
        d.line([(0, y), (s, y)], fill=(*LINE, 255), width=1)
    site = w.sites.get("north_field")
    if site is not None and hasattr(site, "min_xy"):
        p0 = _to_map(np.array(site.min_xy), s)
        p1 = _to_map(np.array(site.max_xy), s)
        d.rectangle([p0[0], p1[1], p1[0], p0[1]], outline=(*INK2, 140), width=3)
        d.text((p0[0] + 12, p1[1] + 8), "north field", fill=(*INK2, 255), font=sans(26))
    hx, hy = _to_map(np.array(w.home[:2]), s)
    d.ellipse([hx - 12, hy - 12, hx + 12, hy + 12], outline=(*INK, 255), width=4)
    d.text((hx + 18, hy - 6), "home", fill=(*INK2, 255), font=sans(24))

    pos = a["pos"][: k + 1]
    tg = a["target"][max(0, k - 600) : k + 1]
    tg = tg[~np.isnan(tg[:, 0])]
    if len(tg) > 1:
        d.line([_to_map(p, s) for p in tg[::2]], fill=(*ORANGE, 230), width=4)
    if len(pos) > 1:
        d.line([_to_map(p, s) for p in pos[::2]], fill=(*BLUE, 255), width=5)
    ox, oy = _to_map(pos[-1], s)
    # camera field of view wedge, 40 m long
    half = math.radians(res.config.camera.hfov_deg / 2)
    r_px = 40.0 / VIEW_SPAN * s
    pts = [(ox, oy)]
    for k2 in range(13):
        ang = yaw - half + 2 * half * k2 / 12
        pts.append((ox + r_px * math.cos(ang), oy - r_px * math.sin(ang)))
    d.polygon(pts, fill=(*BLUE, 46))
    d.ellipse([ox - 11, oy - 11, ox + 11, oy + 11], fill=(*BLUE, 255))
    if len(tg):
        tx, ty = _to_map(tg[-1], s)
        rs = res.config.safety.min_separation / VIEW_SPAN * s
        d.ellipse([tx - rs, ty - rs, tx + rs, ty + rs], outline=(*RED, 150), width=3)
        d.ellipse([tx - 9, ty - 9, tx + 9, ty + 9], fill=(*ORANGE, 255))
    trk = a["track"][k]
    if not np.isnan(trk[0]):
        x, y = _to_map(trk, s)
        d.ellipse([x - 18, y - 18, x + 18, y + 18], outline=(*AQUA, 255), width=4)
    return im.resize((MAP_S, MAP_S), Image.Resampling.LANCZOS)


def _hud(d: ImageDraw.ImageDraw, res: RunResult, a: dict[str, np.ndarray], k: int) -> None:
    x, y = MAP_X, MAP_Y + MAP_S + 16
    pos, vel = a["pos"][k], a["vel"][k]
    tg = a["target"][k]
    rng = float(np.linalg.norm(pos - tg)) if not np.isnan(tg[0]) else float("nan")
    batt = 100.0 * a["battery_wh"][k] / res.config.vehicle.battery_wh
    rows = [
        ("altitude", f"{pos[2]:5.1f} m"),
        ("speed", f"{float(np.linalg.norm(vel[0:2])):4.1f} m/s"),
        ("range to intruder", f"{rng:5.1f} m" if rng == rng else "  n/a"),
        ("battery", f"{batt:4.0f} %"),
    ]
    for i, (label, val) in enumerate(rows):
        yy = y + 26 * i
        d.text((x, yy), label, fill=MUTED, font=sans(18))
        col = RED if label == "range to intruder" and rng < res.config.safety.min_separation else INK
        d.text((x + 260, yy), val, fill=col, font=mono(18))
    recent = [s for s in res.log.interventions[max(0, k - 20) : k + 1] if s]
    tags = sorted({t for s in recent for t in s.split(",")})
    msg = "safety: " + (", ".join(t.replace("_", " ") for t in tags) if tags else "nominal")
    d.text((x, y + 26 * len(rows) + 6), msg[:46], fill=ORANGE if tags else GREEN, font=sans(18))


def _camera(fr: Frame, d: ImageDraw.ImageDraw, im: Image.Image, cam_w: int) -> None:
    sx = CAM_W / cam_w
    img = Image.fromarray(fr.image).resize((CAM_W, CAM_H), Image.Resampling.BILINEAR)
    im.paste(img, (CAM_X, CAM_Y))
    for det in fr.detections:
        x0, y0, x1, y1 = det.bbox
        d.rectangle(
            [CAM_X + x0 * sx - 10, CAM_Y + y0 * sx - 10, CAM_X + x1 * sx + 10, CAM_Y + y1 * sx + 10],
            outline=ORANGE,
            width=2,
        )
    confirmed = [
        (i, u, v) for i, u, v, c in fr.tracks_px if c and 0 <= u < cam_w and 0 <= v < fr.image.shape[0]
    ]
    for ident, u, v in confirmed:
        cx, cy = CAM_X + u * sx, CAM_Y + v * sx
        for a0, a1, b0, b1 in ((-24, -10, 0, 0), (10, 24, 0, 0), (0, 0, -24, -10), (0, 0, 10, 24)):
            d.line([cx + a0, cy + b0, cx + a1, cy + b1], fill=AQUA, width=3)
        d.text((cx + 28, cy - 28), f"track {ident}", fill=AQUA, font=sans(18, "Medium"))
    if confirmed:
        _, u, v = confirmed[0]
        half = 14
        x0 = int(np.clip(u - half, 0, cam_w - 2 * half))
        y0 = int(np.clip(v - half, 0, fr.image.shape[0] - 2 * half))
        crop = Image.fromarray(fr.image[y0 : y0 + 2 * half, x0 : x0 + 2 * half]).resize(
            (196, 196), Image.Resampling.NEAREST
        )
        bx, by = CAM_X + CAM_W - 196 - 14, CAM_Y + 14
        im.paste(crop, (bx, by))
        d.rectangle([bx - 2, by - 2, bx + 198, by + 198], outline=AQUA, width=2)
        label = "×7 zoom: real pixels"
        lw = d.textlength(label, font=sans(16))
        d.rectangle([bx - 2, by + 198, bx + lw + 10, by + 222], fill=BG)
        d.text((bx + 4, by + 201), label, fill=INK, font=sans(16))
    d.rectangle([CAM_X, CAM_Y + CAM_H - 34, CAM_X + CAM_W, CAM_Y + CAM_H], fill=(*BG,))
    d.text(
        (CAM_X + 12, CAM_Y + CAM_H - 30),
        "onboard camera (simulated)  ·  orange: detection  ·  green: confirmed track",
        fill=INK,
        font=sans(16),
    )


def flight_frames(res: RunResult) -> Iterator[Image.Image]:
    """Streamed, not listed: ~1,700 frames of 720p would not fit in memory."""
    a = res.log.arrays()
    t_log = a["t"]
    frames = res.log.frames
    ft = np.array([f.t for f in frames])
    ev: dict[str, float] = {}
    for e in res.events:
        if e.kind in ("interrupt", "resume") and e.kind not in ev:
            ev[e.kind] = e.t
    t_end = float(t_log[-1])
    t_int = ev.get("interrupt", 30.0)
    t_res = ev.get("resume", t_int + 60.0)
    segs = [
        Segment(0.0, t_int - 6.0, 5.0),
        Segment(t_int - 6.0, t_int + 12.0, 1.0),
        Segment(t_int + 12.0, t_res, 4.0),
        Segment(t_res, t_end, 6.0),
    ]
    cam_w = res.config.camera.width
    det_range = float(
        np.linalg.norm(a["pos"][np.searchsorted(t_log, t_int)] - a["target"][np.searchsorted(t_log, t_int)])
    )
    counts = [int((seg.t1 - seg.t0) / seg.speed * FPS) for seg in segs]
    total, idx = sum(counts), 0
    for seg, n in zip(segs, counts, strict=True):
        for i in range(n):
            t = seg.t0 + i * seg.speed / FPS
            k = min(int(np.searchsorted(t_log, t)), len(t_log) - 1)
            j = max(0, int(np.searchsorted(ft, t, side="right")) - 1)
            fr = frames[j]
            im = blank()
            d = ImageDraw.Draw(im)
            beh = res.log.behaviour[k]
            badge = "×1 real time" if seg.speed == 1.0 else f"×{seg.speed:.0f}"
            bw = d.textlength(badge, font=sans(22, "SemiBold"))
            cap = "4  " + _caption(beh, t, ev)
            size = 26
            while size > 16 and d.textlength(cap, font=sans(size, "SemiBold")) > W - 40 - bw - 60 - CAM_X:
                size -= 1
            d.text((CAM_X, 22 + (26 - size) // 2), cap, fill=INK, font=sans(size, "SemiBold"))
            d.rounded_rectangle([W - 40 - bw - 24, 20, W - 40, 58], radius=8, fill=PANEL, outline=LINE)
            d.text(
                (W - 40 - bw - 12, 26),
                badge,
                fill=ORANGE if seg.speed == 1.0 else INK2,
                font=sans(22, "SemiBold"),
            )
            _camera(fr, d, im, cam_w)
            im.paste(_draw_map(res, a, k, float(a["yaw"][k])), (MAP_X, MAP_Y))
            d.text(
                (MAP_X + 10, MAP_Y + 8),
                "top-down · red ring: 12 m minimum separation",
                fill=INK2,
                font=sans(15),
            )
            _hud(d, res, a, k)
            if t_int <= t < t_int + 12:
                d.text(
                    (CAM_X + 14, CAM_Y + 14),
                    f"first confirmed at {det_range:.0f} m",
                    fill=INK,
                    font=sans(20, "Medium"),
                )
            footer(
                d, f"closed-loop simulation · t = {t:5.1f} s · physics 400 Hz, autonomy 20 Hz, camera 10 Hz"
            )
            idx += 1
            yield fade(im, min(1.0, idx / 6, (total - idx + 1) / 9))


# --------------------------------------------------------------------------- SITL


def slide_sitl(path: Path) -> list[Image.Image]:
    data = json.loads(path.read_text(encoding="utf-8"))
    t = np.array(data["t"])
    p = np.array(data["pos"])
    tower = np.array(data["tower"])
    lo, span = np.array([-25.0, -25.0]), 80.0
    seconds = 15.0
    n = int(seconds * FPS)
    speed = t[-1] / (seconds - 1.5)

    def tm(xy: np.ndarray, s: float = 420) -> tuple[float, float]:
        return (xy[0] - lo[0]) / span * s, s - (xy[1] - lo[1]) / span * s

    events = [e for e in data["events"] if e["kind"] in ("behaviour", "mission")]
    # Autopilot messages are not timestamped in the log; anchor them to the phase they belong to
    # (pre-flight before arming, landing ones at touchdown) and keep the order they arrived in.
    keep = (
        "Ready",
        "EKF3 IMU0 initialised",
        "AHRS: EKF3 active",
        "Arming",
        "in-flight yaw",
        "Hit ground",
        "Disarming",
    )
    raw = [m for m in data["autopilot_messages"] if any(k in m for k in keep)]
    t_land = next((e["t"] for e in data["events"] if e["detail"] == "land"), t[-1])
    msgs: list[tuple[float, str]] = []
    armed = False
    for m in raw:
        armed = armed or "Arming" in m
        when = 0.0 if not armed else (t_land + 20.0 if ("Hit ground" in m or "Disarming" in m) else 4.0)
        msgs.append((when, m))
    frames = []
    for i in range(n):
        tt = min(i / FPS * speed, t[-1])
        k = int(np.searchsorted(t, tt))
        im = blank()
        d = ImageDraw.Draw(im)
        d.text(
            (40, 22), "5  The same autonomy code flies a real autopilot", fill=INK, font=sans(26, "SemiBold")
        )
        d.text(
            (40, 60),
            f"{data['autopilot']}  ·  MAVLink, GUIDED mode  ·  “{data['instruction']}”",
            fill=INK2,
            font=sans(18),
        )
        mp = Image.new("RGB", (420, 420), PANEL)
        md = ImageDraw.Draw(mp)
        tx, ty = tm(tower)
        md.polygon([(tx, ty - 12), (tx - 10, ty + 8), (tx + 10, ty + 8)], fill=INK2)
        md.text((tx + 14, ty - 8), "tower", fill=INK2, font=sans(18))
        hx, hy = tm(np.zeros(2))
        md.ellipse([hx - 8, hy - 8, hx + 8, hy + 8], outline=INK, width=3)
        if k > 1:
            md.line([tm(q) for q in p[: k + 1 : 2]], fill=BLUE, width=4)
        qx, qy = tm(p[min(k, len(p) - 1)])
        md.ellipse([qx - 8, qy - 8, qx + 8, qy + 8], fill=BLUE)
        im.paste(mp, (40, 110))
        d.text((40, 538), "top-down, metres from home", fill=MUTED, font=sans(16))
        # altitude trace
        ax, ay, aw, ah = 500, 110, 740, 200
        d.rectangle([ax, ay, ax + aw, ay + ah], fill=PANEL)
        d.text((ax + 10, ay + 8), "altitude [m]", fill=MUTED, font=sans(16))
        zmax = 20.0
        if k > 1:
            pts = [(ax + aw * t[j] / t[-1], ay + ah - ah * p[j, 2] / zmax) for j in range(0, k + 1, 2)]
            d.line(pts, fill=BLUE, width=3)
        d.text((ax, ay + ah + 6), f"t = {tt:5.1f} s (×{speed:.0f})", fill=INK2, font=mono(18))
        # event log
        y = 360
        d.text((ax, y), "mission executive", fill=MUTED, font=sans(16))
        for e in events:
            if e["t"] <= tt:
                y += 26
                d.text((ax, y), f"{e['t']:6.1f} s  {e['detail']}", fill=INK, font=mono(18))
        y2 = 360
        d.text((ax + 400, y2), "autopilot messages (order received)", fill=MUTED, font=sans(16))
        shown = [m for when, m in msgs if when <= tt]
        for m in shown[-6:]:
            y2 += 26
            short = m.replace(" in-flight yaw alignment complete", " yaw aligned in flight")
            d.text((ax + 400, y2), short, fill=INK2, font=mono(15))
        footer(d, "Integration test in CI: builds ArduCopter SITL and flies this mission on every push")
        frames.append(im)
    return list(faded(frames))


# --------------------------------------------------------------------------- results and notes


def slide_results(summary: dict[str, object], rows: list[dict[str, str]]) -> list[Image.Image]:
    s = summary
    sep = s["min_separation_m"]
    err = s["track_error_median_m"]
    assert isinstance(sep, dict) and isinstance(err, dict)
    longest = max(float(r["time_below_min_separation_s"] or 0) for r in rows)
    tiles = [
        (f"{s['completed_and_landed']}/{s['runs']}", "missions completed and landed"),
        (f"{s['runs_with_contact_tracked']}/{s['runs']}", "contacts detected and shadowed"),
        (f"{err['median']:.1f} m", "median track error at ~30 m"),
        (f"{100 * float(s['target_in_fov_fraction_median']):.0f}%", "intruder in view while shadowing"),  # type: ignore[arg-type]
    ]

    def r(t: float) -> Image.Image:
        im = blank()
        d = ImageDraw.Draw(im)
        d.text((96, 70), "6  Measured over 24 randomised encounters", fill=BLUE, font=sans(26, "SemiBold"))
        d.text(
            (96, 110),
            "intruder pattern and speed (2-6 m/s), altitude, position, wind 0-5 m/s, a different sky each run",
            fill=MUTED,
            font=sans(20),
        )
        for i, (big, small) in enumerate(tiles):
            if t < 0.3 * i:
                continue
            x = 96 + i * 280
            d.rounded_rectangle([x, 170, x + 260, 330], radius=12, fill=PANEL, outline=LINE)
            d.text((x + 20, 190), big, fill=INK, font=sans(54, "Bold"))
            for j, line in enumerate(wrap(d, small, sans(19), 220)):
                d.text((x + 20, 268 + 24 * j), line, fill=INK2, font=sans(19))
        if t > 1.6:
            d.rounded_rectangle([96, 370, W - 96, 500], radius=12, fill=PANEL, outline=ORANGE)
            d.text(
                (120, 390),
                f"{s['separation_violations']}/{s['runs']} runs briefly went inside the 12 m minimum separation",
                fill=ORANGE,
                font=sans(28, "SemiBold"),
            )
            d.text(
                (120, 436),
                f"at most {longest:.1f} s each, closest {sep['min']:.1f} m. Published as-is and the next thing to fix,",
                fill=INK2,
                font=sans(21),
            )
            d.text(
                (120, 464),
                "not tuned away. Causes and fixes so far: docs/decisions/0004-separation.md",
                fill=INK2,
                font=sans(21),
            )
        d.text(
            (96, 540),
            "A test fails CI if the README's numbers drift from the committed results.",
            fill=MUTED,
            font=sans(20),
        )
        footer(d)
        return im

    return list(faded(hold(r, 9.5)))


NOTES = [
    (
        "Gaps between clouds were tracked as an aircraft pacing the observer",
        "darker-than-clear-sky test: 98.6% of clutter rejected",
    ),
    (
        "A 180° heading turn started a roll/pitch limit cycle and the drone fell",
        "yaw slew limit + mixer priority: 47.6° → 3.6° tilt",
    ),
    ("13.8 m/s flown against an 8 m/s limit", "cascaded, speed-saturated position loop"),
    ("Four distinct separation failures", "range floor, stopping distance, reachable set, escape velocity"),
    ("SITL armed and disarmed in a loop", "MAVLink ids taken from the heartbeat; arm() waits for armed"),
]


def slide_notes() -> list[Image.Image]:
    def r(t: float) -> Image.Image:
        im = blank()
        d = ImageDraw.Draw(im)
        d.text(
            (96, 70), "7  What simulation caught before any hardware", fill=BLUE, font=sans(26, "SemiBold")
        )
        for i, (prob, fix) in enumerate(NOTES):
            if t < 0.5 * i:
                continue
            y = 140 + 92 * i
            d.text((96, y), prob, fill=INK, font=sans(24, "Medium"))
            d.text((126, y + 36), "→  " + fix, fill=AQUA, font=sans(21))
        footer(d, "Each is a decision record with before/after measurements in docs/decisions/")
        return im

    return list(faded(hold(r, 9.0)))


def slide_outro() -> list[Image.Image]:
    def r(t: float) -> Image.Image:
        im = blank()
        d = ImageDraw.Draw(im)
        d.text((96, 250), "Kestrel", fill=INK, font=sans(72, "Bold"))
        d.text((100, 346), "github.com/mustafarkhan/kestrel", fill=BLUE, font=sans(32, "Medium"))
        d.text(
            (100, 404),
            "Python · NumPy/SciPy · pydantic · pymavlink · ArduPilot SITL · Ollama  ·  Apache-2.0",
            fill=INK2,
            font=sans(22),
        )
        d.text((100, 446), "Simulation and SITL only; not flown on real hardware.", fill=MUTED, font=sans(22))
        return im

    return list(faded(hold(r, 5.0), 9, 15))


# --------------------------------------------------------------------------- main


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "docs" / "video" / "kestrel_demo.mp4"))
    args = ap.parse_args()
    cfg_path = ROOT / "configs" / "default.yaml"
    scen_path = ROOT / "scenarios" / "airfield.yaml"
    print("simulating the demo flight ...", flush=True)
    res = run(
        INSTRUCTION, load_config(cfg_path), load_scenario(scen_path), record_every=1, record_downsample=1
    )
    summary = json.loads((ROOT / "docs" / "results" / "bench_summary.json").read_text(encoding="utf-8"))
    with open(ROOT / "docs" / "results" / "bench_results.csv", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    writer = imageio.get_writer(
        args.out,
        fps=FPS,
        codec="libx264",
        pixelformat="yuv420p",
        ffmpeg_params=["-crf", "24", "-preset", "slow", "-movflags", "+faststart"],
    )
    parts: list[tuple[str, Callable[[], Iterable[Image.Image]]]] = [
        ("title", slide_title),
        ("instruction", slide_instruction),
        ("plan", lambda: slide_plan(cfg_path, scen_path)),
        ("rejected", lambda: slide_rejected(cfg_path, scen_path)),
        ("flight", lambda: flight_frames(res)),
        ("sitl", lambda: slide_sitl(ROOT / "docs" / "results" / "sitl_flight.json")),
        ("results", lambda: slide_results(summary, rows)),
        ("notes", slide_notes),
        ("outro", slide_outro),
    ]
    total = 0
    for name, make in parts:
        n = 0
        for f in make():
            writer.append_data(np.asarray(f))
            n += 1
        total += n
        print(f"  {name}: {n / FPS:5.1f} s", flush=True)
    writer.close()
    print(f"wrote {args.out}: {total / FPS:.1f} s")


if __name__ == "__main__":
    main()
