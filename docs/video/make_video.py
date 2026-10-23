"""Render the project video from the repository's own outputs.

Everything on screen comes from this repository: a closed-loop simulation run
here, the plan validator's real output, SAPIENT messages from that run, frames
from the renderer and detector, the committed benchmark results and the
committed ArduCopter SITL flight log. Nothing is drawn by hand. The context
slide quotes published sources, named on screen.

    python docs/video/make_video.py --out docs/video/kestrel_demo.mp4

Takes about 12 minutes on a 2-core machine (4 of them simulating).
"""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Callable, Iterable, Iterator
from pathlib import Path
from typing import Any

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from kestrel.config import LinkLossEvent, load_config, load_scenario
from kestrel.interop.sapient import SapientExporter, SapientOptions
from kestrel.perception.acoustic import node_grid
from kestrel.perception.camera import Camera
from kestrel.perception.classify import FlickerClassifier
from kestrel.perception.detector import SkyTargetDetector
from kestrel.perception.render import SkyRenderer
from kestrel.runtime import Frame, RunResult, plan_and_validate, run
from kestrel.sim.birds import BirdState
from kestrel.team import run_team

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
PURPLE = (150, 110, 220)
GREY = (160, 160, 150)

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
        d.text((96, 208), "Kestrel", fill=INK, font=sans(92, "Bold"))
        d.text((100, 324), "a language-tasked aerial observer for small drones", fill=INK2, font=sans(34))
        d.line([(100, 394), (100 + 380 * min(1, t / 1.2), 394)], fill=BLUE, width=4)
        for i, line in enumerate(
            [
                "plain-English missions  ·  air-to-air detection, tracking and keep-clear  ·  runtime safety monitor",
                "acoustic cueing  ·  bird/drone discrimination  ·  SAPIENT (NATO C-UAS) reports",
                "simulation-first  ·  ArduPilot over MAVLink, flown on ArduCopter SITL",
            ]
        ):
            d.text((100, 424 + 34 * i), line, fill=MUTED, font=sans(22))
        footer(d, "Simulation and SITL only. Not flown on real hardware.")
        return im

    return list(faded(hold(r, 5.0)))


CONTEXT = [
    ("About 20 Russian drones entered Polish airspace in September 2025.", "ECFR, Buzzkill (2026)"),
    ("Romania reported over 40 drone incursions in the first half of 2026.", "ECFR, Buzzkill (2026)"),
    ("Small, slow drones slip past sensors built for missiles and aircraft;", ""),
    ("birds are the classic false alarm.", "ECFR, Buzzkill (2026)"),
    ("Cheap acoustic nodes cue cameras and radar: about 9,500 fielded in Ukraine.", "US Army CALL (2026)"),
    ("NATO is standardising SAPIENT so sensors from many nations feed one picture.", "Dstl / NATO TIE"),
]


def slide_context() -> list[Image.Image]:
    def r(t: float) -> Image.Image:
        im = blank()
        d = ImageDraw.Draw(im)
        d.text((96, 80), "Why small drones, and why now", fill=BLUE, font=sans(26, "SemiBold"))
        y = 150
        for i, (line, src) in enumerate(CONTEXT):
            if t < 0.6 * i:
                continue
            d.text((96, y), line, fill=INK, font=sans(27, "Medium"))
            if src:
                d.text((100, y + 38), src, fill=MUTED, font=sans(17))
                y += 76
            else:
                y += 40
        d.text(
            (96, 610),
            "Kestrel is an observer: it detects, tracks, keeps clear and reports. No intercept or engagement.",
            fill=INK2,
            font=sans(22),
        )
        footer(d, "Sources quoted on screen; full list in docs/research.md")
        return im

    return list(faded(hold(r, 11.0)))


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


CAM_X, CAM_Y, CAM_W, CAM_H = 24, 84, 768, 576
MAP_X, MAP_Y, MAP_S = 816, 84, 440
VIEW = {"lo": np.array([-100.0, -38.0]), "span": 200.0}


def set_view(lo: tuple[float, float], span: float) -> None:
    VIEW["lo"], VIEW["span"] = np.array(lo, dtype=float), span


def _to_map(xy: np.ndarray, s: float) -> tuple[float, float]:
    lo, span = VIEW["lo"], float(VIEW["span"])  # type: ignore[arg-type]
    return ((xy[0] - lo[0]) / span * s, s - (xy[1] - lo[1]) / span * s)


def _caption(beh: str, t: float, events: dict[str, float], yielding: bool, cue_only: bool) -> str:
    t_int, t_res = events.get("interrupt", 1e9), events.get("resume", 1e9)
    if yielding:
        return "Giving way: the planned path would pass too close to a contact, so hold the standoff instead"
    if cue_only:
        return "Acoustic cue: microphones hear an aircraft the camera has not seen; it is kept clear of until seen"
    if beh == "takeoff":
        return "Take off to 25 m"
    if beh == "survey" and t < t_int:
        return "Survey the north field: lawnmower lanes, camera looking ahead"
    if t_int <= t < t_int + 6:
        return "Contact: drone detected in the camera, track confirmed and classified, survey interrupted"
    if beh == "track":
        return "Shadow from 30 m: below it so it is seen against sky, slow orbit so range becomes observable"
    if beh == "survey" and t >= t_res:
        return "Contact time over: the survey resumes where it left off, camera still watching the contact"
    if beh == "return_home":
        return "Return home"
    if beh == "land":
        return "Land and disarm"
    return beh


def _intervals(res: RunResult, start: str, end: tuple[str, ...]) -> list[tuple[float, float]]:
    out, t0 = [], None
    for e in res.events:
        if e.kind == start and t0 is None:
            t0 = e.t
        elif e.kind in end and t0 is not None:
            out.append((t0, e.t))
            t0 = None
    if t0 is not None:
        out.append((t0, 1e9))
    return out


def _draw_map(res: RunResult, a: dict[str, np.ndarray], k: int, yaw: float) -> Image.Image:
    ss = 2
    s = MAP_S * ss
    im = Image.new("RGB", (s, s), PANEL)
    d = ImageDraw.Draw(im, "RGBA")
    w = res.scenario.world
    lo, span = VIEW["lo"], float(VIEW["span"])  # type: ignore[arg-type]
    for gx in np.arange(math.ceil(lo[0] / 50) * 50, lo[0] + span + 1, 50):
        x, _ = _to_map(np.array([gx, 0.0]), s)
        d.line([(x, 0), (x, s)], fill=(*LINE, 255), width=1)
    for gy in np.arange(math.ceil(lo[1] / 50) * 50, lo[1] + span + 1, 50):
        _, y = _to_map(np.array([0.0, gy]), s)
        d.line([(0, y), (s, y)], fill=(*LINE, 255), width=1)
    for name, site in w.sites.items():
        if "__" in name:
            continue  # per-observer shares of a site, drawn as the site itself
        label = name.replace("_", " ")
        if hasattr(site, "min_xy"):
            p0 = _to_map(np.array(site.min_xy), s)
            p1 = _to_map(np.array(site.max_xy), s)
            d.rectangle([p0[0], p1[1], p1[0], p0[1]], outline=(*INK2, 140), width=3)
            d.text((p0[0] + 12, p1[1] + 8), label, fill=(*INK2, 255), font=sans(24))
        elif hasattr(site, "radius"):
            c = _to_map(np.array(site.center), s)
            r = site.radius / span * s
            d.ellipse([c[0] - r, c[1] - r, c[0] + r, c[1] + r], outline=(*INK2, 110), width=3)
            d.text((c[0] - r + 8, c[1] - 14), label, fill=(*INK2, 255), font=sans(24))
        else:
            c = _to_map(np.array(site.xy), s)
            d.polygon([(c[0], c[1] - 12), (c[0] - 10, c[1] + 8), (c[0] + 10, c[1] + 8)], fill=(*INK2, 200))
            d.text((c[0] + 14, c[1] - 10), label, fill=(*INK2, 255), font=sans(22))
    hx, hy = _to_map(np.array(w.home[:2]), s)
    d.ellipse([hx - 12, hy - 12, hx + 12, hy + 12], outline=(*INK, 255), width=4)
    d.text((hx + 18, hy - 6), "home", fill=(*INK2, 255), font=sans(24))

    if res.config.acoustic.enabled:
        for nx, ny in node_grid(w, res.config.acoustic.node_spacing_m):
            x, y = _to_map(np.array([nx, ny]), s)
            if 0 <= x <= s and 0 <= y <= s:
                d.rectangle([x - 7, y - 7, x + 7, y + 7], outline=(*PURPLE, 200), width=3)
    for cx, cy in res.log.cues[k]:
        x, y = _to_map(np.array([cx, cy]), s)
        d.ellipse([x - 26, y - 26, x + 26, y + 26], outline=(*PURPLE, 255), width=4)
    for bp in res.log.birds[k]:
        x, y = _to_map(bp, s)
        d.line([(x - 12, y - 4), (x, y + 4), (x + 12, y - 4)], fill=(*GREY, 255), width=4)

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
    r_px = 40.0 / float(VIEW["span"]) * s  # type: ignore[arg-type]
    pts = [(ox, oy)]
    for k2 in range(13):
        ang = yaw - half + 2 * half * k2 / 12
        pts.append((ox + r_px * math.cos(ang), oy - r_px * math.sin(ang)))
    d.polygon(pts, fill=(*BLUE, 46))
    d.ellipse([ox - 11, oy - 11, ox + 11, oy + 11], fill=(*BLUE, 255))
    if len(tg):
        tx, ty = _to_map(tg[-1], s)
        rs = res.config.safety.min_separation / float(VIEW["span"]) * s  # type: ignore[arg-type]
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
        p = fr.p_uav.get(ident)
        if p is None or p >= 0.6:
            col, label = AQUA, (f"drone {p:.2f}" if p is not None else f"track {ident}")
        elif p <= 0.4:
            col, label = GREY, f"bird {1 - p:.2f}"
        else:
            col, label = INK2, "unknown"
        for a0, a1, b0, b1 in ((-24, -10, 0, 0), (10, 24, 0, 0), (0, 0, -24, -10), (0, 0, 10, 24)):
            d.line([cx + a0, cy + b0, cx + a1, cy + b1], fill=col, width=3)
        d.text((cx + 28, cy - 28), label, fill=col, font=sans(18, "Medium"))
    drones = [c for c in confirmed if (fr.p_uav.get(c[0]) is None or fr.p_uav[c[0]] >= 0.6)]
    if drones:
        _, u, v = drones[0]
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
        "onboard camera (simulated)  ·  orange: detection  ·  green: drone  ·  grey: bird",
        fill=INK,
        font=sans(16),
    )


def _node_caption(res: RunResult, k: int, t: float) -> str:
    """Captions for the sensor-node flight: say what the node is doing and why."""
    log = res.log
    if log.link_ok and not log.link_ok[k]:
        return (
            "Ground link jammed: the node keeps working on board and holds its reports until the link returns"
        )
    recent = [e for e in res.events if e.t <= t]
    last = recent[-1] if recent else None
    conf = [e for e in recent if e.kind == "confirmed" and t - e.t < 6.0]
    if conf:
        detail = conf[-1].detail.split(": ", 1)[-1]
        return (
            "Confirmed: " + detail.split(", ")[0] + (", " + detail.split(", ")[1] if ", " in detail else "")
        )
    beh = log.behaviour[k]
    if beh == "investigate":
        inv = [e for e in recent if e.kind == "investigate"]
        src = inv[-1].detail.split(":")[0] if inv else "report"
        if "SAPIENT" in src:
            return (
                "C2 tasking over SAPIENT (LOOK_AT), acknowledged: go and look, camera tilted up at the column"
            )
        return f"Investigating a {src}: no altitude from sound, so close in low and look up at the column"
    if last is not None and last.kind == "tasking" and t - last.t < 4.0:
        return "Tasking received from the command node"
    if beh == "standby":
        return "Standing by on station: camera scanning, waiting for the ground sensors"
    if beh == "takeoff":
        return "Launch from the quay"
    if beh == "track":
        return "Follow the confirmed drone from 30 m: detection reports stream to the command node"
    if beh == "return_home":
        return "Return home"
    if beh == "land":
        return "Land and disarm"
    return beh


def _node_speed(res: RunResult) -> Callable[[float], float]:
    slow: list[tuple[float, float]] = []
    for e in res.events:
        if e.kind in ("investigate", "tasking"):
            slow.append((e.t - 1.0, e.t + 4.0))
        if e.kind == "confirmed":
            slow.append((e.t - 4.0, e.t + 5.0))
    jam = [
        res.log.t[i]
        for i in range(1, len(res.log.link_ok))
        if res.log.link_ok[i - 1] and not res.log.link_ok[i]
    ]
    for tj in jam[:1]:
        slow.append((tj - 1.0, tj + 5.0))

    def speed(t: float) -> float:
        return 1.0 if any(a <= t < b for a, b in slow) else 6.0

    return speed


def _speed_profile(res: RunResult, t_end: float) -> Callable[[float], float]:
    """Playback speed over flight time: real time around the moments worth watching."""
    ev = res.events
    slow: list[tuple[float, float]] = []
    t_int = next((e.t for e in ev if e.kind == "interrupt"), None)
    if t_int is not None:
        slow.append((t_int - 6.0, t_int + 10.0))
    t_yield = next((e.t for e in ev if e.kind == "yield"), None)
    if t_yield is not None:
        slow.append((t_yield - 2.0, t_yield + 6.0))
    cue_t = [res.log.t[i] for i, c in enumerate(res.log.cues) if len(c)]
    if cue_t and (t_int is None or cue_t[0] < t_int - 8.0):
        slow.append((cue_t[0] - 1.0, cue_t[0] + 5.0))
    tracking = [(e.t, e.t + 60.0) for e in ev if e.kind == "interrupt"]

    def speed(t: float) -> float:
        if any(a <= t < b for a, b in slow):
            return 1.0
        if any(a <= t < b for a, b in tracking):
            return 4.0
        return 6.0

    return speed


def flight_frames(res: RunResult, number: str = "4", node: bool = False) -> Iterator[Image.Image]:
    """Streamed, not listed: thousands of 720p frames would not fit in memory."""
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
    yields = _intervals(res, "yield", ("resume", "skip", "abort"))
    speed = _node_speed(res) if node else _speed_profile(res, t_end)
    times = [0.0]
    while times[-1] < t_end:
        times.append(times[-1] + speed(times[-1]) / FPS)
    cam_w = res.config.camera.width
    k_int = min(int(np.searchsorted(t_log, t_int)), len(t_log) - 1)
    det_range = float(np.linalg.norm(a["pos"][k_int] - a["target"][k_int]))
    total = len(times)
    for idx, t in enumerate(times, start=1):
        k = min(int(np.searchsorted(t_log, t)), len(t_log) - 1)
        j = max(0, int(np.searchsorted(ft, t, side="right")) - 1)
        fr = frames[j]
        im = blank()
        d = ImageDraw.Draw(im)
        beh = res.log.behaviour[k]
        sp = speed(t)
        badge = "×1 real time" if sp == 1.0 else f"×{sp:.0f}"
        bw = d.textlength(badge, font=sans(22, "SemiBold"))
        yielding = any(a0 <= t < a1 for a0, a1 in yields)
        no_track = all(np.isnan(a["track"][max(0, k - 20) : k + 1, 0]))
        cue_only = len(res.log.cues[k]) > 0 and no_track and t < t_int
        cap = f"{number}  " + (_node_caption(res, k, t) if node else _caption(beh, t, ev, yielding, cue_only))
        size = 26
        while size > 16 and d.textlength(cap, font=sans(size, "SemiBold")) > W - 40 - bw - 60 - CAM_X:
            size -= 1
        d.text((CAM_X, 22 + (26 - size) // 2), cap, fill=INK, font=sans(size, "SemiBold"))
        d.rounded_rectangle([W - 40 - bw - 24, 20, W - 40, 58], radius=8, fill=PANEL, outline=LINE)
        d.text((W - 40 - bw - 12, 26), badge, fill=ORANGE if sp == 1.0 else INK2, font=sans(22, "SemiBold"))
        _camera(fr, d, im, cam_w)
        im.paste(_draw_map(res, a, k, float(a["yaw"][k])), (MAP_X, MAP_Y))
        d.text(
            (MAP_X + 10, MAP_Y + 8),
            "top-down · red ring: 12 m · purple: microphones, fix · grey: birds",
            fill=INK2,
            font=sans(14),
        )
        _hud(d, res, a, k)
        if node and res.log.link_ok and not res.log.link_ok[k]:
            d.rounded_rectangle([CAM_X + 14, CAM_Y + 14, CAM_X + 330, CAM_Y + 52], radius=8, fill=RED)
            d.text(
                (CAM_X + 26, CAM_Y + 20), "LINK JAMMED · reports held", fill=INK, font=sans(20, "SemiBold")
            )
        if not node and t_int <= t < t_int + 10:
            d.text(
                (CAM_X + 14, CAM_Y + 14),
                f"first confirmed at {det_range:.0f} m",
                fill=INK,
                font=sans(20, "Medium"),
            )
        footer(d, f"closed-loop simulation · t = {t:5.1f} s · physics 400 Hz, autonomy 20 Hz, camera 10 Hz")
        yield fade(im, min(1.0, idx / 6, (total - idx + 1) / 9))


# --------------------------------------------------------------------------- sensor node


def slide_role() -> list[Image.Image]:
    boxes = [
        ("ground sensors", "microphones, radar", 70, 250),
        ("C2 / fusion node", "SAPIENT", 470, 250),
        ("Kestrel", "airborne sensor node", 870, 250),
    ]

    def r(t: float) -> Image.Image:
        im = blank()
        d = ImageDraw.Draw(im)
        d.text((70, 60), "9  The role: an airborne sensor node", fill=BLUE, font=sans(26, "SemiBold"))
        d.text(
            (70, 104),
            "Finds, tracks, identifies and reports. Goes where fixed sensors are blind. Never engages.",
            fill=INK,
            font=sans(24),
        )
        for i, (title, sub, x, y) in enumerate(boxes):
            if t < 0.5 * i:
                continue
            d.rounded_rectangle(
                [x, y, x + 330, y + 110], radius=14, fill=PANEL, outline=AQUA if i == 2 else LINE, width=3
            )
            d.text((x + 22, y + 22), title, fill=INK, font=sans(28, "SemiBold"))
            d.text((x + 22, y + 64), sub, fill=INK2, font=sans(20))
        if t > 1.2:
            for x0, x1, y, label in ((400, 470, 290, "cue"), (800, 870, 290, "task")):
                d.line([(x0, y), (x1 - 8, y)], fill=INK2, width=4)
                d.polygon([(x1, y), (x1 - 14, y - 8), (x1 - 14, y + 8)], fill=INK2)
                d.text((x0 + 4, y - 32), label, fill=INK2, font=sans(18))
            d.line([(1035, 360), (1035, 420), (635, 420), (635, 368)], fill=AQUA, width=4)
            d.polygon([(635, 360), (627, 374), (643, 374)], fill=AQUA)
            d.text(
                (700, 428),
                "detection reports: position, velocity, class, 1-sigma errors",
                fill=AQUA,
                font=sans(18),
            )
        rows = [
            "stand by on station → hear or be tasked → go and look → confirm and classify → report → follow",
            "keeps clear of every contact, birds included · keeps working through a jammed link, for a bounded time",
            "air · land · sea surface · electromagnetic: one loop, measured below",
        ]
        if t > 2.0:
            for i, line in enumerate(rows):
                d.text((70, 500 + 40 * i), line, fill=INK2, font=sans(21))
        footer(d, "Concept of operations: docs/conops.md")
        return im

    return list(faded(hold(r, 11.0)))


def team_frames(res: RunResult, number: str = "11") -> Iterator[Image.Image]:
    """Two observers, one fused picture: top-down, with the lines of sight that fix range."""
    a = res.log.arrays()
    t_log = a["t"]
    team = res.log.team_pos
    fused = res.log.fused
    tgt = a["target"]
    t_end = float(t_log[-1])
    speed = 4.0
    times = list(np.arange(0.0, t_end, speed / FPS))
    total = len(times)
    errs: list[tuple[float, float]] = []
    big = 640
    for idx, t in enumerate(times, start=1):
        k = min(int(np.searchsorted(t_log, t)), len(t_log) - 1)
        ss = 2
        sz = big * ss
        mp = Image.new("RGB", (sz, sz), PANEL)
        md = ImageDraw.Draw(mp, "RGBA")
        w = res.scenario.world
        for name, site in w.sites.items():
            if "__" in name or not hasattr(site, "min_xy"):
                continue
            p0, p1 = _to_map(np.array(site.min_xy), sz), _to_map(np.array(site.max_xy), sz)
            md.rectangle([p0[0], p1[1], p1[0], p0[1]], outline=(*INK2, 120), width=3)
        tg = tgt[max(0, k - 400) : k + 1]
        tg = tg[~np.isnan(tg[:, 0])]
        if len(tg) > 1:
            md.line([_to_map(q, sz) for q in tg[::2]], fill=(*ORANGE, 220), width=4)
        cols = [BLUE, (120, 200, 255)]
        for j in range(team[k].shape[0]):
            trail = np.array([tp[j] for tp in team[max(0, k - 600) : k + 1 : 3]])
            if len(trail) > 1:
                md.line([_to_map(q, sz) for q in trail], fill=(*cols[j % 2], 200), width=4)
        fz = fused[k]
        for q in team[k]:
            for f in fz:
                md.line([_to_map(q, sz), _to_map(f, sz)], fill=(*AQUA, 150), width=3)
        for j, q in enumerate(team[k]):
            x, y = _to_map(q, sz)
            md.ellipse([x - 12, y - 12, x + 12, y + 12], fill=(*cols[j % 2], 255))
        if len(tg):
            x, y = _to_map(tg[-1], sz)
            md.ellipse([x - 10, y - 10, x + 10, y + 10], fill=(*ORANGE, 255))
        for f in fz:
            x, y = _to_map(f, sz)
            md.ellipse([x - 20, y - 20, x + 20, y + 20], outline=(*AQUA, 255), width=4)
        if len(fz) and not np.isnan(tgt[k][0]):
            errs.append((t, float(min(np.linalg.norm(f - tgt[k]) for f in fz))))
        im = blank()
        im.paste(mp.resize((big, big), Image.Resampling.LANCZOS), (40, 60))
        d = ImageDraw.Draw(im)
        d.text((40, 18), f"{number}  Two observers, one fused track", fill=INK, font=sans(26, "SemiBold"))
        x0 = 720
        lines = [
            "Both cameras feed one tracker at the fusion node.",
            "One camera measures bearing well and range poorly:",
            "it guesses range from the drone's assumed size.",
            "Two lines of sight crossing at an angle fix range",
            "by triangulation.",
        ]
        for i, line in enumerate(lines):
            d.text((x0, 80 + 32 * i), line, fill=INK2, font=sans(21))
        gx, gy, gw, gh = x0, 270, 520, 200
        d.rectangle([gx, gy, gx + gw, gy + gh], fill=PANEL)
        d.text((gx + 10, gy + 8), "fused position error [m], this flight", fill=MUTED, font=sans(16))
        pts = [(gx + gw * tt / t_end, gy + gh - gh * min(e, 10.0) / 10.0) for tt, e in errs]
        if len(pts) > 1:
            d.line(pts, fill=AQUA, width=2)
        d.text((gx + gw - 40, gy + gh - 22), "0", fill=MUTED, font=sans(14))
        d.text((gx + gw - 40, gy + 6), "10", fill=MUTED, font=sans(14))
        if errs:
            d.text((x0, 490), f"now: {errs[-1][1]:4.1f} m", fill=AQUA, font=mono(26))
        legend = [
            ("observer 1", BLUE),
            ("observer 2", (120, 200, 255)),
            ("intruder", ORANGE),
            ("fused track", AQUA),
        ]
        for i, (lab, col) in enumerate(legend):
            d.ellipse([x0, 548 + 28 * i, x0 + 14, 562 + 28 * i], fill=col)
            d.text((x0 + 24, 542 + 28 * i), lab, fill=INK2, font=sans(18))
        footer(d, f"closed-loop simulation, two observers · t = {t:5.1f} s · ×{speed:.0f}")
        yield fade(im, min(1.0, idx / 6, (total - idx + 1) / 9))


def slide_node_results(matrix: dict[str, Any]) -> list[Image.Image]:
    node = matrix.get("sensor_node", {})
    sets = node.get("sets", [])

    def mp(dd: dict[str, float] | None) -> str:
        return "n/a" if dd is None else f"{dd['median']:.0f} s"

    def r(t: float) -> Image.Image:
        im = blank()
        d = ImageDraw.Draw(im)
        d.text((70, 50), "12  The sensor node, measured", fill=BLUE, font=sans(26, "SemiBold"))
        d.text(
            (70, 88),
            f"stand by, investigate reports, follow: the same {node.get('runs', 24)} encounters "
            f"(fresh seeds {node.get('seed0', '')}+) in each configuration",
            fill=MUTED,
            font=sans(19),
        )
        cols = [("configuration", 70), ("report → track", 470), ("entry → track", 640), ("position error", 800),
                ("inside 12 m", 970), ("held while jammed", 1100)]  # fmt: skip
        y = 140
        for name, x in cols:
            d.text((x, y), name, fill=MUTED, font=sans(17))
        y += 34
        for i, st in enumerate(sets):
            if t < 0.35 * i:
                continue
            sm = st["summary"]
            err = sm.get("track_error_all_m")
            jam = sm.get("observed_while_jammed_s")
            vals = [
                mp(sm.get("cue_to_track_s")),
                mp(sm.get("entry_to_track_s")),
                "n/a" if err is None else f"{err['median']:.1f} m",
                f"{sm['separation_violations']}/{sm['runs']}",
                "n/a" if jam is None else f"{jam:.0f} of {sm['jammed_s']:.0f} s",
            ]
            d.line([(70, y - 6), (W - 60, y - 6)], fill=LINE, width=1)
            d.text((70, y), st["label"], fill=INK, font=sans(19, "Medium"))
            for (_, x), v in zip(cols[1:], vals, strict=True):
                d.text((x, y), v, fill=INK, font=mono(19))
            y += 40
        footer(
            d,
            "medians over the runs; every number in docs/results/matrix.json, reproducible with kestrel bench",
        )
        return im

    return list(faded(hold(r, 13.0)))


COST = [
    ("airframe, autopilot, GPS, telemetry", "Holybro X500 V2 kit", "£645-785"),
    ("companion computer", "Jetson Orin Nano Super (launch price)", "$249"),
    ("camera", "Raspberry Pi HQ camera", "$50"),
    ("battery, lens, gimbal, mounts", "estimate, not sourced", "~€300"),
]


def slide_cost() -> list[Image.Image]:
    def r(t: float) -> Image.Image:
        im = blank()
        d = ImageDraw.Draw(im)
        d.text((70, 60), "13  What a node costs in parts", fill=BLUE, font=sans(26, "SemiBold"))
        for i, (what, item, price) in enumerate(COST):
            y = 130 + 54 * i
            d.text((70, y), what, fill=INK, font=sans(23, "Medium"))
            d.text((520, y), item, fill=INK2, font=sans(21))
            d.text((980, y), price, fill=INK, font=mono(22))
        d.line([(70, 352), (1210, 352)], fill=LINE, width=2)
        d.text((70, 366), "about €1,350-1,500 per node in parts", fill=AQUA, font=sans(28, "SemiBold"))
        notes = [
            "Not a product price: no engineering, certification, ground station or support included.",
            "For scale (ECFR, 2026): a Geran-type drone costs about €10,000; missiles fired at them,",
            "several hundred thousand to a million euros each. A ground acoustic node: $400-500 (US Army CALL).",
            "A sensor node replaces no interceptor; it helps make sure one is spent only on a real drone.",
        ]
        if t > 1.0:
            for i, line in enumerate(notes):
                d.text((70, 430 + 36 * i), line, fill=INK2, font=sans(21))
        footer(
            d, "Prices: retailer listings and launch announcements, October 2026; sources in docs/conops.md"
        )
        return im

    return list(faded(hold(r, 11.0)))


# --------------------------------------------------------------------------- birds


def _bird_vs_drone() -> tuple[
    list[np.ndarray], list[np.ndarray], list[float], list[float], list[float], list[float]
]:
    """A drone and a flapping bird approaching the camera, rendered and detected frame by frame."""
    cfg = load_config(ROOT / "configs" / "default.yaml")
    cam = Camera(cfg.camera, cfg.vehicle.camera_offset)
    det = SkyTargetDetector(cam, cfg.detector)
    obs = np.array([0.0, 0.0, 30.0])
    pose = cam.pose(obs, np.array([1.0, 0.0, 0.0, 0.0]), cfg.camera.pitch_up_deg)
    crops: dict[str, list[np.ndarray]] = {"drone": [], "bird": []}
    flux: dict[str, list[float]] = {"drone": [], "bird": []}
    prob: dict[str, list[float]] = {"drone": [], "bird": []}
    for kind in ("drone", "bird"):
        ren = SkyRenderer(cam, seed=21)
        clf = FlickerClassifier()
        p0, vel = obs + np.array([60.0, -4.0, 5.0]), np.array([-4.0, 0.0, 0.0])
        for k in range(60):
            t = 0.1 * k
            pos = p0 + vel * t
            if kind == "drone":
                img = ren.render(pose, [(pos, 0.35, 0.3 * t)], cfg.camera.noise_sigma)
            else:
                ph = 2 * math.pi * 4.3 * t
                b = BirdState(0, pos, vel, 0.7, 0.55 + 0.45 * math.cos(ph), 0.21 * math.sin(ph), True)
                img = ren.render(pose, [], cfg.camera.noise_sigma, birds=[b])
            uv = cam.project(pose, pos)
            assert uv is not None
            hits = [dd for dd in det.detect(img, pose) if math.hypot(dd.u - uv[0], dd.v - uv[1]) < 4]
            if hits:
                clf.add(t, hits[0].flux)
            flux[kind].append(hits[0].flux if hits else float("nan"))
            prob[kind].append(clf.p_uav())
            u0 = int(np.clip(uv[0] - 12, 0, cam.width - 24))
            v0 = int(np.clip(uv[1] - 12, 0, cam.height - 24))
            crops[kind].append(img[v0 : v0 + 24, u0 : u0 + 24])
    return crops["drone"], crops["bird"], flux["drone"], flux["bird"], prob["drone"], prob["bird"]


def slide_birds() -> list[Image.Image]:
    dc, bc, df, bf, dp, bp = _bird_vs_drone()
    n = len(dc)
    seconds = 13.0

    def panel(im: Image.Image, d: ImageDraw.ImageDraw, x: int, title: str, crop: np.ndarray, fl: list[float],
              pr: list[float], k: int) -> None:  # fmt: skip
        d.text((x, 120), title, fill=INK, font=sans(26, "SemiBold"))
        big = Image.fromarray(crop).resize((240, 240), Image.Resampling.NEAREST)
        im.paste(big, (x, 160))
        d.rectangle([x - 2, 158, x + 242, 402], outline=LINE, width=2)
        d.text((x, 408), "×10 zoom: real rendered pixels", fill=MUTED, font=sans(15))
        gx, gy, gw, gh = x + 270, 160, 300, 240
        d.rectangle([gx, gy, gx + gw, gy + gh], fill=PANEL)
        d.text((gx + 8, gy + 6), "blob darkness (flux), frame by frame", fill=MUTED, font=sans(15))
        vals = np.array(fl[: k + 1])
        top = max(np.nanmax(np.array(df + bf)), 1.0)
        pts = [(gx + gw * i / (n - 1), gy + gh - gh * 0.9 * v / top) for i, v in enumerate(vals) if v == v]
        if len(pts) > 1:
            d.line(pts, fill=BLUE, width=3)
        p = pr[k]
        if p >= 0.6:
            col, txt = AQUA, f"drone  p = {p:.2f}"
        elif p <= 0.4:
            col, txt = GREY, f"bird  p(drone) = {p:.2f}"
        else:
            col, txt = INK2, "unknown: gathering evidence"
        d.text((x, 440), txt, fill=col, font=sans(28, "SemiBold"))

    frames = []
    for i in range(int(seconds * FPS)):
        k = min(int(i / (seconds * FPS - 45) * (n - 1)), n - 1)
        im = blank()
        d = ImageDraw.Draw(im)
        d.text((60, 50), "5  Bird or drone? Watch the wing-beat", fill=BLUE, font=sans(26, "SemiBold"))
        panel(im, d, 60, "quadcopter, 0.35 m", dc[k], df, dp, k)
        panel(im, d, 660, "flapping bird, 0.7 m", bc[k], bf, bp, k)
        for j, line in enumerate(
            [
                "Spinning props blur into steady discs; a flapping wing makes the blob flicker frame to frame.",
                "The classifier scores that flicker against sensor noise. Birds are still kept clear of, just not",
                "reported as drones. A gliding bird has no wing-beat to see, and is the case this cannot catch.",
            ]
        ):
            d.text((60, 520 + 32 * j), line, fill=INK2, font=sans(21))
        footer(d, f"approaching from 60 m at 4 m/s, 10 Hz camera · t = {0.1 * k:3.1f} s")
        frames.append(im)
    return list(faded(frames))


# --------------------------------------------------------------------------- SAPIENT


def slide_sapient(messages: list[dict[str, object]]) -> list[Image.Image]:
    det = next(m for m in messages if "detection_report" in m and "classification" in m["detection_report"])  # type: ignore[operator]
    shown_msg = json.loads(json.dumps(det))
    shown_msg["detection_report"].pop("track_info", None)
    text = json.dumps(shown_msg, indent=2)
    lines = text.splitlines()
    f = mono(14)
    counts: dict[str, int] = {}
    for m in messages:
        key = next(k for k in m if k not in ("timestamp", "node_id"))
        counts[key] = counts.get(key, 0) + 1

    def r(t: float) -> Image.Image:
        im = blank()
        d = ImageDraw.Draw(im)
        d.text((60, 50), "6  Speaking NATO's sensor language: SAPIENT", fill=BLUE, font=sans(26, "SemiBold"))
        d.rounded_rectangle([60, 100, 660, 680], radius=10, fill=PANEL, outline=LINE)
        shown = int(t * 14) + 1
        for i, line in enumerate(lines[: min(shown, 37)]):
            d.text((76, 112 + 15.5 * i), line[:74], fill=INK, font=f)
        x = 700
        paras = [
            ("SAPIENT (UK Dstl, BSI Flex 335) is the message set NATO uses to plug counter-drone sensors "
             "from different nations into one fusion node; trialled at NATO's interoperability exercise.", INK),
            ("Kestrel registers two nodes, the airborne camera and the ground microphones, and reports in "
             f"ICD v7: {counts.get('detection_report', 0)} detection and {counts.get('status_report', 0)} status "
             "reports in this flight, UTM coordinates with one-sigma errors.", INK),
            ("Not yet done: protobuf wire encoding (Dstl issues the .proto files on request) and a "
             "conformance test against a SAPIENT fusion node.", INK2),
        ]  # fmt: skip
        y = 112
        for text_, col in paras:
            for line in wrap(d, text_, sans(20), W - x - 50):
                d.text((x, y), line, fill=col, font=sans(20))
                y += 30
            y += 22
        footer(d, "One detection report from the flight above · details in docs/interop.md")
        return im

    return list(faded(hold(r, 11.0)))


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
            (40, 22), "7  The same autonomy code flies a real autopilot", fill=INK, font=sans(26, "SemiBold")
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


def slide_results(matrix: dict[str, Any]) -> list[Image.Image]:
    sets = matrix["sets"]
    cols = [
        ("configuration", 96),
        ("runs inside 12 m", 420),
        ("closest", 580),
        ("detected 3 s before 30 m", 700),
        ("false alarms / h", 940),
        ("missions done", 1090),
    ]

    def row(sm: dict[str, Any]) -> list[str]:
        sep = sm["min_separation_m"]
        return [
            f"{sm['separation_violations']}/{sm['runs']}",
            f"{sep['min']:.1f} m",
            f"{sm['encounters_detected']}/{sm['encounters']}",
            f"{sm['false_alarms_per_flight_hour']:.1f}",
            f"{sm['completed_and_landed']}/{sm['runs']}",
        ]

    def r(t: float) -> Image.Image:
        im = blank()
        d = ImageDraw.Draw(im)
        d.text((96, 50), "8  Measured, with a baseline", fill=BLUE, font=sans(26, "SemiBold"))
        d.text(
            (96, 88),
            f"the same {matrix['runs']} randomised encounters (fresh seeds {matrix['seed0']}+, never used for tuning) "
            "flown in each configuration",
            fill=MUTED,
            font=sans(19),
        )
        y = 140
        for name, x in cols:
            d.text((x, y), name, fill=MUTED, font=sans(17))
        y += 34
        for i, st in enumerate(sets):
            if t < 0.35 * i:
                continue
            base = st["key"].endswith("baseline")
            d.line([(96, y - 6), (W - 60, y - 6)], fill=LINE, width=1)
            d.text((96, y), st["label"], fill=INK2 if base else INK, font=sans(19, "Medium"))
            for (_, x), v in zip(cols[1:], row(st["summary"]), strict=True):
                d.text((x, y), v, fill=INK2 if base else INK, font=mono(19))
            y += 40
        if t > 0.35 * len(sets):
            rr = matrix.get("risk_ratio")
            if rr is not None:
                d.rounded_rectangle([96, y + 14, W - 96, y + 94], radius=12, fill=PANEL, outline=AQUA)
                d.text(
                    (120, y + 28),
                    f"risk ratio {rr:.2f}: avoidance cut runs losing well clear from "
                    f"{matrix['risk_ratio_from']} to {matrix['risk_ratio_to']} of {matrix['runs']}",
                    fill=AQUA,
                    font=sans(25, "SemiBold"),
                )
                rr2 = matrix.get("risk_ratio_two")
                d.text(
                    (120, y + 62),
                    f"Two intruders: {rr2:.2f}, because one camera cannot watch both (decision 0013). "
                    "All numbers: kestrel bench; CI checks them.",
                    fill=INK2,
                    font=sans(18),
                )
        footer(d, "12 m well clear is Kestrel's own: no standard exists for drone-to-drone encounters")
        return im

    return list(faded(hold(r, 16.0)))


STANDARDS = [
    (
        "Amazon Prime Air AOT benchmark",
        "3 s track before closest range; false alarms per flight hour",
        "measured",
    ),
    (
        "Detect-and-avoid practice (ASTM F3442, MIT LL)",
        "loss of well clear; risk ratio against no avoidance",
        "measured",
    ),
    (
        "US Army CALL, acoustic detection in Ukraine",
        "2° bearings, cueing layer, altitude unknown",
        "implemented",
    ),
    ("SAPIENT ICD v7 (NATO C-UAS)", "registration, status and detection reports", "implemented"),
    ("ECFR and allied studies: birds", "bird false alarms counted, flicker classifier", "implemented"),
    ("ArduPilot", "same autonomy code flies ArduCopter SITL in CI", "implemented"),
    ("Real flight, real footage, certification", "not done; the next step", "not claimed"),
]


def slide_standards() -> list[Image.Image]:
    def r(t: float) -> Image.Image:
        im = blank()
        d = ImageDraw.Draw(im)
        d.text(
            (96, 60),
            "14  Held against published standards and field reports",
            fill=BLUE,
            font=sans(26, "SemiBold"),
        )
        for i, (src, what, status) in enumerate(STANDARDS):
            if t < 0.4 * i:
                continue
            y = 130 + 72 * i
            col = {"measured": BLUE, "implemented": AQUA, "not claimed": ORANGE}[status]
            d.rounded_rectangle([96, y, 260, y + 40], radius=8, outline=col, width=2)
            d.text((110, y + 8), status, fill=col, font=sans(18, "SemiBold"))
            d.text((290, y - 2), src, fill=INK, font=sans(22, "Medium"))
            d.text((290, y + 28), what, fill=INK2, font=sans(18))
        footer(d, "Row by row with sources: docs/research.md")
        return im

    return list(faded(hold(r, 11.0)))


NOTES = [
    (
        "Gaps between clouds were tracked as an aircraft pacing the observer",
        "darker-than-clear-sky test: 98.6% of clutter rejected",
    ),
    (
        "A 180° heading turn started a roll/pitch limit cycle and the drone fell",
        "yaw slew limit + mixer priority: 47.6° → 3.6° tilt",
    ),
    (
        "Separation failures, one cause at a time",
        "range floor, stopping distance, reachable set, yield, keep-clear",
    ),
    (
        "The renderer drew sub-pixel parts too big: drones looked detectable too far out",
        "area-true drawing; every benchmark re-run on the corrected sky",
    ),
    (
        "Acoustic cues made the drone give way to aircraft the camera already had",
        "a cue explained by a visual track is dropped; giving way has a time budget",
    ),
    (
        "The camera kept staring at where an intruder had been a minute earlier",
        "watch a stale contact for 15 s, then look where you are going: 3/24 → 1/24 runs inside 12 m",
    ),
]


def slide_notes() -> list[Image.Image]:
    def r(t: float) -> Image.Image:
        im = blank()
        d = ImageDraw.Draw(im)
        d.text(
            (96, 70), "15  What simulation caught before any hardware", fill=BLUE, font=sans(26, "SemiBold")
        )
        for i, (prob, fix) in enumerate(NOTES):
            if t < 0.5 * i:
                continue
            y = 136 + 84 * i
            d.text((96, y), prob, fill=INK, font=sans(23, "Medium"))
            d.text((126, y + 34), "→  " + fix, fill=AQUA, font=sans(20))
        footer(d, "Each is a decision record with before/after measurements in docs/decisions/")
        return im

    return list(faded(hold(r, 9.0)))


def slide_next() -> list[Image.Image]:
    rows = [
        ("Not yet", "flown on a real airframe; tested on real air-to-air footage; certified for anything"),
        (
            "Next",
            "detector evaluation on public real footage (AOT), then flights with a cooperating target drone",
        ),
        ("Wanted", "university labs and UAS teams with a test range, a target drone or real sensor data"),
    ]

    def r(t: float) -> Image.Image:
        im = blank()
        d = ImageDraw.Draw(im)
        d.text((96, 120), "16  Where this stands", fill=BLUE, font=sans(26, "SemiBold"))
        for i, (k, v) in enumerate(rows):
            if t < 0.6 * i:
                continue
            y = 200 + 110 * i
            d.text((96, y), k, fill=ORANGE if i == 0 else AQUA, font=sans(30, "Bold"))
            for j, line in enumerate(wrap(d, v, sans(26), W - 400)):
                d.text((300, y + 2 + 34 * j), line, fill=INK, font=sans(26))
        footer(d)
        return im

    return list(faded(hold(r, 8.0)))


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


NODE_INSTRUCTION = (
    "take off to 25 m, stand by at the berth for 4 minutes and investigate any acoustic cues, "
    "and if you see a drone, follow it from 30 m for 45 seconds, then return home"
)


def _with_view(
    lo: tuple[float, float], span: float, make: Callable[[], Iterable[Image.Image]]
) -> Iterator[Image.Image]:
    set_view(lo, span)
    yield from make()
    set_view((-100.0, -38.0), 200.0)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "docs" / "video" / "kestrel_demo.mp4"))
    args = ap.parse_args()
    cfg_path = ROOT / "configs" / "default.yaml"
    scen_path = ROOT / "scenarios" / "airfield.yaml"
    demo_cfg = load_config(ROOT / "configs" / "full.yaml")
    demo_scen = load_scenario(ROOT / "scenarios" / "airfield_birds.yaml")
    print("simulating the demo flight ...", flush=True)
    sapient = SapientExporter(
        SapientOptions(
            origin_lla=demo_scen.world.origin_lla, altitude_band_m=(0.0, demo_cfg.safety.max_altitude)
        )
    )
    res = run(INSTRUCTION, demo_cfg, demo_scen, record_every=1, record_downsample=1, sapient=sapient)
    print("simulating the sensor-node flight ...", flush=True)
    node_cfg = demo_cfg.model_copy(
        update={"safety": demo_cfg.safety.model_copy(update={"link_loss_action": "continue"})}
    )
    port = load_scenario(ROOT / "scenarios" / "port.yaml").model_copy(
        update={"link_loss": [LinkLossEvent(start_s=205.0, duration_s=40.0)]}
    )
    node_sapient = SapientExporter(SapientOptions(origin_lla=port.world.origin_lla, seed=port.seed))
    corners = [
        node_sapient.location(np.array([x, y, 30.0])) for x, y in [(70, 80), (110, 80), (110, 120), (70, 120)]
    ]
    task = {
        "node_id": "c2-node",
        "task": {"task_id": "C2-0001", "command": {"look_at": {"locations": corners}}},
    }
    node_res = run(
        NODE_INSTRUCTION, node_cfg, port, record_every=1, record_downsample=1, sapient=node_sapient,
        sapient_tasks=[(140.0, task)],
    )  # fmt: skip
    print("simulating two observers ...", flush=True)
    team_res = run_team(
        "take off to 25 m, survey the north field at 25 m with 30 m spacing, then return home",
        load_config(ROOT / "configs" / "default.yaml"),
        load_scenario(ROOT / "scenarios" / "airfield.yaml"),
        2,
    )
    matrix = json.loads((ROOT / "docs" / "results" / "matrix.json").read_text(encoding="utf-8"))
    writer = imageio.get_writer(
        args.out,
        fps=FPS,
        codec="libx264",
        pixelformat="yuv420p",
        ffmpeg_params=["-crf", "24", "-preset", "slow", "-movflags", "+faststart"],
    )
    parts: list[tuple[str, Callable[[], Iterable[Image.Image]]]] = [
        ("title", slide_title),
        ("context", slide_context),
        ("instruction", slide_instruction),
        ("plan", lambda: slide_plan(cfg_path, scen_path)),
        ("rejected", lambda: slide_rejected(cfg_path, scen_path)),
        ("flight", lambda: flight_frames(res)),
        ("birds", slide_birds),
        ("sapient", lambda: slide_sapient(sapient.messages)),
        ("sitl", lambda: slide_sitl(ROOT / "docs" / "results" / "sitl_flight.json")),
        ("results", lambda: slide_results(matrix)),
        ("role", slide_role),
        (
            "node flight",
            lambda: _with_view((-160.0, -160.0), 320.0, lambda: flight_frames(node_res, "10", node=True)),
        ),
        ("team", lambda: _with_view((-100.0, -38.0), 200.0, lambda: team_frames(team_res))),
        ("node results", lambda: slide_node_results(matrix)),
        ("cost", slide_cost),
        ("standards", slide_standards),
        ("notes", slide_notes),
        ("next", slide_next),
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
