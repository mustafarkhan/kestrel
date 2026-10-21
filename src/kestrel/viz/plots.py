"""Report figures (static PNG) for a simulated run."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.patches import Circle, Polygon, Rectangle

from kestrel.config import CircleSite, PointSite, RectSite
from kestrel.runtime import RunResult

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e6e5e1"
OBSERVER = "#2a78d6"  # categorical slot 1
INTRUDER = "#eb6834"  # slot 2
TRACK = "#1baf7a"  # slot 3
CRITICAL = "#d03b3b"  # status: critical, reserved for limits
BAND = "#efeeea"


def _style(ax: Axes) -> None:
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(INK_2)
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(colors=INK_2, labelsize=8)
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)


def _track_bands(ax: Axes, t: np.ndarray, beh: np.ndarray) -> None:
    on = beh == "track"
    edges = np.flatnonzero(np.diff(np.concatenate([[0], on.astype(int), [0]])))
    for a, b in zip(edges[::2], edges[1::2], strict=True):
        ax.axvspan(t[a], t[min(b, len(t) - 1)], color=BAND, lw=0, zorder=0)


def overview_map(r: RunResult, path: Path) -> None:
    a = r.log.arrays()
    w = r.scenario.world
    fig, ax = plt.subplots(figsize=(6.4, 6.4), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    _style(ax)
    ax.add_patch(Polygon(w.fence, closed=True, fill=False, ec=INK_2, lw=1.2, ls="--"))
    for name, s in w.sites.items():
        if isinstance(s, RectSite):
            ax.add_patch(
                Rectangle(
                    s.min_xy,
                    s.max_xy[0] - s.min_xy[0],
                    s.max_xy[1] - s.min_xy[1],
                    fill=False,
                    ec=GRID,
                    lw=1.5,
                )
            )
            xy = (s.min_xy[0] + 2, s.max_xy[1] - 6)
        elif isinstance(s, CircleSite):
            ax.add_patch(Circle(s.center, s.radius, fill=False, ec=GRID, lw=1.5))
            xy = (s.center[0] - s.radius * 0.6, s.center[1] + s.radius * 0.7)
        else:
            assert isinstance(s, PointSite)
            ax.plot(*s.xy, marker="^", color=INK_2, ms=6)
            xy = (s.xy[0] + 3, s.xy[1] + 3)
        ax.text(*xy, name.replace("_", " "), fontsize=8, color=INK_2)
    tg = a["target"]
    if not np.all(np.isnan(tg)):
        ax.plot(tg[:, 0], tg[:, 1], color=INTRUDER, lw=1.2, label="intruder (truth)")
    tr = a["track"]
    beh = np.array(r.log.behaviour)
    sel = (beh == "track") & ~np.isnan(tr[:, 0])
    ax.scatter(
        tr[sel, 0][::4],
        tr[sel, 1][::4],
        s=8,
        color=TRACK,
        lw=0,
        label="track estimate (while tracking)",
        zorder=3,
    )
    p = a["pos"]
    ax.plot(p[:, 0], p[:, 1], color=OBSERVER, lw=2.0, label="observer", zorder=4)
    ax.plot(*w.home[:2], marker="o", ms=8, mfc=SURFACE, mec=INK, mew=1.5, zorder=5)
    ax.text(w.home[0] + 4, w.home[1] - 8, "home", fontsize=8, color=INK)
    ax.set_aspect("equal")
    xs = [v[0] for v in w.fence]
    ys = [v[1] for v in w.fence]
    ax.set_xlim(min(xs) - 10, max(xs) + 10)
    ax.set_ylim(min(ys) - 10, max(ys) + 10)
    ax.set_xlabel("east [m]", color=INK_2, fontsize=9)
    ax.set_ylabel("north [m]", color=INK_2, fontsize=9)
    ax.set_title("Mission overview (top-down)", color=INK, fontsize=11, loc="left")
    ax.legend(loc="lower right", fontsize=8, frameon=True, facecolor=SURFACE, edgecolor=GRID)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def timeline(r: RunResult, path: Path) -> None:
    a = r.log.arrays()
    t = a["t"]
    beh = np.array(r.log.behaviour)
    lim = r.config.safety
    tg = a["target"]
    fig, axes = plt.subplots(3, 1, figsize=(8.0, 6.4), dpi=150, sharex=True)
    fig.patch.set_facecolor(SURFACE)
    for ax in axes:
        _style(ax)
        _track_bands(ax, t, beh)

    ax = axes[0]
    if not np.all(np.isnan(tg)):
        sep = np.linalg.norm(a["pos"] - tg, axis=1)
        ax.plot(t, sep, color=OBSERVER, lw=2.0)
        ax.axhline(lim.min_separation, color=CRITICAL, lw=1.2, ls="--")
        ax.text(
            t[0],
            lim.min_separation + 2,
            f"minimum separation {lim.min_separation:.0f} m",
            color=CRITICAL,
            fontsize=8,
        )
        ax.set_ylim(0, min(float(np.nanmax(sep)) * 1.05, 200))
    ax.set_ylabel("separation [m]", color=INK_2, fontsize=9)
    ax.set_title(
        "Separation to intruder, tracking error and altitude (shaded: tracking)",
        color=INK,
        fontsize=11,
        loc="left",
    )

    ax = axes[1]
    err = np.linalg.norm(a["track"] - tg, axis=1)
    on = beh == "track"
    ax.plot(t, np.where(on, err, np.nan), color=TRACK, lw=2.0)
    ax.set_ylabel("track error [m]", color=INK_2, fontsize=9)
    ax.set_ylim(bottom=0)

    ax = axes[2]
    ax.plot(t, a["pos"][:, 2], color=OBSERVER, lw=2.0)
    if not np.all(np.isnan(tg)):
        ax.plot(t, tg[:, 2], color=INTRUDER, lw=1.2)
        ax.text(
            t[-1],
            tg[-1, 2] if not np.isnan(tg[-1, 2]) else 0,
            " intruder",
            color=INK_2,
            fontsize=8,
            va="center",
        )
    ax.text(t[-1], a["pos"][-1, 2], " observer", color=INK_2, fontsize=8, va="center")
    ax.axhline(lim.max_altitude, color=INK_2, lw=0.8, ls=":")
    ax.set_ylabel("altitude [m]", color=INK_2, fontsize=9)
    ax.set_xlabel("time [s]", color=INK_2, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)
