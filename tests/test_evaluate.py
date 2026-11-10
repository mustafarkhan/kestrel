"""The real-data evaluation harness, exercised on rendered frames written in each layout."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from kestrel.cli import main
from kestrel.config import CameraParams, Config
from kestrel.geometry import quat_from_euler
from kestrel.perception.camera import Camera
from kestrel.perception.evaluate import READERS, evaluate
from kestrel.perception.render import SkyRenderer

Box = tuple[float, float, float, float]


def _frames(band: str, n: int = 12) -> list[tuple[np.ndarray, Box | None]]:
    camp = CameraParams(band=band) if band == "thermal" else CameraParams()
    cam = Camera(camp, (0.0, 0.0, 0.0))
    ren = SkyRenderer(cam, seed=3, band=band)
    rng = np.random.default_rng(4)
    out: list[tuple[np.ndarray, Box | None]] = []
    for k in range(n):
        yaw = rng.uniform(-math.pi, math.pi)
        pose = cam.pose(np.array([0, 0, 20.0]), quat_from_euler(0, 0, yaw), 30.0)  # all sky
        if k % 4 == 3:
            out.append((ren.render(pose, [], camp.noise_sigma), None))
            continue
        el = math.radians(30.0 + rng.uniform(-5, 5))
        tgt = np.array([0, 0, 20.0]) + 25.0 * np.array(
            [math.cos(yaw) * math.cos(el), math.sin(yaw) * math.cos(el), math.sin(el)]
        )
        u, v, depth = cam.project(pose, tgt)  # type: ignore[misc]
        r = cam.fx * 0.35 / 2 / depth + 1.0
        out.append((ren.render(pose, [(tgt, 0.35, 0.3)], camp.noise_sigma), (u - r, v - r, 2 * r, 2 * r)))
    return out


def _write(fmt: str, root: Path, frames: list[tuple[np.ndarray, Box | None]]) -> Path:
    if fmt == "anti-uav":
        seq = root / "seq01"
        seq.mkdir(parents=True)
        for k, (img, _) in enumerate(frames):
            Image.fromarray(img).save(seq / f"{k:06d}.jpg", quality=95)
        rects = [list(b) if b else [] for _, b in frames]
        (seq / "IR_label.json").write_text(
            json.dumps({"exist": [int(b is not None) for _, b in frames], "gt_rect": rects})
        )
    elif fmt == "yolo":
        (root / "images").mkdir(parents=True)
        (root / "labels").mkdir()
        for k, (img, b) in enumerate(frames):
            Image.fromarray(img).save(root / "images" / f"{k:06d}.png")
            h, w = img.shape[:2]
            line = (
                ""
                if b is None
                else f"0 {(b[0] + b[2] / 2) / w} {(b[1] + b[3] / 2) / h} {b[2] / w} {b[3] / h}\n"
            )
            (root / "labels" / f"{k:06d}.txt").write_text(line)
    else:
        root.mkdir(parents=True)
        lines = []
        for k, (img, b) in enumerate(frames):
            Image.fromarray(img).save(root / f"{k + 1:06d}.png")
            if b is not None:
                lines.append(f"{k + 1},1,{b[0]},{b[1]},{b[2]},{b[3]},1,-1,-1,-1")
        (root / "gt.txt").write_text("\n".join(lines))
    return root


@pytest.mark.parametrize("fmt", ["anti-uav", "yolo", "mot"])
def test_each_layout_reads_and_scores(tmp_path: Path, fmt: str) -> None:
    root = _write(fmt, tmp_path / fmt, _frames("visible"))
    seqs = READERS[fmt](root)
    assert sum(len(s.frames) for s in seqs) == 12
    assert sum(len(f.boxes) for s in seqs for f in s.frames) == 9
    res = evaluate(seqs).summary()
    assert res["targets"] == 9
    assert res["recall"] is not None and res["recall"] > 0.8
    assert res["false_per_frame"] is not None and res["false_per_frame"] < 0.2
    assert res["targets_by_size"]["tiny"] == 9  # a 0.35 m drone at 25 m is under 10 px


def test_thermal_footage_needs_the_bright_polarity(tmp_path: Path) -> None:
    root = _write("anti-uav", tmp_path / "ir", _frames("thermal"))
    seqs = READERS["anti-uav"](root)
    assert evaluate(seqs, polarity="bright").summary()["recall"] > 0.8  # type: ignore[operator]
    assert (evaluate(seqs, polarity="dark").summary()["recall"] or 0.0) < 0.2


def test_cli_writes_a_report(tmp_path: Path, cfg: Config) -> None:
    root = _write("yolo", tmp_path / "set", _frames("visible", 4))
    out = tmp_path / "report.json"
    assert main(["evaluate", str(root), "--format", "yolo", "--out", str(out)]) == 0
    assert json.loads(out.read_text())["frames"] == 4
