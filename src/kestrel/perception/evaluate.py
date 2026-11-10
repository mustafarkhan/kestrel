"""Evaluate the detector on recorded footage with ground-truth boxes.

Everything else in Kestrel is measured on rendered frames. This module runs the same
detector on real frames from public air-to-air and anti-UAV sets, so its numbers can be
checked against the field. Three common annotation layouts are read:

* ``anti-uav``: a folder per sequence holding the frames and a JSON file with ``exist`` (0/1
  per frame) and ``gt_rect`` (``[x, y, w, h]`` per frame, empty when absent), as the Anti-UAV
  challenges and Anti-UAV410 publish them;
* ``yolo``: ``images/`` and ``labels/`` folders, one text file per image with normalised
  ``class cx cy w h`` lines;
* ``mot``: a folder of frames and a ``gt.txt`` with ``frame,id,x,y,w,h,...`` lines (frames
  numbered from 1).

Recorded footage has no camera attitude, so the whole frame is searched: ground clutter that
the horizon mask removes in flight is counted here. A detection matches a box if its centre
lies inside the box grown by ``match_margin_px``. Recall is reported by target size, using
the box diagonal and the bands the CST Anti-UAV benchmark uses (tiny up to 10 px, small up
to 30, normal up to 50, large above).
"""

from __future__ import annotations

import json
import math
from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from kestrel.config import CameraParams, DetectorParams
from kestrel.perception.camera import Camera
from kestrel.perception.detector import SkyTargetDetector

Box = tuple[float, float, float, float]  # x, y, w, h in pixels
IMAGE_EXT = {".jpg", ".jpeg", ".png", ".bmp"}
SIZE_BANDS = (("tiny", 0.0, 10.0), ("small", 10.0, 30.0), ("normal", 30.0, 50.0), ("large", 50.0, math.inf))


@dataclass
class Frame:
    path: Path
    boxes: list[Box]


@dataclass
class Sequence:
    name: str
    frames: list[Frame]


def _images(folder: Path) -> list[Path]:
    return sorted(p for p in folder.iterdir() if p.suffix.lower() in IMAGE_EXT)


def read_anti_uav(root: Path) -> list[Sequence]:
    seqs = []
    for folder in sorted(d for d in [root, *root.iterdir()] if d.is_dir()):
        labels = sorted(folder.glob("*.json"))
        imgs = _images(folder)
        if not labels or not imgs:
            continue
        data = json.loads(labels[0].read_text(encoding="utf-8"))
        rects = data.get("gt_rect") or []
        exist = data.get("exist") or [1 if r else 0 for r in rects]
        frames = []
        for k, img in enumerate(imgs):
            r = rects[k] if k < len(rects) else []
            ok = k < len(exist) and exist[k] and isinstance(r, list) and len(r) == 4
            frames.append(Frame(img, [tuple(map(float, r))] if ok else []))  # type: ignore[list-item]
        seqs.append(Sequence(folder.name, frames))
    return seqs


def read_yolo(root: Path) -> list[Sequence]:
    frames = []
    for img in _images(root / "images"):
        w, h = Image.open(img).size
        lab = root / "labels" / (img.stem + ".txt")
        boxes: list[Box] = []
        if lab.exists():
            for line in lab.read_text(encoding="utf-8").split("\n"):
                parts = line.split()
                if len(parts) >= 5:
                    cx, cy, bw, bh = (float(v) for v in parts[1:5])
                    boxes.append(((cx - bw / 2) * w, (cy - bh / 2) * h, bw * w, bh * h))
        frames.append(Frame(img, boxes))
    return [Sequence(root.name, frames)]


def read_mot(root: Path) -> list[Sequence]:
    imgs = _images(root)
    boxes: dict[int, list[Box]] = {}
    gt = root / "gt.txt"
    for line in gt.read_text(encoding="utf-8").split("\n") if gt.exists() else []:
        parts = line.replace(" ", "").split(",")
        if len(parts) >= 6:
            boxes.setdefault(int(float(parts[0])), []).append(tuple(float(v) for v in parts[2:6]))  # type: ignore[arg-type]
    return [Sequence(root.name, [Frame(p, boxes.get(k + 1, [])) for k, p in enumerate(imgs)])]


READERS = {"anti-uav": read_anti_uav, "yolo": read_yolo, "mot": read_mot}


def _size_band(box: Box) -> str:
    diag = math.hypot(box[2], box[3])
    return next(name for name, _, hi in SIZE_BANDS if diag <= hi)


@dataclass
class EvalResult:
    frames: int = 0
    targets: int = 0
    found: int = 0
    detections: int = 0
    false: int = 0
    by_size: dict[str, list[int]] = field(default_factory=lambda: {n: [0, 0] for n, _, _ in SIZE_BANDS})
    per_sequence: dict[str, dict[str, float]] = field(default_factory=dict)

    def summary(self) -> dict[str, Any]:
        def ratio(a: int, b: int) -> float | None:
            return round(a / b, 3) if b else None

        return {
            "frames": self.frames,
            "targets": self.targets,
            "recall": ratio(self.found, self.targets),
            "precision": ratio(self.detections - self.false, self.detections),
            "false_per_frame": round(self.false / self.frames, 3) if self.frames else None,
            "recall_by_size": {n: ratio(f, t) for n, (f, t) in self.by_size.items()},
            "targets_by_size": {n: t for n, (_, t) in self.by_size.items()},
            "per_sequence": self.per_sequence,
        }


def iterate(seqs: list[Sequence], every: int = 1) -> Iterator[tuple[Sequence, Frame]]:
    for seq in seqs:
        for k, fr in enumerate(seq.frames):
            if k % every == 0:
                yield seq, fr


def evaluate(
    seqs: list[Sequence],
    params: DetectorParams | None = None,
    polarity: str = "dark",
    every: int = 1,
    match_margin_px: float = 3.0,
) -> EvalResult:
    """Run the detector on every ``every``-th frame and score it against the boxes."""
    res = EvalResult()
    detectors: dict[tuple[int, int], SkyTargetDetector] = {}
    seq_stats: dict[str, list[int]] = {}
    for seq, fr in iterate(seqs, every):
        rgb = np.asarray(Image.open(fr.path).convert("RGB"))
        h, w = rgb.shape[:2]
        det = detectors.get((w, h))
        if det is None:
            cam = Camera(CameraParams(width=w, height=h), (0.0, 0.0, 0.0))
            det = SkyTargetDetector(cam, params or DetectorParams(), polarity)
            detectors[(w, h)] = det
        dets = det.detect(rgb, None)
        res.frames += 1
        res.detections += len(dets)
        used = set()
        st = seq_stats.setdefault(seq.name, [0, 0, 0])
        for box in fr.boxes:
            x, y, bw, bh = box
            band = _size_band(box)
            res.targets += 1
            res.by_size[band][1] += 1
            st[0] += 1
            m = match_margin_px
            hit = next(
                (
                    i
                    for i, d in enumerate(dets)
                    if i not in used and x - m <= d.u <= x + bw + m and y - m <= d.v <= y + bh + m
                ),
                None,
            )
            if hit is not None:
                used.add(hit)
                res.found += 1
                res.by_size[band][0] += 1
                st[1] += 1
        res.false += len(dets) - len(used)
        st[2] += len(dets) - len(used)
    for name, (targets, found, false) in seq_stats.items():
        res.per_sequence[name] = {
            "targets": targets,
            "recall": round(found / targets, 3) if targets else float("nan"),
            "false": false,
        }
    return res
