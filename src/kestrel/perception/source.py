"""Where camera frames come from in flight.

The simulator renders its own frames. A real aircraft reads them from a camera, and a
hardware-in-the-loop bench renders them from the autopilot's position. All three give the
flight loop the same thing: the latest RGB frame and the time it was taken.

* ``OpenCvSource``: any device, file or stream OpenCV opens (a V4L2 index, a GStreamer
  pipeline, an RTSP URL, a recorded video). A reader thread keeps only the newest frame, so
  a slow consumer never works through a backlog of stale ones. Needs
  ``pip install kestrel-uas[camera]``.
* ``CallableSource``: frames from a function of time, for bench tests and for replaying
  rendered scenes against a real autopilot.

A source reports when it has gone quiet (``age_s``); the flight loop treats a camera that
stopped delivering as a failed sensor, not as an empty sky.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Any, Protocol

import numpy as np


class FrameSource(Protocol):
    def read(self) -> tuple[float, np.ndarray] | None:
        """The newest frame not yet returned, as ``(time taken, RGB uint8 HxWx3)``; the time is
        on the clock the flight loop runs on (``time.monotonic`` in flight)."""
        ...

    def age_s(self) -> float:
        """Seconds since the source last produced a frame (infinite if it never has)."""
        ...

    def close(self) -> None: ...


class CallableSource:
    """Frames from ``render(now)`` at a fixed rate; ``render`` may return ``None`` for no frame."""

    def __init__(
        self,
        render: Callable[[float], np.ndarray | None],
        rate_hz: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.render = render
        self.period = 1.0 / rate_hz
        self.clock = clock
        self._next = clock()
        self._last = -1.0

    def read(self) -> tuple[float, np.ndarray] | None:
        now = self.clock()
        if now < self._next:
            return None
        self._next = max(self._next + self.period, now - self.period)
        frame = self.render(now)
        if frame is None:
            return None
        self._last = now
        return now, frame

    def age_s(self) -> float:
        return self.clock() - self._last if self._last >= 0.0 else float("inf")

    def close(self) -> None:
        return None


class OpenCvSource:
    """The newest frame from an OpenCV capture, read on its own thread."""

    def __init__(self, spec: str, width: int | None = None, height: int | None = None) -> None:
        try:
            import cv2
        except ImportError as e:  # pragma: no cover - depends on the optional extra
            raise RuntimeError("a camera needs OpenCV: pip install kestrel-uas[camera]") from e
        self._cv2: Any = cv2
        target: int | str = int(spec) if spec.isdigit() else spec
        self._cap = cv2.VideoCapture(target)
        if not self._cap.isOpened():
            raise RuntimeError(f"cannot open camera {spec!r}")
        if width and height and isinstance(target, int):
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._lock = threading.Lock()
        self._frame: tuple[float, np.ndarray] | None = None
        self._last = 0.0
        self._running = True
        self.frames = 0
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while self._running:
            ok, bgr = self._cap.read()
            now = time.monotonic()
            if not ok:
                time.sleep(0.02)
                continue
            if bgr.ndim == 2:  # a thermal core delivering one channel
                rgb = np.repeat(bgr[:, :, None], 3, axis=2)
            else:
                rgb = self._cv2.cvtColor(bgr, self._cv2.COLOR_BGR2RGB)
            with self._lock:
                self._frame = (now, np.ascontiguousarray(rgb, dtype=np.uint8))
                self._last = now
                self.frames += 1

    def read(self) -> tuple[float, np.ndarray] | None:
        with self._lock:
            frame, self._frame = self._frame, None
        return frame

    def age_s(self) -> float:
        with self._lock:
            return time.monotonic() - self._last if self._last else float("inf")

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=2.0)
        self._cap.release()
