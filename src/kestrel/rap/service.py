"""The fusion node as a live service: real SAPIENT sensors in, one air picture out.

``kestrel fusion`` listens for SAPIENT nodes over TCP (``interop/wire.py``), fuses what they
report with the same ``FusionNode`` the benchmarks measure, and every period:

* writes the recognised air picture as one JSON line (tracks, identities, sensor health);
* sends each mobile node the picture's 3-D tracks as detection reports (its traffic),
  without the track that is that node itself;
* if tasking is on, sends the camera node a ``LOOK_AT`` task for the track that most needs
  a look.

What it refuses, and counts in the picture's ``rejected`` section so an operator sees it:

* **reports with a wrong clock.** A report stamped more than ``max_skew_s`` away from the
  service's clock is dropped: fusing a stale or future position corrupts every track it
  touches, and a replayed recording is exactly a stream of old timestamps. Nodes must be
  time-synchronised (GNSS or NTP);
* **reports outside the site.** A position further than ``max_range_m`` from the site origin
  is a wrong datum, a wrong UTM zone or a fault;
* everything the wire layer refuses (unregistered senders, taken node ids, missing mandatory
  fields, oversized or undecodable frames).

A sensor that has registered and then gone quiet for three status intervals is listed as
``STALE``: a picture that silently lost its radar looks the same as an empty sky.

``HOSTILE`` is never set here. ``declare`` records an operator's decision.
"""

from __future__ import annotations

import json
import logging
import os
import ssl
import threading
import time
from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import IO, Any

import numpy as np

from kestrel.config import Site, WorldConfig
from kestrel.interop.sapient import SapientExporter, SapientOptions
from kestrel.interop.wire import SapientServer, content_kind
from kestrel.rap.fusion import FusionNode, Message
from kestrel.rap.replay import snapshot
from kestrel.rap.tasking import LookTasker

log = logging.getLogger("kestrel.fusion")


@dataclass
class ServiceOptions:
    host: str = "127.0.0.1"
    port: int = 5020
    period_s: float = 1.0
    tasking: bool = True
    traffic: bool = True
    max_skew_s: float = 2.0
    max_range_m: float = 20_000.0
    stale_after_intervals: float = 3.0
    tls: ssl.SSLContext | None = None
    out: Path | None = None


@dataclass
class _NodeHealth:
    node_type: str
    name: str
    interval_s: float
    last_heard: float
    reports: int = 0


@dataclass
class FusionService:
    world: WorldConfig
    whitelist: list[str] = field(default_factory=list)
    protected: dict[str, Site] = field(default_factory=dict)
    opts: ServiceOptions = field(default_factory=ServiceOptions)
    clock: Callable[[], float] = time.time

    def __post_init__(self) -> None:
        self.epoch = self.clock()
        seed = int.from_bytes(os.urandom(8), "big")  # node and task ids must not be guessable
        self.geo = SapientExporter(
            SapientOptions(origin_lla=self.world.origin_lla, epoch_s=self.epoch, seed=seed)
        )
        self.fusion = FusionNode(self.geo, self.world, self.whitelist, self.protected)
        self.tasker = LookTasker(self.fusion)
        self.health: dict[str, _NodeHealth] = {}
        self.rejected: Counter[str] = Counter()
        self.task_acks: Counter[str] = Counter()
        self.declarations: list[dict[str, Any]] = []
        self.pictures = 0
        self._lock = threading.Lock()
        self._out: IO[str] | None = None
        self.server = SapientServer(
            self.opts.host,
            self.opts.port,
            self._on_message,
            self.fusion.node_id,
            tls=self.opts.tls,
            timestamp=lambda: self.geo.timestamp(self.now()),
        )

    def now(self) -> float:
        return self.clock() - self.epoch

    # ------------------------------------------------------------------ input
    def _on_message(self, msg: Message) -> None:
        node = str(msg.get("node_id", ""))
        kind = content_kind(msg)
        t = self.now()
        try:
            skew = self.geo.sim_time(str(msg["timestamp"])) - t
        except (KeyError, ValueError):
            self.rejected["bad timestamp"] += 1
            return
        if abs(skew) > self.opts.max_skew_s:
            self.rejected["clock skew"] += 1
            if self.rejected["clock skew"] in (1, 100, 10_000):
                log.warning("node %s is %.1f s off the fusion clock; its reports are dropped", node, skew)
            return
        loc = (msg.get("detection_report") or {}).get("location")
        if isinstance(loc, dict):
            try:
                far = float(np.linalg.norm(self.geo.to_enu(loc)[0:2])) > self.opts.max_range_m
            except (KeyError, ValueError):
                far = True
            if far:
                self.rejected["outside the site"] += 1
                return
        with self._lock:
            if kind == "registration":
                reg = msg["registration"]
                defs = reg.get("node_definition") or [{}]
                interval = float(
                    ((reg.get("status_definition") or {}).get("status_interval") or {}).get("value", 5)
                )
                self.health[node] = _NodeHealth(
                    str(defs[0].get("node_type", "NODE_TYPE_OTHER")),
                    str(reg.get("short_name") or reg.get("name") or node[:8]),
                    max(interval, 1.0),
                    t,
                )
            elif node in self.health:
                self.health[node].last_heard = t
                self.health[node].reports += kind == "detection_report"
            if kind == "task_ack":
                self.task_acks[str(msg["task_ack"].get("task_status", ""))] += 1
            self.fusion.ingest(msg)

    def declare(self, track: int, identity: str) -> None:
        """Record an operator's identification of a track (the only way ``HOSTILE`` is set)."""
        with self._lock:
            if track not in {tr.ident for tr in self.fusion.tracks}:
                raise ValueError(f"no track {track} in the picture")
            self.fusion.declare(track, identity)
            self.declarations.append({"t": round(self.now(), 1), "track": track, "identity": identity})
        log.warning("operator declared track %d %s", track, identity)

    def operator_command(self, line: str) -> str:
        """One line from the operator's console: ``declare <track> <IDENTITY>``.

        Identification is a human decision. This is the only path by which a track becomes
        ``HOSTILE`` (or is overruled to anything else), and every use is kept in the picture.
        """
        words = line.split()
        if len(words) == 3 and words[0].lower() == "declare" and words[1].isdigit():
            try:
                self.declare(int(words[1]), words[2].upper())
            except ValueError as e:
                return f"refused: {e}"
            return f"track {words[1]} declared {words[2].upper()}"
        return "commands: declare <track id> <FRIEND|ASSUMED_FRIEND|NEUTRAL|SUSPECT|UNKNOWN|HOSTILE>"

    # ------------------------------------------------------------------ output
    def tick(self) -> dict[str, Any]:
        """Advance the picture to now, send traffic and tasking, and return the picture."""
        t = self.now()
        connected = set(self.server.nodes())
        with self._lock:
            self.fusion.step(t)
            picture = snapshot(self.fusion, t, self.world.origin_lla, self.epoch)
            outgoing: list[Message] = []
            if self.opts.traffic:
                for node in self.fusion.mobile_nodes & connected:
                    outgoing += self.fusion.traffic_reports(t, node)
            if self.opts.tasking and self.fusion.camera_node in connected:
                outgoing += self.tasker.step(t)
            picture["nodes"] = [
                {
                    "node_id": node,
                    "name": h.name,
                    "type": h.node_type,
                    "state": self._state(node, h, t, connected),
                    "silent_s": round(t - h.last_heard, 1),
                    "reports": h.reports,
                }
                for node, h in sorted(self.health.items())
            ]
            picture["rejected"] = (
                dict(self.rejected) | dict(self.fusion.rejected) | {"by the link": self.server.rejected}
            )
            picture["tasks"] = {"sent": self.tasker.tasks_sent, "acks": dict(self.task_acks)}
            picture["declared_by_operator"] = list(self.declarations)
        for m in outgoing:
            self.server.send(m)
        self.pictures += 1
        if self._out is not None:
            self._out.write(json.dumps(picture, separators=(",", ":")) + "\n")
            self._out.flush()
        return picture

    def _state(self, node: str, h: _NodeHealth, t: float, connected: set[str]) -> str:
        if node not in connected:
            return "DISCONNECTED"
        return "STALE" if t - h.last_heard > self.opts.stale_after_intervals * h.interval_s else "OK"

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> FusionService:
        if self.opts.out is not None:
            self.opts.out.parent.mkdir(parents=True, exist_ok=True)
            self._out = open(self.opts.out, "a", encoding="utf-8")  # noqa: SIM115 - closed in close()
        self.server.start()
        return self

    def close(self) -> None:
        self.server.close()
        if self._out is not None:
            self._out.close()
            self._out = None

    def run(self, stop: threading.Event, on_picture: Callable[[dict[str, Any]], None] | None = None) -> None:
        """Tick every ``period_s`` on a monotonic schedule until ``stop`` is set."""
        self.start()
        try:
            nxt = time.monotonic()
            while not stop.is_set():
                picture = self.tick()
                if on_picture is not None:
                    on_picture(picture)
                nxt += self.opts.period_s
                stop.wait(max(0.0, nxt - time.monotonic()))
        finally:
            self.close()


def summary_line(picture: dict[str, Any]) -> str:
    ids = Counter(tr["identity"] for tr in picture["tracks"])
    nodes = Counter(n["state"] for n in picture["nodes"])
    bad = sum(int(v) for v in picture["rejected"].values())
    return (
        f"t={picture['t']:7.1f}s tracks={len(picture['tracks'])} "
        + " ".join(f"{k.lower()}={v}" for k, v in sorted(ids.items()))
        + f" | nodes {' '.join(f'{k}={v}' for k, v in sorted(nodes.items())) or 'none'} | rejected={bad}"
    )


def stream_log(
    path: str | Path,
    host: str,
    port: int,
    speed: float = 1.0,
    tls: ssl.SSLContext | None = None,
    stop: threading.Event | None = None,
) -> int:
    """Replay a recorded SAPIENT log (JSON lines) to a fusion node in real time.

    Timestamps are shifted so the recording starts now (a fusion node drops old reports), and
    each node in the log gets its own connection, as real sensors have. For integration
    tests against a fusion node or decision module without the sensors present.
    """
    from kestrel.interop.sapient import rfc3339
    from kestrel.interop.wire import SapientClient
    from kestrel.rap.replay import _epoch, load

    msgs = load(path)
    if not msgs:
        return 0
    t0 = _epoch(str(msgs[0]["timestamp"]))
    wall0 = time.time()
    clients: dict[str, SapientClient] = {}
    sent = 0
    try:
        for m in msgs:
            if stop is not None and stop.is_set():
                break
            due = (_epoch(str(m["timestamp"])) - t0) / max(speed, 1e-6)
            delay = wall0 + due - time.time()
            if delay > 0:
                time.sleep(delay)
            node = str(m.get("node_id", ""))
            if m.get("destination_id") or "task" in m:
                continue  # what a fusion node sent in the recording is not a sensor's to resend
            if node not in clients:
                clients[node] = SapientClient(host, port, tls=tls).start()
            out = dict(m)
            out["timestamp"] = rfc3339(time.time())
            clients[node].send(out)
            sent += 1
        deadline = time.monotonic() + 5.0
        while any(c.pending() for c in clients.values()) and time.monotonic() < deadline:
            time.sleep(0.05)
    finally:
        for c in clients.values():
            c.close()
    return sent


__all__ = ["FusionService", "ServiceOptions", "stream_log", "summary_line"]
