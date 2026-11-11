"""Which track the airborne node should look at, and when a look is over.

Shared by the simulated loop (``rap/loop.py``) and the live fusion service
(``rap/service.py``), so both task Kestrel by the same rules.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from kestrel.rap.fusion import FusionNode, Message


@dataclass
class Look:
    track: int
    start_t: float
    last_sent_t: float


@dataclass
class LookTasker:
    """One look at a time: start it, follow the track with re-sent positions, end it.

    A look is done when the camera has seen the track and the picture can now say what it is,
    when the track is gone, or when it has taken longer than ``look_timeout_s``.
    """

    fusion: FusionNode
    refresh_s: float = 3.0
    look_timeout_s: float = 75.0
    active: Look | None = None
    looks: list[dict[str, Any]] = field(default_factory=list)
    tasks_sent: int = 0

    def step(self, t: float) -> list[Message]:
        fusion = self.fusion
        live = {tr.ident: tr for tr in fusion.tracks}
        look = self.active
        if look is not None:
            tr = live.get(look.track)
            resolved = tr is not None and tr.camera_t >= look.start_t and fusion.class_of(tr) != "Unknown"
            done = tr is None or resolved or t - look.start_t > self.look_timeout_s
            if done:
                outcome = "gone" if tr is None else ("resolved" if resolved else "timeout")
                self.looks.append(
                    {"track": look.track, "start_t": look.start_t, "end_t": t, "outcome": outcome}
                )
                self.active = None
            elif tr is not None and t - look.last_sent_t >= self.refresh_s:
                look.last_sent_t = t
                self.tasks_sent += 1
                return [fusion.look_at_task(t, tr)]
            else:
                return []
        cand = fusion.most_needed_look(t)
        if cand is None or fusion.camera_node is None:
            return []
        self.active = Look(cand.ident, t, t)
        self.tasks_sent += 1
        return [fusion.look_at_task(t, cand)]
