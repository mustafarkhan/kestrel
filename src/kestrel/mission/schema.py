"""Mission plan schema: the only thing the language layer is allowed to produce.

An LLM never sends setpoints or MAVLink. It emits JSON that must validate against
:class:`Mission`; the same JSON Schema is handed to the model as a structured-output
constraint. Anything outside this vocabulary is unrepresentable, which is the
first of three safety barriers (schema -> static plan validation -> runtime monitor).
"""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field

Location = Annotated[
    str | tuple[float, float],
    Field(description="A named site from the site list, or [x, y] metres in the local frame"),
]


class _Task(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class TakeoffTask(_Task):
    type: Literal["takeoff"] = "takeoff"
    altitude: float = Field(20.0, description="metres above home")


class GotoTask(_Task):
    type: Literal["goto"] = "goto"
    at: Location
    altitude: float | None = None
    speed: float | None = None


class OrbitTask(_Task):
    type: Literal["orbit"] = "orbit"
    at: Location
    radius: float = 25.0
    altitude: float | None = None
    laps: float = 1.0
    speed: float | None = None
    clockwise: bool = False


class SurveyTask(_Task):
    type: Literal["survey"] = "survey"
    area: str = Field(description="name of a rectangular or circular site")
    altitude: float | None = None
    spacing: float = Field(30.0, description="distance between survey lanes, metres")
    speed: float | None = None


class SearchTask(_Task):
    type: Literal["search"] = "search"
    at: Location
    radius: float = 15.0
    altitude: float | None = None
    duration_s: float = 60.0


class HoldTask(_Task):
    type: Literal["hold"] = "hold"
    duration_s: float = 10.0


class TrackTask(_Task):
    type: Literal["track"] = "track"
    standoff_m: float = 30.0
    duration_s: float = 60.0


class ReturnHomeTask(_Task):
    type: Literal["return_home"] = "return_home"


class LandTask(_Task):
    type: Literal["land"] = "land"


Task = Annotated[
    TakeoffTask
    | GotoTask
    | OrbitTask
    | SurveyTask
    | SearchTask
    | HoldTask
    | TrackTask
    | ReturnHomeTask
    | LandTask,
    Field(discriminator="type"),
]


class Mission(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    tasks: list[Task] = Field(min_length=1, max_length=24)
    on_contact: Literal["ignore", "report", "track"] = Field(
        "report", description="what to do when an unknown aircraft is detected"
    )
    contact_standoff_m: float = 30.0
    contact_track_s: float = 60.0
    max_speed: float | None = None

    def describe(self) -> str:
        parts = []
        for t in self.tasks:
            fields = {k: v for k, v in t.model_dump().items() if k != "type" and v is not None}
            args = ", ".join(f"{k}={v}" for k, v in fields.items())
            parts.append(f"{t.type}({args})")
        return " -> ".join(parts) + f"  [on_contact={self.on_contact}]"
