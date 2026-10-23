"""Deterministic command handling.

Two jobs, both independent of any language model:

* :func:`parse_emergency` recognises stop / abort / land-now. These are routed
  straight to the safety layer: no model latency, no chance of the model
  misreading "STOP" as something else.
* :class:`RuleParser` turns a constrained English phrasing into a
  :class:`~kestrel.mission.schema.Mission`. It is the fallback when no model is
  available and the reference the LLM planner is tested against.
"""

from __future__ import annotations

import re
from enum import StrEnum

from kestrel.mission.schema import (
    GotoTask,
    HoldTask,
    LandTask,
    Location,
    Mission,
    OrbitTask,
    ReturnHomeTask,
    SearchTask,
    StandbyTask,
    SurveyTask,
    TakeoffTask,
    Task,
    TrackTask,
)


class Emergency(StrEnum):
    HOLD = "hold"  # brake and hold position
    RETURN = "return"  # abort mission, return home and land
    LAND = "land"  # land where you are


_EMERGENCY = [
    (re.compile(r"^\s*(stop|halt|freeze|brake|hold position)\b", re.I), Emergency.HOLD),
    (re.compile(r"\b(abort|rtl now|return (home )?now|come back now)\b", re.I), Emergency.RETURN),
    (re.compile(r"\bland (now|immediately|here)\b|^\s*land\s*!*\s*$", re.I), Emergency.LAND),
]


def parse_emergency(text: str) -> Emergency | None:
    for pattern, cmd in _EMERGENCY:
        if pattern.search(text):
            return cmd
    return None


class ParseError(ValueError):
    pass


_NUM = r"(\d+(?:\.\d+)?)"
_SPLIT = re.compile(r"\s*(?:,?\s*\bthen\b|,?\s*\bafter that\b|;|\.\s+|\band finally\b)\s*", re.I)
_VERBS = (
    r"take ?off|launch|fly|go|head|move|proceed|transit|orbit|circle|loiter|survey|sweep|scan|"
    r"patrol|search|look for|track|follow|shadow|hold|hover|wait|return|come back|land|rtl|"
    r"stand ?by|keep watch"
)
_SUBSPLIT = re.compile(rf"\s*(?:,\s*(?:and\s+)?|\s+and\s+)(?=(?:{_VERBS})\b)", re.I)
_CONTACT = re.compile(
    r"(?:,?\s*and\s+)?\bif (?:you )?(?:see|spot|detect|find|encounter)s? (?:a|an|any)\s+"
    r"(?:drone|uav|uas|aircraft|intruder|target|contact)s?\b[^,.;]*?,?\s*"
    r"(?P<verb>track|follow|shadow|watch|observe|report|ignore)(?: it| them)?"
    r"(?P<rest>[^.;]*)",
    re.I,
)
_CUE = re.compile(
    r"(?:,?\s*and\s+)?\b(?:investigate|check|confirm|look at)\s+(?:any|all|every|each)?\s*"
    r"(?:acoustic\s+|ground\s+|sensor\s+)?(?:cues?|alerts?|alarms?|sounds?|reports?)\b"
    r"(?:\s+(?:from|by) the (?:ground|microphones?|sensors?))?",
    re.I,
)
_COORD = re.compile(r"\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)")


def _num(pattern: str, text: str) -> float | None:
    m = re.search(pattern, text, re.I)
    return float(m.group(1)) if m else None


def _duration(text: str) -> float | None:
    m = re.search(rf"\bfor {_NUM}\s*(s|sec|secs|seconds|min|mins|minutes)\b", text, re.I)
    if not m:
        return None
    v = float(m.group(1))
    return v * 60.0 if m.group(2).lower().startswith("m") else v


def _altitude(text: str) -> float | None:
    return _num(rf"\b(?:at|to)\s+{_NUM}\s*(?:m|metres|meters)\b(?!\s*(?:radius|spacing|apart|lanes))", text)


class RuleParser:
    def __init__(self, site_names: list[str]) -> None:
        # longest names first so "north field" wins over "north"
        self.sites = sorted(site_names, key=len, reverse=True)

    def _location(self, clause: str) -> Location | None:
        m = _COORD.search(clause)
        if m:
            return (float(m.group(1)), float(m.group(2)))
        low = clause.lower()
        for name in self.sites:
            variants = {name.lower(), name.lower().replace("_", " ")}
            if any(re.search(rf"\b{re.escape(v)}\b", low) for v in variants):
                return name
        return None

    def _need_location(self, clause: str) -> Location:
        loc = self._location(clause)
        if loc is None:
            known = ", ".join(self.sites) or "(none configured)"
            raise ParseError(f"no known place in '{clause}'. Known sites: {known}")
        return loc

    def _clause(self, c: str) -> Task | None:
        low = c.lower().strip(" ,")
        if not low:
            return None
        alt = _altitude(c)
        speed = _num(rf"\b(?:at|speed of)\s+{_NUM}\s*(?:m/s|mps)\b", c)
        if re.search(r"\b(take ?off|launch|lift off)\b", low):
            return TakeoffTask(altitude=alt or 20.0)
        if re.search(r"\bland\b", low) and not re.search(r"\b(return|home)\b", low):
            return LandTask()
        if re.search(r"\b(return|come back|go home|rtl|head home|fly home)\b", low):
            return ReturnHomeTask()
        if re.search(r"\b(survey|sweep|scan|map|patrol|cover)\b", low):
            loc = self._need_location(c)
            if not isinstance(loc, str):
                raise ParseError("a survey needs a named area")
            spacing = _num(rf"{_NUM}\s*m\s*(?:spacing|apart|lanes)", c) or _num(rf"spacing (?:of )?{_NUM}", c)
            return SurveyTask(area=loc, altitude=alt, spacing=spacing or 30.0, speed=speed)
        if re.search(r"\b(orbit|circle|loiter (?:around|over))\b", low):
            radius = _num(rf"radius (?:of )?{_NUM}", c) or _num(rf"{_NUM}\s*m\s*radius", c)
            laps = _num(rf"{_NUM}\s*(?:times|laps?)\b", c)
            if laps is None:
                laps = 2.0 if "twice" in low else 1.0
            return OrbitTask(
                at=self._need_location(c),
                radius=radius or 25.0,
                altitude=alt,
                laps=laps,
                speed=speed,
                clockwise="clockwise" in low and "counter" not in low and "anti" not in low,
            )
        if re.search(r"\b(search|look for|hunt)\b", low):
            return SearchTask(
                at=self._need_location(c),
                altitude=alt,
                radius=_num(rf"radius (?:of )?{_NUM}", c) or 15.0,
                duration_s=_duration(c) or 60.0,
            )
        if re.search(r"\b(stand ?by|keep watch|on station)\b", low):
            return StandbyTask(at=self._need_location(c), altitude=alt, duration_s=_duration(c) or 300.0)
        if re.search(r"\b(track|follow|shadow|observe|watch)\b", low):
            return TrackTask(
                standoff_m=_num(rf"\bfrom {_NUM}\s*m\b", c) or 30.0, duration_s=_duration(c) or 60.0
            )
        if re.search(r"\b(hold|hover|wait|loiter)\b", low):
            return HoldTask(duration_s=_duration(c) or 10.0)
        if re.search(r"\b(fly|go|head|move|proceed|transit|navigate)\b", low):
            return GotoTask(at=self._need_location(c), altitude=alt, speed=speed)
        raise ParseError(f"could not understand '{c.strip()}'")

    def parse(self, text: str) -> Mission:
        on_cue = "ignore"
        mc = _CUE.search(text)
        if mc:
            on_cue = "investigate"
            text = text[: mc.start()] + " then " + text[mc.end() :]
        on_contact = "report"
        standoff = 30.0
        track_s = 60.0
        m = _CONTACT.search(text)
        if m:
            verb = m.group("verb").lower()
            on_contact = "ignore" if verb == "ignore" else "report" if verb == "report" else "track"
            rest = m.group("rest") or ""
            standoff = _num(rf"\bfrom {_NUM}\s*m\b", rest) or standoff
            track_s = _duration(rest) or track_s
            text = text[: m.start()] + " then " + text[m.end() :]
        clauses = [sub for c in _SPLIT.split(text) for sub in _SUBSPLIT.split(c)]
        tasks = [t for c in clauses if (t := self._clause(c)) is not None]
        if not tasks:
            raise ParseError("no tasks found")
        return Mission(
            tasks=tasks,
            on_contact=on_contact,
            contact_standoff_m=standoff,
            contact_track_s=track_s,
            on_cue=on_cue,
        )


_INVESTIGATE = re.compile(r"^\s*(?:investigate|check|confirm|look at)\b", re.I)


def parse_investigate(text: str, parser: RuleParser) -> Location | None:
    """An operator tasking during flight: "investigate (40, 90)" or "check the tower"."""
    if not _INVESTIGATE.search(text):
        return None
    return parser._location(text)
