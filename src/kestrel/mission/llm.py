"""Natural-language mission planning with a local LLM (Ollama).

Design rules, each one a lesson from flying an LLM-commanded vehicle:

* The model runs locally (default ``qwen2.5:3b``): no cloud dependency, no
  operational data leaves the machine, works without a network link.
* Output is constrained with Ollama structured outputs, passing the
  :class:`Mission` JSON Schema as ``format``, then *re-validated* with pydantic.
  The schema constraint reduces bad output; validation is what we rely on.
* One repair round: the validation error goes back to the model. After that the
  deterministic :class:`RuleParser` gets a try; if both fail the command is
  rejected, never guessed.
* Emergency phrases never reach this module (see :func:`parse_emergency`).
"""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Literal, Protocol

from pydantic import ValidationError

from kestrel.mission.parser import ParseError, RuleParser
from kestrel.mission.schema import Mission

SYSTEM_PROMPT = """You convert an operator's instruction for a small observation drone \
into a JSON mission plan. Output JSON only, matching the provided schema.

Frame: local ENU metres, x = east, y = north, origin at home. Altitudes are metres above home.
Known sites (use these names exactly, never invent new ones):
{sites}

Rules:
- Use only task types in the schema. Do not add tasks the operator did not ask for.
- If the operator does not give an altitude, leave it null.
- "if you see a drone, follow/track it" sets on_contact to "track";
  the standoff distance goes in contact_standoff_m.
- End with return_home when the operator asks to come back, return, or go home.
"""


class ChatBackend(Protocol):
    def chat(self, system: str, user: str, schema: dict[str, object]) -> str: ...


class OllamaBackend:
    def __init__(
        self,
        model: str = "qwen2.5:3b",
        host: str = "http://127.0.0.1:11434",
        timeout_s: float = 30.0,
    ) -> None:
        self.model = model
        self.host = host.rstrip("/")
        self.timeout_s = timeout_s

    def chat(self, system: str, user: str, schema: dict[str, object]) -> str:
        body = {
            "model": self.model,
            "messages": [{"role": "system", "content": system}, {"role": "user", "content": user}],
            "format": schema,
            "stream": False,
            "options": {"temperature": 0},
        }
        req = urllib.request.Request(
            f"{self.host}/api/chat",
            data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
            payload = json.loads(resp.read().decode())
        content = payload.get("message", {}).get("content")
        if not isinstance(content, str):
            raise ValueError("unexpected response from Ollama: no message content")
        return content


@dataclass
class PlanResult:
    mission: Mission | None
    source: Literal["llm", "llm-repaired", "rules", "none"]
    notes: list[str] = field(default_factory=list)


class MissionPlanner:
    def __init__(self, site_names: list[str], backend: ChatBackend | None = None) -> None:
        self.sites = site_names
        self.backend = backend
        self.rules = RuleParser(site_names)
        self.schema = Mission.model_json_schema()

    def _system(self) -> str:
        return SYSTEM_PROMPT.format(sites="\n".join(f"- {s}" for s in self.sites) or "- (none)")

    def plan(self, text: str) -> PlanResult:
        notes: list[str] = []
        if self.backend is not None:
            prompt = text
            for attempt in range(2):
                try:
                    raw = self.backend.chat(self._system(), prompt, self.schema)
                    mission = Mission.model_validate_json(raw)
                    unknown = self._unknown_sites(mission)
                    if unknown:
                        raise ValueError(f"unknown site(s): {', '.join(unknown)}")
                    return PlanResult(mission, "llm" if attempt == 0 else "llm-repaired", notes)
                except (ValidationError, ValueError) as exc:
                    msg = str(exc).splitlines()[0]
                    notes.append(f"LLM attempt {attempt + 1} rejected: {msg}")
                    prompt = (
                        f"{text}\n\nYour previous answer was invalid: {msg}. Return a corrected JSON plan."
                    )
                except (urllib.error.URLError, TimeoutError, OSError) as exc:
                    notes.append(f"LLM unavailable ({exc.__class__.__name__}); using rule parser")
                    break
        try:
            return PlanResult(self.rules.parse(text), "rules", notes)
        except ParseError as exc:
            notes.append(f"rule parser: {exc}")
            return PlanResult(None, "none", notes)

    def _unknown_sites(self, mission: Mission) -> list[str]:
        names = []
        for t in mission.tasks:
            for attr in ("at", "area"):
                v = getattr(t, attr, None)
                if isinstance(v, str) and v not in self.sites:
                    names.append(v)
        return names
