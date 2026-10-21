"""Language layer: rule parser, emergency routing, LLM planner contract, validation."""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any

import pytest

from kestrel.config import Config, Scenario
from kestrel.mission.llm import MissionPlanner, OllamaBackend
from kestrel.mission.parser import Emergency, ParseError, RuleParser, parse_emergency
from kestrel.mission.schema import Mission, OrbitTask, ReturnHomeTask, SurveyTask, TakeoffTask
from kestrel.mission.validator import validate

SITES = ["north_field", "tower", "lake", "hangar"]


@pytest.mark.parametrize(
    "text,expected",
    [
        ("STOP", Emergency.HOLD),
        ("  stop now", Emergency.HOLD),
        ("abort the mission", Emergency.RETURN),
        ("come back now!", Emergency.RETURN),
        ("land now", Emergency.LAND),
        ("land", Emergency.LAND),
        ("survey the lake then land", None),
        ("orbit the tower", None),
    ],
)
def test_emergency_routing(text: str, expected: Emergency | None) -> None:
    assert parse_emergency(text) is expected


def test_rule_parser_full_sentence() -> None:
    m = RuleParser(SITES).parse(
        "Take off to 25 m, survey the north field at 30 m with 25 m spacing, and if you see a drone, "
        "track it from 35 m for 90 seconds. Then return home and land."
    )
    types = [t.type for t in m.tasks]
    assert types == ["takeoff", "survey", "return_home", "land"]
    assert isinstance(m.tasks[1], SurveyTask) and m.tasks[1].spacing == 25 and m.tasks[1].altitude == 30
    assert m.on_contact == "track" and m.contact_standoff_m == 35 and m.contact_track_s == 90


def test_rule_parser_commas_and_coordinates() -> None:
    m = RuleParser(SITES).parse("launch to 15 m and fly to (40, -20), hover for 2 min and land")
    assert [t.type for t in m.tasks] == ["takeoff", "goto", "hold", "land"]
    assert m.tasks[1].at == (40.0, -20.0)  # type: ignore[union-attr]
    assert m.tasks[2].duration_s == 120  # type: ignore[union-attr]
    orbit = RuleParser(SITES).parse("take off, orbit the tower twice with a radius of 30 m").tasks[1]
    assert isinstance(orbit, OrbitTask) and orbit.laps == 2 and orbit.radius == 30


def test_rule_parser_rejects_unknown_places() -> None:
    with pytest.raises(ParseError, match="Known sites"):
        RuleParser(SITES).parse("fly to the moon")


# --------------------------------------------------------------------------- LLM planner


class FakeBackend:
    def __init__(self, replies: list[str]) -> None:
        self.replies = list(replies)
        self.prompts: list[str] = []

    def chat(self, system: str, user: str, schema: dict[str, object]) -> str:
        self.prompts.append(user)
        return self.replies.pop(0)


GOOD = Mission(tasks=[TakeoffTask(altitude=20), OrbitTask(at="tower"), ReturnHomeTask()]).model_dump_json()


def test_llm_valid_plan_is_used() -> None:
    r = MissionPlanner(SITES, FakeBackend([GOOD])).plan("go look at the tower")
    assert r.source == "llm" and r.mission is not None


def test_llm_gets_one_repair_round() -> None:
    backend = FakeBackend(['{"tasks": [{"type": "teleport"}]}', GOOD])
    r = MissionPlanner(SITES, backend).plan("go look at the tower")
    assert r.source == "llm-repaired"
    assert "previous answer was invalid" in backend.prompts[1]


def test_llm_invented_site_is_rejected_then_rules_fallback() -> None:
    bad = Mission(tasks=[OrbitTask(at="eiffel_tower")]).model_dump_json()
    r = MissionPlanner(SITES, FakeBackend([bad, bad])).plan("orbit the tower")
    assert r.source == "rules"
    assert any("unknown site" in n for n in r.notes)


def test_llm_garbage_and_unparseable_text_gives_no_plan() -> None:
    r = MissionPlanner(SITES, FakeBackend(["not json", "still not json"])).plan("do a barrel roll")
    assert r.mission is None and r.source == "none"


def test_llm_unreachable_falls_back_to_rules() -> None:
    backend = OllamaBackend(host="http://127.0.0.1:9", timeout_s=0.5)
    r = MissionPlanner(SITES, backend).plan("take off and orbit the tower then return home")
    assert r.source == "rules" and r.mission is not None


def test_ollama_request_carries_schema_and_is_deterministic() -> None:
    seen: dict[str, Any] = {}

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self) -> None:
            seen["path"] = self.path
            seen["body"] = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            out = json.dumps({"message": {"role": "assistant", "content": GOOD}}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(out)

        def log_message(self, *args: object) -> None:
            pass

    srv = HTTPServer(("127.0.0.1", 0), Handler)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    try:
        backend = OllamaBackend(host=f"http://127.0.0.1:{srv.server_port}")
        r = MissionPlanner(SITES, backend).plan("orbit the tower")
    finally:
        srv.shutdown()
    assert r.source == "llm"
    body = seen["body"]
    assert seen["path"] == "/api/chat" and body["stream"] is False
    assert body["format"]["title"] == "Mission"  # JSON schema passed as structured-output constraint
    assert body["options"]["temperature"] == 0
    assert "tower" in body["messages"][0]["content"]  # site list is in the system prompt


# --------------------------------------------------------------------------- validation


def _check(text: str, cfg: Config, sc: Scenario) -> list[str]:
    m = RuleParser(list(sc.world.sites)).parse(text)
    return validate(m, cfg, sc.world).errors


def test_validator_accepts_reasonable_mission(cfg: Config, airfield: Scenario) -> None:
    m = RuleParser(list(airfield.world.sites)).parse("survey the north field at 25 m")
    rep = validate(m, cfg, airfield.world)
    assert rep.ok, rep.summary()
    assert rep.mission.tasks[0].type == "takeoff" and rep.mission.tasks[-1].type == "return_home"
    assert 0 < rep.est_energy_wh < rep.usable_energy_wh


@pytest.mark.parametrize(
    "text,fragment",
    [
        ("orbit the tower at 80 m", "altitude 80 m outside"),
        ("fly to (200, 0) at 20 m", "leaves the geofence"),
        ("fly to (145, 0) at 20 m", "within"),
        ("survey the lake at 20 m with 6 m spacing then hover for 25 minutes", "exceeds"),
        ("track it from 8 m", "standoff 8 m is below"),
        ("orbit the tower with a radius of 3 m", "radius must be at least"),
    ],
)
def test_validator_rejects_with_reason(cfg: Config, airfield: Scenario, text: str, fragment: str) -> None:
    errors = _check(text, cfg, airfield)
    assert errors and any(fragment in e for e in errors), errors
