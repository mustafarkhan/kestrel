"""The threat library and the scorecard: repeatable situations, and one way of scoring any picture."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from kestrel.rap import scorecard as sc
from kestrel.rap.picture import make_geo, run_picture
from kestrel.rap.replay import fuse_messages
from kestrel.rap.sensors import SensorNetwork
from kestrel.rap.threats import LIBRARY_VERSION, THREATS, build
from kestrel.rap.truth import Truth

ROOT = Path(__file__).resolve().parents[1]
PUBLISHED = ROOT / "docs" / "results" / "scorecard"


@pytest.mark.parametrize("name", list(THREATS))
def test_a_threat_is_the_same_situation_every_time(name: str) -> None:
    a, b, other = build(name, 9000), build(name, 9000), build(name, 9001)
    assert a == b, "same seed, same situation"
    assert a != other, "another seed, another situation"
    assert a.sensors is not None and a.name == f"{name}-9000"
    assert set(THREATS[name].watch) <= set(sc.MEASURES)


def test_threats_contain_what_they_say() -> None:
    hostile = lambda s: [d for d in s.intruders if d.role == "hostile"]  # noqa: E731
    assert all(
        not d.rf_link and d.propulsion == "quiet" and d.center[2] <= 20
        for d in hostile(build("silent-low", 1))
    )
    assert len(hostile(build("saturation", 1))) == 12
    assert not hostile(build("birds-only", 1)) and build("birds-only", 1).birds.count == 40
    imp = build("remote-id-impersonation", 1)
    serial = hostile(imp)[0].remote_id
    assert serial in imp.sensors.whitelist  # type: ignore[union-attr]
    assert hostile(imp)[0].remote_id_offset_m == (0.0, 0.0, 0.0)  # it tells the truth about where it is
    assert not [
        d for d in imp.intruders if d.role == "friendly" and d.remote_id == serial
    ]  # ours is grounded
    with pytest.raises(KeyError):
        build("no-such-threat", 1)


def test_a_sensor_in_an_outage_is_silent() -> None:
    s = build("radar-outage", 3)
    assert s.sensors is not None
    geo = make_geo(s, s.seed)
    net = SensorNetwork(s.sensors, s.world, geo, s.seed, None)
    truth = Truth(s)
    radar = net.radar.node.node_id  # type: ignore[union-attr]
    sent: dict[bool, int] = {True: 0, False: 0}
    for k in range(int(420 / 0.5)):
        t = 0.5 * k
        msgs = net.step(t, truth.objects(t))
        sent[200.0 <= t < 400.0] += sum(m["node_id"] == radar for m in msgs)
        if 200.0 <= t < 400.0:
            assert any(m["node_id"] != radar for m in msgs) or k % 2, "the other sensors carry on"
    assert sent[True] == 0, "no reports and no status from a sensor that is down"
    assert sent[False] > 100


def test_scoring_a_picture_file_agrees_with_scoring_the_run(tmp_path: Path) -> None:
    """The route a system under test takes (exported files in, picture file out, scored from
    files) gives what the direct benchmark gives for Kestrel's own fusion node."""
    s = build("one-way-attack", 9000).model_copy(update={"max_duration_s": 240.0})
    direct = run_picture(s, dt=sc.STEP_S).score.summary()
    paths = sc.export(s, tmp_path)
    site = json.loads(paths["site"].read_text(encoding="utf-8"))
    assert site["threat_library"] == LIBRARY_VERSION and site["friendly_serials"]
    msgs = [json.loads(line) for line in paths["sensors"].read_text(encoding="utf-8").splitlines()]
    _, pictures = fuse_messages(msgs, site["friendly_serials"], period_s=sc.STEP_S)
    for p in pictures:  # a third party reports positions as latitude and longitude, not in our frame
        for tr in p["tracks"]:
            del tr["enu_m"]
    picture = tmp_path / "picture.jsonl"
    picture.write_text("\n".join(json.dumps(p) for p in pictures), encoding="utf-8")
    got = sc.score_files(picture, paths["truth"], paths["site"])
    for key in ("hostile_coverage", "drone_as_uav", "friend_identified"):
        assert got[key] == pytest.approx(direct[key], abs=0.05), key
    assert got["hostile_as_friend_s"] == direct["hostile_as_friend_s"] == 0.0

    # no picture at all is scored as nothing tracked, not as an error
    empty = tmp_path / "empty.jsonl"
    empty.write_text(json.dumps({"t": 0.0, "tracks": []}) + "\n", encoding="utf-8")
    assert sc.score_files(empty, paths["truth"], paths["site"])["hostile_coverage"] == 0.0


def test_the_published_reference_is_what_the_library_produces() -> None:
    card = json.loads((PUBLISHED / "scorecard.json").read_text(encoding="utf-8"))
    assert card["threat_library"] == LIBRARY_VERSION
    assert list(card["threats"]) == list(THREATS)
    assert (PUBLISHED / "scorecard.md").read_text(encoding="utf-8") == sc.markdown(card)
    lo, hi = card["seeds"]
    for name in THREATS:
        rows = [r for r in card["runs"] if r["threat"] == name]
        assert sorted(r["seed"] for r in rows) == list(range(lo, hi + 1))
        assert card["threats"][name] == sc.mean_rows(rows)


@pytest.mark.slow
def test_the_published_reference_reproduces() -> None:
    card = json.loads((PUBLISHED / "scorecard.json").read_text(encoding="utf-8"))
    row = next(r for r in card["runs"] if r["threat"] == "remote-id-spoof" and r["seed"] == card["seeds"][0])
    again = run_picture(build("remote-id-spoof", row["seed"]), dt=sc.STEP_S).score.summary()
    assert {k: row[k] for k in again} == again
