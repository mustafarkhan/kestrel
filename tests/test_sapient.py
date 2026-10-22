"""SAPIENT export: mandatory fields per the ICD v7 tables, identifiers, coordinates."""

from __future__ import annotations

import re

import numpy as np
import pytest

from kestrel.interop.sapient import SapientExporter, SapientOptions, rfc3339, ulid
from kestrel.interop.utm import enu_to_lla, to_utm, zone_of
from kestrel.perception.acoustic import AcousticFix
from kestrel.perception.tracker import TrackEstimate

ORIGIN = (-35.363261, 149.165230, 584.0)

# Mandatory (M) fields from the ICD v7 tables: wrapper, Registration (5.1), StatusReport (6.1),
# DetectionReport (6.2), Location (4.3.1).
WRAPPER = {"timestamp", "node_id"}
REGISTRATION = {"node_type", "icd_version", "status_definition", "mode_definition"}
MODE_DEFINITION = {"mode_name", "settle_time", "detection_definition", "task", "mode_type"}
STATUS = {"report_id", "system", "info"}
DETECTION = {"report_id", "object_id", "location"}
LOCATION = {"x", "y", "coordinate_system", "datum"}
ULID = re.compile(r"^[0-9A-HJKMNP-TV-Z]{26}$")
UUID4 = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$")


@pytest.mark.parametrize(
    ("lat", "lon", "zone", "easting", "northing"),
    [
        # reference values from PROJ (pyproj 3.7, EPSG:327xx/326xx)
        (-35.363261, 149.165230, "55H", 696719.2449, 6084519.5869),
        (51.5007, -0.1246, "30U", 699567.5396, 5709427.5625),
        (54.6872, 25.2797, "35U", 389106.0120, 6061342.9052),
    ],
)
def test_utm_matches_proj(lat: float, lon: float, zone: str, easting: float, northing: float) -> None:
    e, n, _, _ = to_utm(lat, lon)
    assert zone_of(lat, lon) == zone
    assert e == pytest.approx(easting, abs=1e-3)
    assert n == pytest.approx(northing, abs=1e-3)


def test_enu_offsets_land_where_expected() -> None:
    e0, n0, _, _ = to_utm(ORIGIN[0], ORIGIN[1])
    lat, lon, alt = enu_to_lla(ORIGIN, 100.0, 200.0, 30.0)
    e1, n1, _, _ = to_utm(lat, lon, 55)
    # UTM grid north differs from true north by the grid convergence (about 1.1 deg here)
    assert np.hypot(e1 - e0, n1 - n0) == pytest.approx(np.hypot(100.0, 200.0) * 0.9996, rel=2e-3)
    assert alt == pytest.approx(614.0)


def test_ulid_and_timestamp_format() -> None:
    rng = np.random.default_rng(0)
    a, b = ulid(1767225600000, rng), ulid(1767225600001, rng)
    assert ULID.match(a) and ULID.match(b)
    assert a < b  # ULIDs sort by time
    assert rfc3339(1767225600.25) == "2026-01-01T00:00:00.250Z"


def _track(ident: int, pos: tuple[float, float, float], p_uav: float | None = None) -> TrackEstimate:
    cov = np.diag([4.0, 9.0, 1.0, 1.0, 1.0, 1.0])
    return TrackEstimate(ident, np.array(pos), np.array([1.0, -2.0, 0.0]), cov, True, 10, 0.0, 0.9, p_uav)


def _exporter() -> SapientExporter:
    return SapientExporter(SapientOptions(origin_lla=ORIGIN, seed=3))


def test_messages_carry_mandatory_fields() -> None:
    ex = _exporter()
    fix = AcousticFix(1.5, np.array([20.0, 30.0]), np.diag([16.0, 25.0]), 3)
    for k in range(30):
        t = 0.1 * k
        ex.step(t, np.array([0.0, 0.0, 25.0]), "survey", [_track(4, (30.0, 60.0, 30.0), 0.8)],
                [fix] if k == 15 else [], acoustic_fitted=True)  # fmt: skip
    kinds = [next(k for k in m if k not in WRAPPER) for m in ex.messages]
    assert kinds.count("registration") == 2  # camera and acoustic nodes
    assert kinds.count("detection_report") == 6 + 1  # 2 Hz over 3 s, plus one acoustic fix
    for m, kind in zip(ex.messages, kinds, strict=True):
        assert m.keys() >= WRAPPER
        assert UUID4.match(m["node_id"])
        body = m[kind]
        if kind == "registration":
            assert body.keys() >= REGISTRATION
            assert body["icd_version"] == "7.0"
            assert all(md.keys() >= MODE_DEFINITION for md in body["mode_definition"])
        elif kind == "status_report":
            assert body.keys() >= STATUS
            assert ULID.match(body["report_id"])
        else:
            assert body.keys() >= DETECTION
            assert body["location"].keys() >= LOCATION
            assert body["location"]["coordinate_system"] == "LOCATION_COORDINATE_SYSTEM_UTM_M"


def test_detection_content() -> None:
    ex = _exporter()
    ex.step(0.0, np.zeros(3), "track", [_track(7, (0.0, 0.0, 30.0), 0.9)], [], acoustic_fitted=False)
    ex.step(0.6, np.zeros(3), "track", [_track(7, (0.0, 0.0, 30.0), 0.9)], [], acoustic_fitted=False)
    dets = [m["detection_report"] for m in ex.messages if "detection_report" in m]
    assert len(dets) == 2
    assert dets[0]["object_id"] == dets[1]["object_id"]  # same track, same object
    assert dets[0]["report_id"] != dets[1]["report_id"]
    loc = dets[0]["location"]
    e0, n0, _, _ = to_utm(ORIGIN[0], ORIGIN[1])
    assert (loc["x"], loc["y"], loc["z"]) == (pytest.approx(e0, abs=0.01), pytest.approx(n0, abs=0.01), 614.0)
    assert (loc["x_error"], loc["y_error"], loc["z_error"]) == (2.0, 3.0, 1.0)  # one sigma
    assert dets[0]["enu_velocity"]["north_rate"] == -2.0
    assert dets[0]["detection_confidence"] == 0.9
    cls = {c["type"]: c["confidence"] for c in dets[0]["classification"]}
    assert cls == {"UAV": 0.9, "Bird": pytest.approx(0.1)}


def test_acoustic_fix_is_never_placed_on_the_ground() -> None:
    ex = SapientExporter(SapientOptions(origin_lla=ORIGIN, altitude_band_m=(0.0, 60.0)))
    ex.step(
        0.0, np.zeros(3), "survey", [], [AcousticFix(0.0, np.zeros(2), np.eye(2), 2)], acoustic_fitted=True
    )
    det = next(m["detection_report"] for m in ex.messages if "detection_report" in m)
    assert det["location"]["z"] == 614.0 and det["location"]["z_error"] == 30.0
    assert "classification" not in det
