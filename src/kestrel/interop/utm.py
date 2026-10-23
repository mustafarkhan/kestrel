"""WGS84 latitude/longitude to UTM, and the local ENU frame around a geodetic origin.

Krüger's series to third order in n, which is accurate to well under a millimetre within
a UTM zone. Implemented here (about forty lines) rather than adding a projection library
as a dependency; ``tests/test_sapient.py`` checks it against published reference points.
"""

from __future__ import annotations

import math

A = 6378137.0  # WGS84 semi-major axis, m
F = 1.0 / 298.257223563
K0 = 0.9996
E0 = 500000.0
N0_SOUTH = 10000000.0

_N = F / (2.0 - F)
_A_HAT = A / (1.0 + _N) * (1.0 + _N**2 / 4.0 + _N**4 / 64.0)
_ALPHA = (
    _N / 2.0 - 2.0 * _N**2 / 3.0 + 5.0 * _N**3 / 16.0,
    13.0 * _N**2 / 48.0 - 3.0 * _N**3 / 5.0,
    61.0 * _N**3 / 240.0,
)
_BETA = (
    _N / 2.0 - 2.0 * _N**2 / 3.0 + 37.0 * _N**3 / 96.0,
    _N**2 / 48.0 + _N**3 / 15.0,
    17.0 * _N**3 / 480.0,
)
_DELTA = (
    2.0 * _N - 2.0 * _N**2 / 3.0 - 2.0 * _N**3,
    7.0 * _N**2 / 3.0 - 8.0 * _N**3 / 5.0,
    56.0 * _N**3 / 15.0,
)
_E2 = F * (2.0 - F)
_LAT_BANDS = "CDEFGHJKLMNPQRSTUVWX"


def zone_of(lat_deg: float, lon_deg: float) -> str:
    """UTM zone with latitude band, e.g. ``"55H"`` (Norway/Svalbard exceptions not handled)."""
    number = int((lon_deg + 180.0) // 6.0) % 60 + 1
    band = _LAT_BANDS[min(max(int((lat_deg + 80.0) // 8.0), 0), len(_LAT_BANDS) - 1)]
    return f"{number}{band}"


def to_utm(lat_deg: float, lon_deg: float, zone_number: int | None = None) -> tuple[float, float, int, bool]:
    """Return ``(easting, northing, zone_number, northern)``."""
    if zone_number is None:
        zone_number = int((lon_deg + 180.0) // 6.0) % 60 + 1
    lon0 = math.radians((zone_number - 1) * 6 - 180 + 3)
    phi = math.radians(lat_deg)
    lam = math.radians(lon_deg) - lon0
    e = math.sqrt(_E2)
    t = math.sinh(math.atanh(math.sin(phi)) - e * math.atanh(e * math.sin(phi)))
    xi_p = math.atan2(t, math.cos(lam))
    eta_p = math.atanh(math.sin(lam) / math.sqrt(1.0 + t * t))
    xi, eta = xi_p, eta_p
    for j, a in enumerate(_ALPHA, start=1):
        xi += a * math.sin(2 * j * xi_p) * math.cosh(2 * j * eta_p)
        eta += a * math.cos(2 * j * xi_p) * math.sinh(2 * j * eta_p)
    easting = E0 + K0 * _A_HAT * eta
    northing = K0 * _A_HAT * xi
    northern = lat_deg >= 0.0
    if not northern:
        northing += N0_SOUTH
    return easting, northing, zone_number, northern


def from_utm(easting: float, northing: float, zone_number: int, northern: bool) -> tuple[float, float]:
    """Inverse of :func:`to_utm`: ``(lat_deg, lon_deg)``."""
    xi = (northing - (0.0 if northern else N0_SOUTH)) / (K0 * _A_HAT)
    eta = (easting - E0) / (K0 * _A_HAT)
    xi_p, eta_p = xi, eta
    for j, b in enumerate(_BETA, start=1):
        xi_p -= b * math.sin(2 * j * xi) * math.cosh(2 * j * eta)
        eta_p -= b * math.cos(2 * j * xi) * math.sinh(2 * j * eta)
    chi = math.asin(math.sin(xi_p) / math.cosh(eta_p))
    phi = chi + sum(d * math.sin(2 * j * chi) for j, d in enumerate(_DELTA, start=1))
    lon0 = math.radians((zone_number - 1) * 6 - 180 + 3)
    lam = lon0 + math.atan2(math.sinh(eta_p), math.cos(xi_p))
    return math.degrees(phi), math.degrees(lam)


def lla_to_enu(
    origin: tuple[float, float, float], lat: float, lon: float, alt: float
) -> tuple[float, float, float]:
    """Inverse of :func:`enu_to_lla` (same local approximation)."""
    lat0, lon0, alt0 = origin
    phi = math.radians(lat0)
    s2 = math.sin(phi) ** 2
    r_n = A / math.sqrt(1.0 - _E2 * s2)
    r_m = A * (1.0 - _E2) / (1.0 - _E2 * s2) ** 1.5
    return math.radians(lon - lon0) * r_n * math.cos(phi), math.radians(lat - lat0) * r_m, alt - alt0


def enu_to_lla(
    origin: tuple[float, float, float], east: float, north: float, up: float
) -> tuple[float, float, float]:
    """Local tangent-plane offset to latitude, longitude, altitude.

    Uses the meridian and prime-vertical radii at the origin. Over the few hundred metres
    of an operating area the error is millimetres, far below the sensor errors reported.
    """
    lat0, lon0, alt0 = origin
    phi = math.radians(lat0)
    s2 = math.sin(phi) ** 2
    r_n = A / math.sqrt(1.0 - _E2 * s2)  # prime vertical
    r_m = A * (1.0 - _E2) / (1.0 - _E2 * s2) ** 1.5  # meridian
    lat = lat0 + math.degrees(north / r_m)
    lon = lon0 + math.degrees(east / (r_n * math.cos(phi)))
    return lat, lon, alt0 + up
