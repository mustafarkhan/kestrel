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
