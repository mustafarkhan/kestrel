"""The air picture as Cursor-on-Target events, for ATAK, WinTAK and other TAK clients.

SAPIENT is how sensors and fusion nodes talk to each other; Cursor on Target (CoT) is what
most tactical displays read. ``events`` turns one picture (as ``kestrel fusion`` writes it)
into CoT XML, one event per track, and ``CotSender`` puts them on the network as UDP
datagrams (unicast or the TAK multicast group, ``239.2.3.1:6969``).

Mapping:

* **Affiliation** is the track's standard identity: friend ``f``, assumed friend ``a``,
  neutral ``n``, suspect ``s``, unknown ``u``, pending ``p``, and hostile ``h`` only where an
  operator declared it. Kestrel never sets hostile by itself, and neither does this.
* **Type**: a track classified *UAV* is an air track, military rotary-wing drone
  (``a-.-A-M-H-Q``); anything else is a plain air track (``a-.-A``). Tracks classified *Bird*
  are not sent unless asked for.
* **Position** is WGS84 with height above the ellipsoid taken as the picture's altitude
  (the difference from the geoid is not applied: tens of metres, stated in ``le``), circular
  error ``ce`` and linear error ``le`` from the track's one-sigma position error.
* **Stale** time is short (default 5 s): a display must not show an air track that the
  picture has stopped confirming.

CoT over UDP is unauthenticated and unencrypted. Send it only on a network that is
trusted, or through a TAK server that adds TLS.
"""

from __future__ import annotations

import math
import socket
from datetime import UTC, datetime, timedelta
from typing import Any
from xml.sax.saxutils import quoteattr

AFFILIATION = {
    "FRIEND": "f",
    "ASSUMED_FRIEND": "a",
    "NEUTRAL": "n",
    "SUSPECT": "s",
    "UNKNOWN": "u",
    "PENDING": "p",
    "HOSTILE": "h",
}
GEOID_UNCERTAINTY_M = 50.0  # altitude is above the geoid; CoT wants the ellipsoid


def _time(dt: datetime) -> str:
    return dt.strftime("%Y-%m-%dT%H:%M:%S.") + f"{dt.microsecond // 1000:03d}Z"


def event(track: dict[str, Any], when: datetime, source: str = "KESTREL", stale_s: float = 5.0) -> str:
    """One picture track as a CoT event (XML text)."""
    aff = AFFILIATION.get(str(track.get("identity")), "u")
    kind = f"a-{aff}-A-M-H-Q" if track.get("class") == "UAV" else f"a-{aff}-A"
    sigma = float(track.get("sigma_m", 30.0))
    speed = (float(track["vel_mps"][0]) ** 2 + float(track["vel_mps"][1]) ** 2) ** 0.5
    course = math.degrees(math.atan2(float(track["vel_mps"][0]), float(track["vel_mps"][1]))) % 360.0
    name = f"{source}-{track['id']}"
    lat, lon, alt = float(track["lat"]), float(track["lon"]), float(track["alt_m"])
    remarks = (
        f"{track.get('identity', 'UNKNOWN')} {track.get('class', 'Unknown')} "
        f"p(uav)={track.get('p_uav', 0):.2f} sensors={','.join(track.get('sensors', []))}"
        + (f" remote_id={track['remote_id']}" if track.get("remote_id") else "")
        + (" own node" if track.get("own_node") else "")
    )
    return (
        f'<event version="2.0" uid={quoteattr(name)} type="{kind}" how="m-f" time="{_time(when)}" '
        f'start="{_time(when)}" stale="{_time(when + timedelta(seconds=stale_s))}">'
        f'<point lat="{lat:.7f}" lon="{lon:.7f}" hae="{alt:.1f}" '
        f'ce="{2.0 * sigma:.1f}" le="{2.0 * sigma + GEOID_UNCERTAINTY_M:.1f}"/>'
        f"<detail><contact callsign={quoteattr(name)}/>"
        f'<track course="{course:.1f}" speed="{speed:.1f}"/>'
        f"<remarks>{quoteattr(remarks)[1:-1]}</remarks></detail></event>"
    )


def events(
    picture: dict[str, Any], source: str = "KESTREL", birds: bool = False, stale_s: float = 5.0
) -> list[str]:
    when = datetime.fromisoformat(str(picture["timestamp"])).astimezone(UTC)
    return [
        event(tr, when, source, stale_s) for tr in picture["tracks"] if birds or tr.get("class") != "Bird"
    ]


class CotSender:
    """Sends CoT events as UDP datagrams; multicast addresses get a TTL of 1 (the local network)."""

    def __init__(self, host: str, port: int) -> None:
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if 224 <= int(host.split(".")[0]) <= 239:
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
        self.sent = 0

    def send(self, picture: dict[str, Any]) -> None:
        for xml in events(picture):
            try:
                self.sock.sendto(xml.encode("utf-8"), self.addr)
                self.sent += 1
            except OSError:
                pass  # a display that is not there must never stop the picture

    def close(self) -> None:
        self.sock.close()
