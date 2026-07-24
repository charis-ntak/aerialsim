"""Live traffic over Greece from the OpenSky Network (anonymous REST API).

Anonymous access is rate-limited (~100 requests/day per IP) — fine for
snapshots; register an OpenSky account for polling use.
"""
from __future__ import annotations

from dataclasses import dataclass

import requests

from .geo import haversine_km

OPENSKY_URL = "https://opensky-network.org/api/states/all"
GREECE_BBOX = (34.5, 41.8, 19.0, 29.7)   # lat_min, lat_max, lon_min, lon_max
M_TO_FT = 3.28084
MS_TO_KT = 1.943844
NM_PER_KM = 0.539957


@dataclass
class LiveAircraft:
    icao24: str
    callsign: str
    lat: float
    lon: float
    alt_ft: float          # barometric altitude
    gs_kt: float
    track_deg: float
    vrate_fpm: float


@dataclass
class Proximity:
    a: LiveAircraft
    b: LiveAircraft
    lateral_nm: float
    vertical_ft: float


def fetch_live_traffic(bbox: tuple[float, float, float, float] = GREECE_BBOX,
                       timeout_s: float = 30.0) -> list[LiveAircraft]:
    """One snapshot of airborne aircraft in the bbox. Raises on network error."""
    lat_min, lat_max, lon_min, lon_max = bbox
    r = requests.get(OPENSKY_URL, params={
        "lamin": lat_min, "lamax": lat_max, "lomin": lon_min, "lomax": lon_max,
    }, timeout=timeout_s)
    r.raise_for_status()
    states = r.json().get("states") or []
    out = []
    for s in states:
        # states/all vector: 0 icao24, 1 callsign, 5 lon, 6 lat, 7 baro_alt m,
        # 8 on_ground, 9 velocity m/s, 10 true_track, 11 vertical_rate m/s
        if s[8] or s[5] is None or s[6] is None or s[7] is None:
            continue
        out.append(LiveAircraft(
            icao24=s[0],
            callsign=(s[1] or "").strip() or s[0],
            lat=s[6], lon=s[5],
            alt_ft=s[7] * M_TO_FT,
            gs_kt=(s[9] or 0.0) * MS_TO_KT,
            track_deg=s[10] or 0.0,
            vrate_fpm=(s[11] or 0.0) * M_TO_FT * 60.0,
        ))
    return out


def snapshot_proximities(aircraft: list[LiveAircraft],
                         lateral_nm: float = 5.0, vertical_ft: float = 1000.0,
                         floor_ft: float = 5000.0) -> list[Proximity]:
    """Pairs currently closer than the separation minima (snapshot only —
    a legitimate climb/descent through another's level can appear here;
    it is a proximity screen, not a loss-of-separation verdict)."""
    hits = []
    cruisers = [a for a in aircraft if a.alt_ft >= floor_ft]
    for i in range(len(cruisers)):
        for j in range(i + 1, len(cruisers)):
            a, b = cruisers[i], cruisers[j]
            vert = abs(a.alt_ft - b.alt_ft)
            if vert >= vertical_ft:
                continue
            lat_nm = haversine_km(a.lat, a.lon, b.lat, b.lon) * NM_PER_KM
            if lat_nm < lateral_nm:
                hits.append(Proximity(a, b, lat_nm, vert))
    return sorted(hits, key=lambda p: p.lateral_nm)
