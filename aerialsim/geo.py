"""Geodesic helpers: distances, bearings, point-in-zone tests."""
from __future__ import annotations

import math

EARTH_RADIUS_KM = 6371.0
FT_PER_M = 3.28084


def haversine_km(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_KM * math.asin(math.sqrt(a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x = math.sin(dl) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def point_in_polygon(lat: float, lon: float, polygon: list[tuple[float, float]]) -> bool:
    """Ray-casting test. polygon is a list of (lat, lon) vertices."""
    inside = False
    n = len(polygon)
    j = n - 1
    for i in range(n):
        yi, xi = polygon[i]
        yj, xj = polygon[j]
        if (yi > lat) != (yj > lat):
            x_cross = xj + (lat - yj) / (yi - yj) * (xi - xj)
            if lon < x_cross:
                inside = not inside
        j = i
    return inside


def point_in_circle(lat: float, lon: float, center: tuple[float, float], radius_km: float) -> bool:
    return haversine_km(lat, lon, center[0], center[1]) <= radius_km


def wind_components(track_deg: float, wind_from_deg: float, wind_speed: float) -> tuple[float, float]:
    """Return (headwind, crosswind) in the wind's units along a given track.

    Positive headwind opposes motion; wind_from_deg is the meteorological
    direction the wind blows FROM.
    """
    rel = math.radians(wind_from_deg - track_deg)
    headwind = wind_speed * math.cos(rel)
    crosswind = wind_speed * math.sin(rel)
    return headwind, crosswind
