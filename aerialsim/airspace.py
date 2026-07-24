"""Airspace zone model and spatial lookup for the Athinai FIR (LGGG).

Zones are loaded from a JSON seed file. Geometry is either a circle
(center + radius_km) or a polygon (list of [lat, lon] vertices); vertical
limits are in feet AMSL (0 = SFC). The seed data ships with approximate
geometry — replace it with eAIP / OpenAIP exports for operational accuracy.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from .geo import point_in_circle, point_in_polygon

# Zone types a UAV must not enter without clearance / authorization.
HARD_NO_FLY = {"P", "R", "D"}          # prohibited / restricted / danger
CLEARANCE_REQUIRED = {"CTR", "TMA"}    # controlled airspace


@dataclass
class Zone:
    id: str
    name: str
    type: str                     # CTR | TMA | P | R | D
    lower_ft: float
    upper_ft: float
    geometry: dict = field(repr=False, default_factory=dict)
    approx: bool = True
    remarks: str = ""

    def contains(self, lat: float, lon: float, alt_ft: float) -> bool:
        if not (self.lower_ft <= alt_ft <= self.upper_ft):
            return False
        g = self.geometry
        if g["kind"] == "circle":
            return point_in_circle(lat, lon, tuple(g["center"]), g["radius_km"])
        if g["kind"] == "polygon":
            return point_in_polygon(lat, lon, [tuple(v) for v in g["vertices"]])
        raise ValueError(f"unknown geometry kind {g['kind']!r}")


class AirspaceIndex:
    def __init__(self, zones: list[Zone]):
        self.zones = zones

    @classmethod
    def from_json(cls, path: str | Path) -> "AirspaceIndex":
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls([Zone(**z) for z in raw["zones"]])

    def zones_at(self, lat: float, lon: float, alt_ft: float) -> list[Zone]:
        return [z for z in self.zones if z.contains(lat, lon, alt_ft)]

    def violations(self, lat: float, lon: float, alt_ft: float,
                   allowed_zone_ids: set[str] | None = None) -> list[Zone]:
        """Zones at this point that the flight is NOT allowed inside.

        allowed_zone_ids: zones for which clearance/authorization is assumed
        (e.g. a CTR the operator has coordinated with).
        """
        allowed = allowed_zone_ids or set()
        hits = []
        for z in self.zones_at(lat, lon, alt_ft):
            if z.id in allowed:
                continue
            if z.type in HARD_NO_FLY or z.type in CLEARANCE_REQUIRED:
                hits.append(z)
        return hits
