"""Layered 3D graph of the civil ATS route network.

Nodes:
  - ("APT", icao)      airport (ground)
  - (fix, fl)          en-route fix at a flight level, e.g. ("ATV", 340)

Edges (directed):
  - route segments per FL, direction-filtered by the ICAO semicircular rule
    (track 000-179 deg -> odd flight levels FL290/310/...; 180-359 -> even).
    Below RVSM the same parity logic is applied in 1000-ft steps for
    simplicity.
  - vertical climb/descent edges (fix, fl) <-> (fix, fl +/- 10)
  - airport <-> terminal-fix edges representing the climb/descent phase
    (SID/STAR abstraction).

Danger/restricted zones from the airspace index can block segments at the
levels they span when listed as active.
"""
from __future__ import annotations

import json
from pathlib import Path

import networkx as nx

from .airspace import AirspaceIndex
from .geo import bearing_deg, haversine_km

FL_LEVELS = list(range(200, 420, 10))          # FL200 ... FL410


def fl_is_odd(fl: int) -> bool:
    return (fl // 10) % 2 == 1


def fl_allowed_for_track(fl: int, track_deg: float) -> bool:
    """ICAO Annex 2 semicircular rule (using true track as an approximation)."""
    eastbound = 0.0 <= track_deg < 180.0
    return fl_is_odd(fl) if eastbound else not fl_is_odd(fl)


class RouteNetwork:
    def __init__(self, path: str | Path):
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        self.airports: dict = raw["airports"]
        self.waypoints: dict = raw["waypoints"]
        self.routes: list = raw["routes"]

    def build_graph(self) -> nx.DiGraph:
        g = nx.DiGraph()
        wp = self.waypoints

        for name, w in wp.items():
            for fl in FL_LEVELS:
                g.add_node((name, fl), lat=w["lat"], lon=w["lon"],
                           alt_ft=fl * 100, fix=name)

        for route in self.routes:
            pts = route["waypoints"]
            for a, b in zip(pts, pts[1:]):
                wa, wb = wp[a], wp[b]
                dist = haversine_km(wa["lat"], wa["lon"], wb["lat"], wb["lon"])
                fwd = bearing_deg(wa["lat"], wa["lon"], wb["lat"], wb["lon"])
                rev = (fwd + 180.0) % 360.0
                for fl in FL_LEVELS:
                    if fl_allowed_for_track(fl, fwd):
                        g.add_edge((a, fl), (b, fl), dist_km=dist,
                                   track=fwd, route=route["name"], kind="enroute")
                    if fl_allowed_for_track(fl, rev):
                        g.add_edge((b, fl), (a, fl), dist_km=dist,
                                   track=rev, route=route["name"], kind="enroute")

        # Vertical climb/descent edges at every fix.
        for name in wp:
            for lo, hi in zip(FL_LEVELS, FL_LEVELS[1:]):
                g.add_edge((name, lo), (name, hi), dfl=1000, kind="climb")
                g.add_edge((name, hi), (name, lo), dfl=1000, kind="descent")

        # Airport <-> terminal fix edges (climb-out / descent abstraction).
        for icao, apt in self.airports.items():
            n_apt = ("APT", icao)
            g.add_node(n_apt, lat=apt["lat"], lon=apt["lon"], alt_ft=0,
                       fix=icao, airport=True)
            for fix in apt["links"]:
                w = wp[fix]
                dist = haversine_km(apt["lat"], apt["lon"], w["lat"], w["lon"])
                for fl in FL_LEVELS:
                    g.add_edge(n_apt, (fix, fl), dist_km=dist,
                               climb_ft=fl * 100, kind="departure")
                    g.add_edge((fix, fl), n_apt, dist_km=dist,
                               descent_ft=fl * 100, kind="arrival")
        return g


def segment_blocked(g: nx.DiGraph, u, v, airspace: AirspaceIndex,
                    active_zone_ids: set[str], n_samples: int = 8) -> bool:
    """True if the u->v en-route segment crosses an ACTIVE hazard zone at its FL."""
    if not active_zone_ids:
        return False
    du, dv = g.nodes[u], g.nodes[v]
    alt_ft = max(du["alt_ft"], dv["alt_ft"])
    for i in range(n_samples + 1):
        f = i / n_samples
        lat = du["lat"] + f * (dv["lat"] - du["lat"])
        lon = du["lon"] + f * (dv["lon"] - du["lon"])
        for z in airspace.zones_at(lat, lon, alt_ft):
            if z.id in active_zone_ids:
                return True
    return False
