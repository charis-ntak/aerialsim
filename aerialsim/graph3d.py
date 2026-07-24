"""3D graph construction.

Two layers share one coordinate convention (altitude in metres AMSL-ish,
treated as AGL over sea for the maritime scenarios):

1. UAV grid — a lat/lon lattice at several very-low-level altitude layers
   (e.g. 60/100/120 m), 8-connected horizontally plus vertical edges,
   with nodes removed where they fall inside airspace the operator has no
   authorization for.
2. Civil route graph — ATS route segments at flight levels, kept for
   context, separation awareness, and visualization.
"""
from __future__ import annotations

import json
from pathlib import Path

import networkx as nx

from .airspace import AirspaceIndex
from .geo import FT_PER_M, haversine_km

GridNode = tuple[int, int, int]  # (i lat index, j lon index, k altitude index)


def build_uav_grid(
    bbox: tuple[float, float, float, float],   # (lat_min, lat_max, lon_min, lon_max)
    spacing_km: float,
    alt_layers_m: list[float],
    airspace: AirspaceIndex,
    allowed_zone_ids: set[str] | None = None,
) -> nx.Graph:
    lat_min, lat_max, lon_min, lon_max = bbox
    lat_step = spacing_km / 111.0
    import math
    lon_step = spacing_km / (111.0 * max(0.2, math.cos(math.radians((lat_min + lat_max) / 2))))

    n_lat = max(2, int((lat_max - lat_min) / lat_step) + 1)
    n_lon = max(2, int((lon_max - lon_min) / lon_step) + 1)

    g = nx.Graph()
    blocked = 0
    for i in range(n_lat):
        lat = lat_min + i * lat_step
        for j in range(n_lon):
            lon = lon_min + j * lon_step
            for k, alt_m in enumerate(alt_layers_m):
                viol = airspace.violations(lat, lon, alt_m * FT_PER_M, allowed_zone_ids)
                if viol:
                    blocked += 1
                    continue
                g.add_node((i, j, k), lat=lat, lon=lon, alt_m=alt_m)

    # Horizontal edges: 8-connectivity within each altitude layer.
    offsets = [(1, 0), (0, 1), (1, 1), (1, -1)]
    for (i, j, k) in list(g.nodes):
        a = g.nodes[(i, j, k)]
        for di, dj in offsets:
            nb = (i + di, j + dj, k)
            if nb in g:
                b = g.nodes[nb]
                g.add_edge((i, j, k), nb,
                           dist_km=haversine_km(a["lat"], a["lon"], b["lat"], b["lon"]))
        # Vertical edge to the next altitude layer.
        up = (i, j, k + 1)
        if up in g:
            dz_km = abs(alt_layers_m[k + 1] - alt_layers_m[k]) / 1000.0
            g.add_edge((i, j, k), up, dist_km=dz_km, vertical=True)

    g.graph["blocked_nodes"] = blocked
    g.graph["alt_layers_m"] = alt_layers_m
    return g


def nearest_node(g: nx.Graph, lat: float, lon: float, alt_m: float | None = None) -> GridNode:
    best, best_d = None, float("inf")
    for n, d in g.nodes(data=True):
        dd = haversine_km(lat, lon, d["lat"], d["lon"])
        if alt_m is not None:
            dd += abs(d["alt_m"] - alt_m) / 1000.0
        if dd < best_d:
            best, best_d = n, dd
    if best is None:
        raise ValueError("graph has no nodes (all blocked by airspace?)")
    return best


def build_civil_graph(routes_path: str | Path) -> nx.Graph:
    raw = json.loads(Path(routes_path).read_text(encoding="utf-8"))
    wp = raw["waypoints"]
    g = nx.Graph()
    for route in raw["routes"]:
        alt_m = route["fl_used"] * 100 / FT_PER_M   # FL -> feet -> metres
        pts = route["waypoints"]
        for name in pts:
            w = wp[name]
            g.add_node((name, route["fl_used"]),
                       lat=w["lat"], lon=w["lon"], alt_m=alt_m,
                       fix=name, route=route["name"])
        for a, b in zip(pts, pts[1:]):
            g.add_edge((a, route["fl_used"]), (b, route["fl_used"]),
                       route=route["name"],
                       dist_km=haversine_km(wp[a]["lat"], wp[a]["lon"],
                                            wp[b]["lat"], wp[b]["lon"]))
    return g
