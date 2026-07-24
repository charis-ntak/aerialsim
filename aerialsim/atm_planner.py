"""Flight planning over the civil route graph: wind-optimal A* airport to airport."""
from __future__ import annotations

from dataclasses import dataclass, field

import networkx as nx

from .aircraft import KT_PER_MS, AircraftModel
from .airspace import AirspaceIndex
from .geo import haversine_km, wind_components
from .routegraph import segment_blocked
from .weather import WindsAloftProvider

CONVECTIVE_CAPE_JKG = 1500.0     # above this, a segment gets a cost penalty
CONVECTIVE_PENALTY = 1.6


@dataclass
class FlightPlan:
    ok: bool
    reason: str = ""
    nodes: list = field(default_factory=list)          # ("APT", icao) / (fix, fl)
    legs: list = field(default_factory=list)           # per-leg dicts (see below)
    distance_km: float = 0.0
    time_min: float = 0.0
    cruise_fls: list[int] = field(default_factory=list)
    convective_warnings: list[str] = field(default_factory=list)
    weather_synthetic: bool = False


def _leg_time_min(g: nx.DiGraph, u, v, attrs: dict,
                  ac: AircraftModel, winds: WindsAloftProvider) -> tuple[float, dict]:
    du, dv = g.nodes[u], g.nodes[v]
    kind = attrs["kind"]
    info = {"from": u, "to": v, "kind": kind}

    if kind in ("climb", "descent"):
        rate = ac.climb_fpm if kind == "climb" else ac.descent_fpm
        t = attrs["dfl"] / rate
        info.update(dist_km=0.0, gs_kt=0.0, time_min=t)
        return t, info

    dist = attrs["dist_km"]
    if kind in ("departure", "arrival"):
        # Climb/descent happens WHILE flying toward/from the fix, so the leg
        # takes whichever is longer: the vertical profile or the ground track.
        vert_ft = attrs.get("climb_ft") or attrs.get("descent_ft")
        rate = ac.climb_fpm if kind == "departure" else ac.descent_fpm
        t = max(vert_ft / rate, dist / (ac.cruise_tas_kt * 0.7 * 1.852 / 60))
        info.update(dist_km=dist, gs_kt=ac.cruise_tas_kt * 0.7, time_min=t)
        return t, info

    # En-route cruise leg with winds aloft.
    fl = u[1]
    mid_lat = (du["lat"] + dv["lat"]) / 2
    mid_lon = (du["lon"] + dv["lon"]) / 2
    w = winds.sample(mid_lat, mid_lon)
    speed_ms, direction = w.wind_at_fl(fl)
    headwind_kt = wind_components(attrs["track"], direction, speed_ms)[0] * KT_PER_MS
    gs_kt = max(120.0, ac.cruise_tas_kt - headwind_kt)
    t = dist / (gs_kt * 1.852 / 60)          # kt -> km/min
    cost = t * ac.fuel_factor(fl)
    if w.cape_jkg > CONVECTIVE_CAPE_JKG:
        cost *= CONVECTIVE_PENALTY
        info["convective"] = True
        info["cape"] = w.cape_jkg
    info.update(dist_km=dist, gs_kt=gs_kt, headwind_kt=headwind_kt,
                fl=fl, time_min=t)
    return cost, info


def plan_flight(
    g: nx.DiGraph,
    dep_icao: str,
    arr_icao: str,
    ac: AircraftModel,
    winds: WindsAloftProvider,
    airspace: AirspaceIndex,
    active_zone_ids: set[str] | None = None,
    forbidden_fls: set[int] | None = None,
) -> FlightPlan:
    active = active_zone_ids or set()
    forbidden = forbidden_fls or set()

    def usable(n) -> bool:
        if isinstance(n, tuple) and n[0] == "APT":
            return True
        fl = n[1]
        return ac.min_cruise_fl <= fl <= ac.max_fl and fl not in forbidden

    work = g.subgraph([n for n in g.nodes if usable(n)]).copy()
    if active:
        drop = [(u, v) for u, v, d in work.edges(data=True)
                if d["kind"] == "enroute" and segment_blocked(work, u, v, airspace, active)]
        work.remove_edges_from(drop)

    s, t = ("APT", dep_icao), ("APT", arr_icao)
    if s not in work or t not in work:
        return FlightPlan(False, f"unknown airport {dep_icao} or {arr_icao}")

    def weight(u, v, attrs):
        return _leg_time_min(work, u, v, attrs, ac, winds)[0]

    def heuristic(a, b):
        da, db = work.nodes[a], work.nodes[b]
        d = haversine_km(da["lat"], da["lon"], db["lat"], db["lon"])
        return d / ((ac.cruise_tas_kt + 150) * 1.852 / 60)

    try:
        path = nx.astar_path(work, s, t, heuristic=heuristic, weight=weight)
    except nx.NetworkXNoPath:
        return FlightPlan(False, "no route: active zones / FL restrictions block all paths",
                          weather_synthetic=winds.used_fallback)

    legs, total_t, total_d, warnings = [], 0.0, 0.0, []
    for u, v in zip(path, path[1:]):
        _, info = _leg_time_min(work, u, v, work.edges[u, v], ac, winds)
        legs.append(info)
        total_t += info["time_min"]
        total_d += info.get("dist_km", 0.0)
        if info.get("convective"):
            warnings.append(
                f"convective risk (CAPE {info['cape']:.0f} J/kg) on {u}->{v}")

    return FlightPlan(
        ok=True, nodes=path, legs=legs,
        distance_km=total_d, time_min=total_t,
        cruise_fls=sorted({leg["fl"] for leg in legs if "fl" in leg}),
        convective_warnings=warnings,
        weather_synthetic=winds.used_fallback,
    )
