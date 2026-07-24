"""Weather-aware A* route planning over the UAV grid."""
from __future__ import annotations

from dataclasses import dataclass, field

import networkx as nx

from .geo import bearing_deg, haversine_km, wind_components
from .graph3d import GridNode, nearest_node
from .uav import UAVModel
from .weather import OpenMeteoProvider


@dataclass
class PlanResult:
    feasible: bool
    reason: str = ""
    path_nodes: list[GridNode] = field(default_factory=list)
    path_coords: list[tuple[float, float, float]] = field(default_factory=list)  # lat, lon, alt_m
    distance_km: float = 0.0
    flight_time_min: float = 0.0
    max_wind_ms: float = 0.0
    max_gust_ms: float = 0.0
    weather_warnings: list[str] = field(default_factory=list)
    weather_synthetic: bool = False


def plan_route(
    g: nx.Graph,
    start: tuple[float, float],
    goal: tuple[float, float],
    uav: UAVModel,
    weather: OpenMeteoProvider,
) -> PlanResult:
    """A* over the grid; edge cost = traversal time including head/tailwind.

    Weather is sampled per ~0.25 deg cell (provider caches), so the wind
    penalty varies along the route. Nodes whose weather violates the UAV's
    hard limits are excluded up front.
    """
    # 1. Pre-flight weather gate: remove nodes where the UAV cannot fly at all.
    bad_nodes, warnings = [], set()
    for n, d in g.nodes(data=True):
        w = weather.sample(d["lat"], d["lon"])
        problems = uav.check_flyable(w, d["alt_m"])
        if problems:
            bad_nodes.append(n)
            warnings.update(problems)
    work = g.copy()
    work.remove_nodes_from(bad_nodes)
    if work.number_of_nodes() == 0:
        return PlanResult(False, "weather exceeds UAV limits everywhere in the area",
                          weather_warnings=sorted(warnings),
                          weather_synthetic=weather.used_fallback)

    try:
        s = nearest_node(work, *start, alt_m=uav.cruise_alt_agl_m)
        t = nearest_node(work, *goal, alt_m=uav.cruise_alt_agl_m)
    except ValueError as e:
        return PlanResult(False, str(e))

    # 2. Edge weight: time in hours, wind-adjusted ground speed.
    def edge_time_h(a: GridNode, b: GridNode, attrs: dict) -> float:
        da, db = work.nodes[a], work.nodes[b]
        if attrs.get("vertical"):
            return attrs["dist_km"] / (5.0 * 3.6)   # ~5 m/s climb/descent
        track = bearing_deg(da["lat"], da["lon"], db["lat"], db["lon"])
        w = weather.sample((da["lat"] + db["lat"]) / 2, (da["lon"] + db["lon"]) / 2)
        speed, direction = w.wind_at(da["alt_m"])
        headwind, _ = wind_components(track, direction, speed)
        ground_ms = max(2.0, uav.cruise_speed_ms - headwind)
        return attrs["dist_km"] / (ground_ms * 3.6)

    def heuristic(a: GridNode, b: GridNode) -> float:
        da, db = work.nodes[a], work.nodes[b]
        d = haversine_km(da["lat"], da["lon"], db["lat"], db["lon"])
        best_ms = uav.cruise_speed_ms + 20.0   # admissible: assumes strong tailwind
        return d / (best_ms * 3.6)

    try:
        path = nx.astar_path(work, s, t, heuristic=heuristic, weight=edge_time_h)
    except nx.NetworkXNoPath:
        return PlanResult(False,
                          "no path: airspace restrictions and/or weather block the corridor",
                          weather_warnings=sorted(warnings),
                          weather_synthetic=weather.used_fallback)

    # 3. Stats along the chosen path.
    dist = time_h = 0.0
    max_wind = max_gust = 0.0
    for a, b in zip(path, path[1:]):
        attrs = work.edges[a, b]
        dist += attrs["dist_km"]
        time_h += edge_time_h(a, b, attrs)
        da = work.nodes[a]
        w = weather.sample(da["lat"], da["lon"])
        wind_speed, _ = w.wind_at(da["alt_m"])
        max_wind = max(max_wind, wind_speed)
        max_gust = max(max_gust, w.wind_gusts_10m)

    time_min = time_h * 60.0
    coords = [(work.nodes[n]["lat"], work.nodes[n]["lon"], work.nodes[n]["alt_m"]) for n in path]
    feasible = time_min <= uav.endurance_min
    reason = "" if feasible else (
        f"flight time {time_min:.0f} min exceeds endurance {uav.endurance_min:.0f} min")
    return PlanResult(
        feasible=feasible, reason=reason,
        path_nodes=path, path_coords=coords,
        distance_km=dist, flight_time_min=time_min,
        max_wind_ms=max_wind, max_gust_ms=max_gust,
        weather_warnings=sorted(warnings),
        weather_synthetic=weather.used_fallback,
    )
