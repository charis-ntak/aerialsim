"""SIGPA multi-objective flight planning over the civil route graph.

Plans airport-to-airport flights with the swarm intelligence graph-based
pathfinding algorithm (SIGPA; Ntakolia & Iakovidis, Computers & Operations
Research 133 (2021) 105358, ``pip install git+https://github.com/charis-ntak/sigpa``)
instead of the single-cost A* of :mod:`aerialsim.atm_planner`.

Where A* minimizes one scalar (leg time x fuel factor), SIGPA evaluates
each candidate segment on four normalized measures and balances them with
its NRMSE criterion:

    risk      -- convective exposure (CAPE at the segment midpoint)
    duration  -- wind-adjusted leg time
    turn      -- track change between consecutive segments (geometric)
    loss      -- congestion of the ATC sector the segment crosses,
                 given the occupancy of the flights planned so far

Planning a departure wave sequentially therefore load-balances the
sectors: each flight sees the sector entries of the previously planned
flights as its ``loss`` measure.  The returned object is the same
:class:`aerialsim.atm_planner.FlightPlan`, so trajectories, separation
monitoring, regulation and the 3D view work unchanged.
"""
from __future__ import annotations

import math
import random
from typing import Optional

import networkx as nx
from sigpa import Graph as SigpaGraph
from sigpa import sigpa as sigpa_solve
from sigpa.graph import ArcData

from .aircraft import AircraftModel
from .airspace import AirspaceIndex
from .atm_planner import _leg_time_min, FlightPlan
from .routegraph import segment_blocked
from .sectors import Sector
from .weather import WindsAloftProvider

CONVECTIVE_CAPE_FULL_RISK = 3000.0   # CAPE (J/kg) mapped to risk = 1.0
HAZARD_TYPES = {"P", "R", "D"}       # prohibited / restricted / danger zones
KM_PER_DEG_LAT = 110.57

# Safety-dominant weighting of the three benchmark objectives
# (decided for the Greek-airspace case study): total = 1.0 x fuel-weighted
# time + 60 x danger-area exposure + 20 x sector congestion.
DEFAULT_WEIGHTS = (1.0, 60.0, 0.0, 20.0)   # (time, safety, turn, congestion)


def zone_exposure(airspace: AirspaceIndex, du: dict, dv: dict, fl: int) -> float:
    """Fraction of an en-route segment inside P/R/D hazard zones.

    Sampled at the endpoints and midpoint at the segment's flight level.
    This is the *soft* safety measure (exposure even to inactive zones);
    zones listed in ``active_zone_ids`` additionally hard-block segments
    exactly as in the A* planner.
    """
    alt_ft = fl * 100
    pts = ((du["lat"], du["lon"]),
           ((du["lat"] + dv["lat"]) / 2, (du["lon"] + dv["lon"]) / 2),
           (dv["lat"], dv["lon"]))
    inside = 0
    for lat, lon in pts:
        if any(z.type in HAZARD_TYPES and z.contains(lat, lon, alt_ft)
               for z in airspace.zones):
            inside += 1
    return inside / len(pts)


class ATMGraph(SigpaGraph):
    """Directed SIGPA graph over (fix, FL) nodes with ATM measures.

    The distance function toward the arrival airport is the *network*
    distance-to-go (reverse Dijkstra over segment lengths) rather than
    the straight-line distance -- the paper leaves the distance norm of
    the fitness function pluggable, and on a sparse airway network the
    straight line is misleading: a fix may be geometrically closer to
    the destination while its only airway continues away from it.
    """

    def __init__(self):
        super().__init__()
        self._durations: dict = {}
        self._measures: dict = {}
        self._ac: Optional[AircraftModel] = None
        self.target = None
        self._dist_to_go: Optional[dict] = None

    def add_directed_arc(self, i, j, risk, loss, distance, duration, energy):
        data = ArcData(risk=risk, cult=1.0, loss=loss,
                       distance=distance, energy=energy)
        self._adj[i][j] = data          # one direction only
        self._durations[(i, j)] = duration

    def travel_duration(self, i, j):
        """Duration measure for the NRMSE criterion.

        The greedy search is myopic: a departure leg to a low flight
        level is genuinely cheaper *on that leg*, while its fuel penalty
        only materializes over the rest of the cruise.  We therefore use
        the potential-shaped reduced cost

            measure(i, j) = cost(i, j) + h(j) - h(i)

        with h(n) = fuel_factor(FL(n)) x time-to-go(n), so every step
        sees the downstream fuel implication of the level it commits to.
        The shaping telescopes along a route, leaving route rankings
        identical to the true fuel-weighted cost.
        """
        return self._measures.get((i, j), self._adj[i][j].energy)

    def leg_time(self, i, j):
        """Raw wind-adjusted leg time (min)."""
        return self._durations[(i, j)]

    def _potential(self, n) -> float:
        d = self._dist_to_go.get(n, 1e9)
        ff = 1.0 if (isinstance(n, tuple) and n[0] == "APT") \
            else self._ac.fuel_factor(n[1])
        return ff * d / (self._ac.cruise_tas_kt * 1.852 / 60)

    def shape_measures(self) -> None:
        """Fill the potential-shaped duration measures (needs target + ac)."""
        if self._ac is None or self._dist_to_go is None:
            return
        for i, nbrs in self._adj.items():
            for j, data in nbrs.items():
                self._measures[(i, j)] = (
                    data.energy + self._potential(j) - self._potential(i)
                )

    def set_target(self, target) -> None:
        """Precompute network distance-to-go toward ``target`` (Dijkstra
        on the reversed arc set; vertical arcs cost a small epsilon so
        gratuitous level changes read as negative progress)."""
        import heapq

        dist = {target: 0.0}
        reverse: dict = {}
        for i, nbrs in self._adj.items():
            for j, data in nbrs.items():
                reverse.setdefault(j, []).append((i, data.distance + 0.5))
        heap = [(0.0, repr(target), target)]
        while heap:
            d, _, n = heapq.heappop(heap)
            if d > dist.get(n, float("inf")):
                continue
            for m, w in reverse.get(n, ()):
                nd = d + w
                if nd < dist.get(m, float("inf")):
                    dist[m] = nd
                    heapq.heappush(heap, (nd, repr(m), m))
        self.target = target
        self._dist_to_go = dist

    def euclidean(self, i, j):
        if j == self.target and self._dist_to_go is not None:
            return self._dist_to_go.get(i, 1e9)
        return super().euclidean(i, j)


def _project(lat: float, lon: float, lat0: float, lon0: float):
    return ((lon - lon0) * KM_PER_DEG_LAT * math.cos(math.radians(lat0)),
            (lat - lat0) * KM_PER_DEG_LAT)


def _segment_sector(sectors, mid_lat, mid_lon, fl) -> Optional[str]:
    for s in sectors:
        if s.contains(mid_lat, mid_lon, fl * 100):
            return s.id
    return None


def build_sigpa_graph(
    g: nx.DiGraph,
    ac: AircraftModel,
    winds: WindsAloftProvider,
    airspace: AirspaceIndex,
    sectors: list[Sector],
    dep_icao: Optional[str] = None,
    arr_icao: Optional[str] = None,
    occupancy: Optional[dict] = None,
    capacities: Optional[dict] = None,
    active_zone_ids: Optional[set] = None,
    forbidden_fls: Optional[set] = None,
) -> ATMGraph:
    """Project the networkx route graph into a SIGPA graph for one aircraft.

    ``occupancy`` maps sector id -> number of already-planned flights
    entering it; with ``capacities`` (sector id -> declared capacity) it
    becomes the normalized ``loss`` measure of every segment in that
    sector.
    """
    occupancy = occupancy or {}
    capacities = capacities or {}
    active = active_zone_ids or set()
    forbidden = forbidden_fls or set()

    def usable(n) -> bool:
        if isinstance(n, tuple) and n[0] == "APT":
            # only the flight's own airports; no landing at intermediates
            return dep_icao is None or n[1] in (dep_icao, arr_icao)
        return ac.min_cruise_fl <= n[1] <= ac.max_fl and n[1] not in forbidden

    lats = [d["lat"] for _, d in g.nodes(data=True)]
    lons = [d["lon"] for _, d in g.nodes(data=True)]
    lat0, lon0 = sum(lats) / len(lats), sum(lons) / len(lons)

    sg = ATMGraph()
    for n, d in g.nodes(data=True):
        if usable(n):
            x, y = _project(d["lat"], d["lon"], lat0, lon0)
            sg.add_node(n, x, y)

    for u, v, attrs in g.edges(data=True):
        if not (usable(u) and usable(v)):
            continue
        if active and attrs["kind"] == "enroute" and \
                segment_blocked(g, u, v, airspace, active):
            continue

        cost, info = _leg_time_min(g, u, v, attrs, ac, winds)
        duration = info["time_min"]
        distance = info.get("dist_km", 0.0)

        risk = 0.0
        loss = 0.0
        if attrs["kind"] == "enroute":
            du, dv = g.nodes[u], g.nodes[v]
            mid_lat = (du["lat"] + dv["lat"]) / 2
            mid_lon = (du["lon"] + dv["lon"]) / 2
            w = winds.sample(mid_lat, mid_lon)
            cape_risk = min(1.0, max(0.0, w.cape_jkg / CONVECTIVE_CAPE_FULL_RISK))
            # safety measure: danger-area exposure, with convective risk
            # folded in on days when CAPE is present
            risk = max(cape_risk, zone_exposure(airspace, du, dv, u[1]))
            sector = _segment_sector(sectors, mid_lat, mid_lon, u[1])
            if sector is not None:
                cap = capacities.get(sector, 6)
                loss = min(1.0, occupancy.get(sector, 0) / max(1, cap))

        sg.add_directed_arc(u, v, risk=risk, loss=loss, distance=distance,
                            duration=duration, energy=cost)

    sg._ac = ac
    if arr_icao is not None:
        sg.set_target(("APT", arr_icao))
        sg.shape_measures()
    return sg


class ATMRouteEvaluator:
    """Route score for the SIGPA swarm: weighted sum of the four ATM terms.

    total = w_time * sum(time x fuel factor)          [min-equivalent]
          + w_risk * sum(convective risk per segment) [scaled]
          + w_turn * sum(track-change penalties)      [scaled]
          + w_load * sum(sector-congestion factors)   [scaled]
    """

    def __init__(self, graph: ATMGraph,
                 weights=DEFAULT_WEIGHTS):
        self.graph = graph
        self.w_time, self.w_risk, self.w_turn, self.w_load = weights

    def __call__(self, route) -> float:
        g = self.graph
        time_cost = risk = turn = load = 0.0
        for idx in range(len(route) - 1):
            i, j = route[idx], route[idx + 1]
            data = g.arc(i, j)
            time_cost += data.energy      # leg time x fuel factor
            risk += data.risk
            load += data.loss
            if idx + 2 < len(route):
                turn += g.turn_penalty(i, j, route[idx + 2])
        return (self.w_time * time_cost + self.w_risk * risk
                + self.w_turn * turn + self.w_load * load)


def plan_flight_sigpa(
    g: nx.DiGraph,
    dep_icao: str,
    arr_icao: str,
    ac: AircraftModel,
    winds: WindsAloftProvider,
    airspace: AirspaceIndex,
    sectors: list[Sector],
    occupancy: Optional[dict] = None,
    capacities: Optional[dict] = None,
    active_zone_ids: Optional[set] = None,
    forbidden_fls: Optional[set] = None,
    weights=DEFAULT_WEIGHTS,
    k: int = 3,
    max_iterations: int = 400,
    max_no_improve: int = 40,
    rng: Optional[random.Random] = None,
    arc_evaluator=None,
) -> FlightPlan:
    """Plan one flight with SIGPA; drop-in alternative to ``plan_flight``.

    ``arc_evaluator`` optionally replaces the NRMSE candidate criterion
    (e.g. a SIGPA-LLM evolved ``WeightedNRMSE``)."""
    sg = build_sigpa_graph(g, ac, winds, airspace, sectors,
                           dep_icao=dep_icao, arr_icao=arr_icao,
                           occupancy=occupancy, capacities=capacities,
                           active_zone_ids=active_zone_ids,
                           forbidden_fls=forbidden_fls)

    s, t = ("APT", dep_icao), ("APT", arr_icao)
    if s not in set(sg.nodes) or t not in set(sg.nodes):
        return FlightPlan(False, f"unknown airport {dep_icao} or {arr_icao}")

    try:
        result = sigpa_solve(
            sg, s, t, pois=[],
            k=k, max_iterations=max_iterations, max_no_improve=max_no_improve,
            evaluator=ATMRouteEvaluator(sg, weights),
            rng=rng or random.Random(0),
            arc_evaluator=arc_evaluator,
        )
    except RuntimeError:
        return FlightPlan(False, "no route: active zones / FL restrictions block all paths",
                          weather_synthetic=winds.used_fallback)

    path = result.best_route
    legs, total_t, total_d, warnings = [], 0.0, 0.0, []
    for u, v in zip(path, path[1:]):
        _, info = _leg_time_min(g, u, v, g.edges[u, v], ac, winds)
        legs.append(info)
        total_t += info["time_min"]
        total_d += info.get("dist_km", 0.0)
        if info.get("convective"):
            warnings.append(f"{u} -> {v}: CAPE {info['cape']:.0f} J/kg")

    cruise = sorted({n[1] for n in path if not (isinstance(n, tuple) and n[0] == "APT")})
    return FlightPlan(True, nodes=path, legs=legs,
                      distance_km=total_d, time_min=total_t,
                      cruise_fls=cruise, convective_warnings=warnings,
                      weather_synthetic=winds.used_fallback)


def sectors_entered(g: nx.DiGraph, plan: FlightPlan,
                    sectors: list[Sector]) -> set:
    """Sector ids crossed by a plan (for sequential occupancy updates)."""
    entered = set()
    for u, v in zip(plan.nodes, plan.nodes[1:]):
        if not (isinstance(u, tuple) and u[0] != "APT"):
            continue
        if isinstance(v, tuple) and v[0] == "APT":
            continue
        du, dv = g.nodes[u], g.nodes[v]
        sector = _segment_sector(sectors, (du["lat"] + dv["lat"]) / 2,
                                 (du["lon"] + dv["lon"]) / 2, u[1])
        if sector:
            entered.add(sector)
    return entered
