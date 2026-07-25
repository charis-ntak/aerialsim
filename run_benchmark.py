"""Exact-vs-heuristic benchmark on real Greek domestic traffic.

Objectives (decided for the Greek-airspace case study; all edge-additive,
safety-dominant weighting):

    J = 1.0 x fuel-weighted flight time            [min-equivalent]
      + 60  x danger-area exposure                 [P/R/D zone crossing]
      + 20  x ATC-sector congestion                [occupancy / capacity]

Every unique method plans the full historical day sequentially (flights
in real departure order); each method maintains its own sector-occupancy
stream, so congestion is endogenous to its own planning decisions.
Because all three terms are edge-additive, J reduces to a scalar edge
weight per flight and the exact methods stay exact:

    CPLEX        0-1 flow MILP per flight (IBM ILOG CPLEX via docplex)
    Dijkstra     label-setting shortest path (networkx)
    Bellman-Ford label-correcting shortest path (networkx)
    A*           admissible-heuristic search (networkx, time heuristic)
    SIGPA        swarm intelligence graph-based pathfinding (COR 2021)
    SIGPA-LLM    SIGPA with an offline-designed evaluation criterion

CPLEX / Dijkstra / Bellman-Ford / A* must agree per flight -- they
cross-validate the model.  SIGPA and SIGPA-LLM are anytime
metaheuristics; the report shows their per-day optimality gap, the
objective breakdown, and per-flight planning runtimes.

Usage:
    python run_benchmark.py scenarios/atm_real_20250610.yaml
"""
from __future__ import annotations

import argparse
import random
import time
from collections import defaultdict

import networkx as nx

from aerialsim.aircraft import AIRCRAFT_CATALOG
from aerialsim.airspace import AirspaceIndex
from aerialsim.atm_planner import _leg_time_min
from aerialsim.geo import haversine_km
from aerialsim.routegraph import RouteNetwork
from aerialsim.scenario_atm import ATMScenario, DATA_DIR
from aerialsim.sectors import load_sectors
from aerialsim.sigpa_planner import (
    DEFAULT_WEIGHTS,
    _segment_sector,
    build_sigpa_graph,
    plan_flight_sigpa,
    zone_exposure,
)
from aerialsim.weather import WindsAloftProvider

from sigpa import sigpa_llm_train

W_TIME, W_SAFETY, _, W_LOAD = DEFAULT_WEIGHTS
DEFAULT_CAPACITY = 6

METHODS = ["CPLEX", "Dijkstra", "Bellman-Ford", "A*", "SIGPA", "SIGPA-LLM"]


def usable_subgraph(g, ac):
    def ok(n):
        if isinstance(n, tuple) and n[0] == "APT":
            return True
        return ac.min_cruise_fl <= n[1] <= ac.max_fl
    return g.subgraph([n for n in g.nodes if ok(n)])


def edge_measures(g, ac, winds, airspace, sectors):
    """Per-edge (cost, safety, sector) -- occupancy-independent parts."""
    out = {}
    for u, v, d in g.edges(data=True):
        cost = _leg_time_min(g, u, v, d, ac, winds)[0]
        safety, sector = 0.0, None
        if d["kind"] == "enroute":
            du, dv = g.nodes[u], g.nodes[v]
            w = winds.sample((du["lat"] + dv["lat"]) / 2,
                             (du["lon"] + dv["lon"]) / 2)
            cape = min(1.0, max(0.0, w.cape_jkg / 3000.0))
            safety = max(cape, zone_exposure(airspace, du, dv, u[1]))
            sector = _segment_sector(sectors,
                                     (du["lat"] + dv["lat"]) / 2,
                                     (du["lon"] + dv["lon"]) / 2, u[1])
        out[(u, v)] = (cost, safety, sector)
    return out


def combined_weight(measures, occupancy, capacities):
    """J as a scalar edge weight given a sector-occupancy state."""
    w = {}
    for e, (cost, safety, sector) in measures.items():
        load = 0.0
        if sector is not None:
            cap = capacities.get(sector, DEFAULT_CAPACITY)
            load = min(1.0, occupancy.get(sector, 0) / max(1, cap))
        w[e] = W_TIME * cost + W_SAFETY * safety + W_LOAD * load
    return w


def path_objective(path, weights):
    return sum(weights[(u, v)] for u, v in zip(path, path[1:]))


def path_breakdown(path, measures, occupancy, capacities):
    cost = safety = load = 0.0
    for u, v in zip(path, path[1:]):
        c, s, sector = measures[(u, v)]
        cost += c
        safety += s
        if sector is not None:
            cap = capacities.get(sector, DEFAULT_CAPACITY)
            load += min(1.0, occupancy.get(sector, 0) / max(1, cap))
    return cost, safety, load


def sectors_of_path(path, measures):
    return {sector for e in zip(path, path[1:])
            for sector in [measures[e][2]] if sector is not None}


def cplex_exact(g, weights, s, t):
    from docplex.mp.model import Model

    m = Model(name="atm_J", log_output=False)
    x = {e: m.binary_var(name=f"x_{i}") for i, e in enumerate(weights)}
    m.minimize(m.sum(weights[e] * x[e] for e in weights))
    out_by, in_by = defaultdict(list), defaultdict(list)
    for (u, v), var in x.items():
        out_by[u].append(var)
        in_by[v].append(var)
    for n in g.nodes:
        balance = 1 if n == s else (-1 if n == t else 0)
        m.add_constraint(m.sum(out_by[n]) - m.sum(in_by[n]) == balance)
    sol = m.solve()
    if sol is None:
        return None
    path_edges = {u: v for (u, v), var in x.items() if sol.get_value(var) > 0.5}
    path, cur = [s], s
    while cur != t:
        cur = path_edges[cur]
        path.append(cur)
    return path


def main():
    ap = argparse.ArgumentParser(description="exact vs heuristic benchmark")
    ap.add_argument("scenario")
    ap.add_argument("--llm-generations", type=int, default=8)
    args = ap.parse_args()

    sc = ATMScenario.from_yaml(args.scenario)
    g = RouteNetwork(DATA_DIR / "network_gr.json").build_graph()
    airspace = AirspaceIndex.from_json(DATA_DIR / "airspace_gr.json")
    winds = WindsAloftProvider(hour_offset=sc.hour_offset)
    sectors = load_sectors(DATA_DIR / "sectors_gr.json")
    capacities = dict(sc.sector_capacities or {})

    flights = sorted(sc.flights, key=lambda f: f.get("dep_time_min", 0))
    print(f"=== Exact vs heuristics: {sc.name} ===")
    print(f"objective J = {W_TIME:g} x time_cost + {W_SAFETY:g} x danger exposure "
          f"+ {W_LOAD:g} x sector congestion (safety-dominant)")
    print(f"{len(flights)} flights planned sequentially in departure order; "
          "each method evolves its own sector occupancy\n")

    # per-aircraft caches
    measures_by_ac, sub_by_ac = {}, {}
    for key in {f["aircraft"] for f in flights}:
        ac = AIRCRAFT_CATALOG[key]
        sub = usable_subgraph(g, ac)
        sub_by_ac[key] = sub
        measures_by_ac[key] = edge_measures(sub, ac, winds, airspace, sectors)

    # ---- SIGPA-LLM offline phase on this day's city pairs (objective J,
    # empty occupancy), matched to deployment conditions
    pairs = sorted({(f["from"], f["to"], f["aircraft"]) for f in flights})
    print(f"SIGPA-LLM offline design ({args.llm_generations} generations; "
          "LLM candidates if credentials are available, mutation fallback "
          "otherwise) ...")
    train = []
    for dep, arr, ackey in pairs:
        ac = AIRCRAFT_CATALOG[ackey]
        sg = build_sigpa_graph(g, ac, winds, airspace, sectors,
                               dep_icao=dep, arr_icao=arr)
        train.append((sg, ("APT", dep), ("APT", arr), []))

    def metric_J(graph, route):
        total = 0.0
        for i, j in zip(route, route[1:]):
            a = graph.arc(i, j)
            total += W_TIME * a.energy + W_SAFETY * a.risk + W_LOAD * a.loss
        return total

    t0 = time.perf_counter()
    model = sigpa_llm_train(
        train, route_metric=metric_J,
        generations=args.llm_generations, offspring=3,
        problem_description=(
            "Layered Greek airway graph (fix, flight level). Arc measures: "
            "risk = danger-area exposure, duration = potential-shaped fuel "
            "cost, turn, loss = ATC sector congestion. Deployment objective "
            "J = 1*cost + 60*risk + 20*congestion (safety-dominant)."),
        rng=random.Random(7), eval_seed=0,
        k=3, max_iterations=400, max_no_improve=40,
    )
    train_s = time.perf_counter() - t0
    used = (f"{model.llm_proposals} LLM candidates" if model.used_llm
            else "mutation fallback, no LLM credentials")
    print(f"  done in {train_s:.0f}s ({used})")
    print(f"  evolved evaluator: {model.evaluator}\n")

    occupancy = {name: defaultdict(int) for name in METHODS}
    day_J = dict.fromkeys(METHODS, 0.0)
    day_parts = {name: [0.0, 0.0, 0.0] for name in METHODS}
    runtimes = dict.fromkeys(METHODS, 0.0)

    print(f"{'flight':9s}{'route':13s}" +
          "".join(f"{n:>10s}" for n in
                  ["CPLEX", "Dijkstra", "BellFord", "A*", "SIGPA", "SIGPALLM"]))

    for spec in flights:
        dep, arr, ackey = spec["from"], spec["to"], spec["aircraft"]
        ac = AIRCRAFT_CATALOG[ackey]
        sub, meas = sub_by_ac[ackey], measures_by_ac[ackey]
        s_node, t_node = ("APT", dep), ("APT", arr)

        def astar_h(a, b):
            da, db = sub.nodes[a], sub.nodes[b]
            d = haversine_km(da["lat"], da["lon"], db["lat"], db["lon"])
            return W_TIME * d / ((ac.cruise_tas_kt + 150) * 1.852 / 60)

        results = {}
        for name in METHODS:
            wts = combined_weight(meas, occupancy[name], capacities)
            t0 = time.perf_counter()
            if name == "CPLEX":
                path = cplex_exact(sub, wts, s_node, t_node)
            elif name == "Dijkstra":
                path = nx.dijkstra_path(sub, s_node, t_node,
                                        weight=lambda u, v, d: wts[(u, v)])
            elif name == "Bellman-Ford":
                path = nx.bellman_ford_path(sub, s_node, t_node,
                                            weight=lambda u, v, d: wts[(u, v)])
            elif name == "A*":
                path = nx.astar_path(sub, s_node, t_node, heuristic=astar_h,
                                     weight=lambda u, v, d: wts[(u, v)])
            else:
                plan = plan_flight_sigpa(
                    g, dep, arr, ac, winds, airspace, sectors,
                    occupancy=dict(occupancy[name]), capacities=capacities,
                    arc_evaluator=model.evaluator if name == "SIGPA-LLM" else None)
                path = plan.nodes
            runtimes[name] += time.perf_counter() - t0

            J = path_objective(path, wts)
            c, s_, l = path_breakdown(path, meas, occupancy[name], capacities)
            results[name] = J
            day_J[name] += J
            for i, v in enumerate((c, s_, l)):
                day_parts[name][i] += v
            for sec in sectors_of_path(path, meas):
                occupancy[name][sec] += 1

        print(f"{spec['callsign']:9s}{dep}->{arr} {ackey[:4]:5s}" +
              "".join(f"{results[n]:10.1f}" for n in METHODS))

    ex = day_J["CPLEX"]
    print(f"\n{'day total J':22s}" +
          "".join(f"{day_J[n]:10.1f}" for n in METHODS))
    print(f"{'gap vs CPLEX':22s}" +
          "".join(f"{100*(day_J[n]-ex)/ex:+9.1f}%" for n in METHODS))
    print("\nday totals by objective term:")
    print(f"  {'method':13s}{'time cost':>11s}{'danger exp.':>12s}{'congestion':>12s}")
    for n in METHODS:
        c, s_, l = day_parts[n]
        print(f"  {n:13s}{c:11.1f}{s_:12.2f}{l:12.2f}")
    print("\nmean planning runtime per flight:")
    for n in METHODS:
        print(f"  {n:13s}{1000*runtimes[n]/len(flights):9.1f} ms")
    print(f"\nSIGPA-LLM offline design: one-off {train_s:.0f}s "
          "(not part of the per-flight runtime)")


if __name__ == "__main__":
    main()
