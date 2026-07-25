"""Replay a scenario with SIGPA multi-objective planning vs the A* baseline.

Plans every flight twice — with the wind-optimal single-cost A*
(`aerialsim.atm_planner.plan_flight`) and with the SIGPA swarm
metaheuristic (`aerialsim.sigpa_planner.plan_flight_sigpa`) — and compares
time, distance, convective exposure, route smoothness and sector load.
SIGPA plans the wave sequentially: each flight sees the sector occupancy
of the previously planned flights as its congestion measure, so the swarm
load-balances the Athinai FIR sectors.

Usage:
    python run_sigpa.py scenarios/atm_real_20250610.yaml
    python run_sigpa.py scenarios/atm_morning_wave.yaml --hour-offset 3
"""
from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path

from aerialsim.aircraft import AIRCRAFT_CATALOG
from aerialsim.airspace import AirspaceIndex
from aerialsim.atm_planner import plan_flight, _leg_time_min
from aerialsim.routegraph import RouteNetwork
from aerialsim.scenario_atm import ATMScenario, DATA_DIR
from aerialsim.sectors import load_sectors, peak_loads
from aerialsim.sigpa_planner import plan_flight_sigpa, sectors_entered
from aerialsim.traffic import Flight, build_trajectory, detect_conflicts
from aerialsim.viz_atm import render_atm
from aerialsim.weather import WindsAloftProvider


def _turns(plan) -> int:
    """Count track changes > 30 degrees along the en-route portion."""
    tracks = [t for t in (leg.get("track") for leg in plan.legs) if t is not None]
    n = 0
    for a, b in zip(tracks, tracks[1:]):
        d = abs(b - a) % 360
        if min(d, 360 - d) > 30:
            n += 1
    return n


def _cost(plan, g, ac, winds) -> float:
    """Fuel-weighted planning cost (the scalar A* optimizes)."""
    return sum(_leg_time_min(g, u, v, g.edges[u, v], ac, winds)[0]
               for u, v in zip(plan.nodes, plan.nodes[1:]))


def _convective_legs(plan) -> int:
    return sum(1 for leg in plan.legs if leg.get("convective"))


def main() -> None:
    ap = argparse.ArgumentParser(description="SIGPA vs A* over a scenario")
    ap.add_argument("scenario")
    ap.add_argument("--hour-offset", type=int, default=None)
    ap.add_argument("--out", default="output")
    args = ap.parse_args()

    sc = ATMScenario.from_yaml(args.scenario)
    if args.hour_offset is not None:
        sc.hour_offset = args.hour_offset

    network = RouteNetwork(DATA_DIR / "network_gr.json")
    g = network.build_graph()
    airspace = AirspaceIndex.from_json(DATA_DIR / "airspace_gr.json")
    winds = WindsAloftProvider(hour_offset=sc.hour_offset)
    sectors = load_sectors(DATA_DIR / "sectors_gr.json")
    capacities = dict(sc.sector_capacities or {})

    rows = []
    occupancy: dict = defaultdict(int)
    astar_flights, sigpa_flights = [], []

    for spec in sc.flights:
        ac = AIRCRAFT_CATALOG[spec["aircraft"]]
        base = plan_flight(g, spec["from"], spec["to"], ac, winds, airspace,
                           sc.active_zone_ids)
        swarm = plan_flight_sigpa(
            g, spec["from"], spec["to"], ac, winds, airspace, sectors,
            occupancy=dict(occupancy), capacities=capacities)
        if swarm.ok:
            for sid in sectors_entered(g, swarm, sectors):
                occupancy[sid] += 1
        rows.append((spec, base, swarm))

        for plans, bucket in ((base, astar_flights), (swarm, sigpa_flights)):
            f = Flight(callsign=spec["callsign"], aircraft_id=spec["aircraft"],
                       dep=spec["from"], arr=spec["to"],
                       dep_time_min=float(spec.get("dep_time_min", 0)),
                       plan=plans)
            if plans.ok:
                build_trajectory(f, g)
            bucket.append(f)

    print(f"=== SIGPA vs A* : {sc.name} ===")
    if winds.used_fallback:
        print("!! Live winds unavailable - synthetic weather fallback in use.")
    print(f"{'flight':9s}{'route':12s}"
          f"{'A* cost':>9s}{'time':>7s}{'km':>6s}{'cnv':>4s}{'trn':>4s}   "
          f"{'SIGPA cost':>10s}{'time':>7s}{'km':>6s}{'cnv':>4s}{'trn':>4s}")
    ok_pairs = []
    for spec, base, swarm in rows:
        route = f"{spec['from']}->{spec['to']}"
        if not (base.ok and swarm.ok):
            print(f"{spec['callsign']:9s}{route:12s}  planning failed: "
                  f"A*={base.reason or 'ok'} SIGPA={swarm.reason or 'ok'}")
            continue
        ac = AIRCRAFT_CATALOG[spec["aircraft"]]
        bc, sc_ = _cost(base, g, ac, winds), _cost(swarm, g, ac, winds)
        ok_pairs.append((base, swarm, bc, sc_))
        print(f"{spec['callsign']:9s}{route:12s}"
              f"{bc:9.1f}{base.time_min:7.1f}{base.distance_km:6.0f}"
              f"{_convective_legs(base):4d}{_turns(base):4d}   "
              f"{sc_:10.1f}{swarm.time_min:7.1f}{swarm.distance_km:6.0f}"
              f"{_convective_legs(swarm):4d}{_turns(swarm):4d}")

    if ok_pairs:
        bcost = sum(p[2] for p in ok_pairs)
        scost = sum(p[3] for p in ok_pairs)
        bt = sum(p[0].time_min for p in ok_pairs)
        st = sum(p[1].time_min for p in ok_pairs)
        gap = (scost - bcost) / bcost * 100
        print(f"\n{'totals':9s}{'':12s}{bcost:9.1f}{bt:7.1f}"
              f"{'':14s}{scost:10.1f}{st:7.1f}   cost gap {gap:+.1f}%")

    for name, bucket in (("A*", astar_flights), ("SIGPA", sigpa_flights)):
        planned = [f for f in bucket if f.plan and f.plan.ok]
        loads = peak_loads(planned, sectors, sc.regulation_window_min)
        conflicts = detect_conflicts(planned)
        loads_txt = ", ".join(f"{sid}:{peak}" for sid, peak in sorted(loads.items()))
        print(f"{name:6s} peak sector entries/{sc.regulation_window_min:.0f}min: "
              f"{loads_txt or 'none'}   conflicts: {len(conflicts)}")

    out = {
        "scenario": sc, "graph": g, "network": network, "airspace": airspace,
        "flights": sigpa_flights, "conflicts_before": [], "conflicts_after": [],
        "actions": [], "sectors": sectors, "sector_loads": {},
        "delays": {}, "regulation_log": [], "metars": [], "winds": winds,
    }
    html = render_atm(out, Path(args.out) / (Path(args.scenario).stem + "_sigpa_3d.html"))
    print(f"\n3D view of the SIGPA plans written to {html}")


if __name__ == "__main__":
    main()
