"""How to use aerialsim from your own code — minimal worked examples.

Run from the project root:  python examples/api_demo.py
(or add E:/aerial_simulation_env to sys.path / install it in your project)
"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # project root

from aerialsim.aircraft import AIRCRAFT_CATALOG, AircraftModel
from aerialsim.airspace import AirspaceIndex
from aerialsim.atm_planner import plan_flight
from aerialsim.routegraph import RouteNetwork
from aerialsim.traffic import Flight, build_trajectory, detect_conflicts
from aerialsim.weather import WindsAloftProvider

DATA = Path(__file__).resolve().parent.parent / "data"

# ---------------------------------------------------------------- 1. Setup
# Build the 3D route graph and load supporting data ONCE, then reuse.
network = RouteNetwork(DATA / "network_gr.json")
graph = network.build_graph()                      # networkx.DiGraph
airspace = AirspaceIndex.from_json(DATA / "airspace_gr.json")
winds = WindsAloftProvider(hour_offset=0)          # live; caches per 0.5 deg cell

print(f"graph: {graph.number_of_nodes()} nodes, {graph.number_of_edges()} edges")

# ------------------------------------------------- 2. Plan a single flight
a320 = AIRCRAFT_CATALOG["a320"]
plan = plan_flight(graph, "LGAV", "LGTS", a320, winds, airspace)
print(f"\nLGAV->LGTS: ok={plan.ok}  {plan.distance_km:.0f} km  "
      f"{plan.time_min:.1f} min  cruise FL{plan.cruise_fls}")
for leg in plan.legs:                              # per-leg detail
    if leg["kind"] == "enroute":
        print(f"  {leg['from']} -> {leg['to']}  GS {leg['gs_kt']:.0f} kt  "
              f"headwind {leg['headwind_kt']:+.0f} kt")

# ------------------------------------- 3. Same flight, constraints applied
# Danger area active + FL360-380 unavailable (e.g. blocked by other traffic):
plan2 = plan_flight(graph, "LGAV", "LGIR", a320, winds, airspace,
                    active_zone_ids={"GR-D-AEGEAN-HIGH"},
                    forbidden_fls={360, 370, 380})
fixes = list(dict.fromkeys(n[0] for n in plan2.nodes if n[0] != "APT"))
print(f"\nLGAV->LGIR around active danger area: {plan2.distance_km:.0f} km "
      f"via {'-'.join(fixes)}")

# ------------------------------------------ 4. Traffic + conflict checking
flights = []
for cs, ac_id, dep, arr, t0 in [("AAA100", "a320", "LGAV", "LGTS", 0),
                                ("BBB200", "e190", "LGIR", "LGTS", 5)]:
    f = Flight(cs, ac_id, dep, arr, t0,
               plan=plan_flight(graph, dep, arr, AIRCRAFT_CATALOG[ac_id],
                                winds, airspace))
    build_trajectory(f, graph)                     # -> f.trajectory: 4D points
    flights.append(f)

conflicts = detect_conflicts(flights)
print(f"\nconflicts: {len(conflicts)}")
t, lat, lon, alt = flights[0].trajectory[20]       # position 10 min after dep
print(f"{flights[0].callsign} at T+{t:.1f} min: {lat:.3f}N {lon:.3f}E {alt:.0f} ft")

# ----------------------------------------------- 5. Add your own aircraft
AIRCRAFT_CATALOG["myjet"] = AircraftModel(
    name="My business jet", icao_type="C68A",
    cruise_tas_kt=430, climb_fpm=3000, descent_fpm=2500,
    max_fl=450, opt_fl=410)
# ...then use it like any other: plan_flight(graph, "LGAV", "LGRP",
#                                            AIRCRAFT_CATALOG["myjet"], ...)

# --------------------------------------------- 6. Query the graph directly
# It is a normal networkx DiGraph — use any graph algorithm you like.
node = ("ATV", 370)
nbrs = list(graph.successors(node))[:4]
print(f"\n{node} attrs: {graph.nodes[node]}")
print(f"sample successors: {nbrs}")
