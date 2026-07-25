# aerialsim — Greek airspace 3D-graph simulation environment

Simulates civil air-traffic-management scenarios over Greek airspace (Athinai
FIR / Hellas UIR) with **live weather**. The airspace is modeled as a 3D
directed graph: nodes are `(fix, flight level)` pairs from FL200 to FL410,
edges are ATS route segments filtered by the ICAO semicircular cruising-level
rule, plus climb/descent edges and airport departure/arrival links.

## Quick start

```bash
pip install -r requirements.txt
python run_atm.py scenarios/atm_morning_wave.yaml
python run_atm.py scenarios/atm_danger_active.yaml
python run_atm.py scenarios/atm_regulated.yaml
python run_live.py            # real traffic over Greece right now (OpenSky)
```

Each run prints a flight-plan + separation report and writes an interactive
3D view to `output/<scenario>_3d.html`.

## What a scenario run does

1. **Builds the 3D route graph** from [data/network_gr.json](data/network_gr.json):
   airports, VOR/waypoints, route segments. Directed edges per FL enforce the
   semicircular rule (tracks 000–179° → odd levels, 180–359° → even), so
   opposite-direction flows are vertically separated by construction.
2. **Fetches live winds aloft** from Open-Meteo pressure-level data
   (700–200 hPa mapped to flight levels) plus CAPE as a convective-risk flag,
   and live METARs from aviationweather.gov. A flagged synthetic fallback
   keeps runs working offline.
3. **Plans each flight** with A*: cost = leg time × a fuel factor that grows
   away from the aircraft's optimum level (a crude FMS cost index). Level
   choice therefore trades climb time, fuel altitude, and head/tailwind —
   e.g. a short sector with a strong low-level tailwind may stay at FL210
   while long sectors cruise FL360–380. Active danger areas block segments
   and force reroutes.
4. **Regulates demand** (strategic, optional): sector entry counts per time
   window are checked against declared capacities
   (`sector_capacities: { ATH-E: 3 }` in the YAML); a CASA-style loop assigns
   ground delays to the latest entrants of overloaded windows until demand
   fits. Sectors live in [data/sectors_gr.json](data/sectors_gr.json).
5. **Simulates the traffic** as 4D trajectories (30-second sampling) and
   monitors pairwise separation (5 NM lateral / 1000 ft vertical above FL050).
6. **Resolves conflicts** tactically: the later-departing flight of the worst
   conflict is replanned with the conflicting levels excluded, iterating
   until separation is restored (or no level remains).

## Live traffic

`python run_live.py` fetches a snapshot of real aircraft over Greece from the
OpenSky Network (anonymous, rate-limited to ~100 requests/day), prints an
altitude-band breakdown with a 5 NM / 1000 ft proximity screen, and renders
the traffic over the route lattice in 3D
([aerialsim/live.py](aerialsim/live.py), [aerialsim/viz_live.py](aerialsim/viz_live.py)).

## Historical traffic — real domestic flights

`python tools/fetch_history.py 202506 --scenario 2025-06-10` downloads the
public [EUROCONTROL OPDI](https://www.opdi.aero/) monthly flight list
(OpenSky ADS-B based, no authentication), filters it to Greek domestic
flights between the six network airports, and writes a replay scenario with
the historical callsigns, aircraft types and departure times.
June 2025 contains 1,272 Greek domestic flights (236 between the network
airports; filtered list committed at
[data/history/greek_domestic_202506.csv](data/history/greek_domestic_202506.csv)).
The busiest day, 2025-06-10 with 12 flights (Aegean, SKY Express, Marathon),
is committed as
[scenarios/atm_real_20250610.yaml](scenarios/atm_real_20250610.yaml).

## SIGPA multi-objective planning

`python run_sigpa.py scenarios/atm_real_20250610.yaml` replays a scenario
with the swarm intelligence graph-based pathfinding algorithm
([SIGPA](https://github.com/charis-ntak/sigpa), Ntakolia & Iakovidis, COR
133 (2021) 105358) side-by-side with the A* baseline. Where A* minimizes a
single scalar (leg time × fuel factor), SIGPA evaluates each candidate
segment on four normalized measures — convective risk, fuel-weighted leg
cost, track change, and the congestion of the ATC sector the segment
crosses — and plans the departure wave **sequentially**, so each flight
sees the sector occupancy of the flights planned before it
([aerialsim/sigpa_planner.py](aerialsim/sigpa_planner.py)).

Two adapter-level design choices matter on a sparse airway lattice: the
fitness distance term is the **network distance-to-go** (reverse Dijkstra)
rather than the straight line, and the greedy duration measure is the
**potential-shaped reduced cost** `cost + h(next) − h(current)` with
`h` = fuel-weighted time-to-go, so each myopic step sees the downstream
fuel implication of the flight level it commits to.

On the real 2025-06-10 traffic day, SIGPA plans land within ~2% of the A*
cost optimum with zero conflicts and equal-or-better peak sector loads.
Requires `pip install git+https://github.com/charis-ntak/sigpa.git`.

## Exact-vs-heuristic benchmark

`python run_benchmark.py scenarios/atm_real_20250610.yaml` solves the real
historical day with six methods on the identical graph and identical
objective, decided for this case study as the safety-dominant additive sum

    J = 1.0 x fuel-weighted time + 60 x danger-area exposure (P/R/D zones,
        convective risk folded in when CAPE is present) + 20 x ATC-sector
        congestion

Each method plans the day sequentially in real departure order and evolves
its own sector-occupancy stream. Methods: **CPLEX** (exact 0-1 flow MILP
via docplex), **Dijkstra**, **Bellman-Ford**, **A***
(admissible time heuristic), **SIGPA**, and **SIGPA-LLM** (SIGPA with an
offline-designed evaluation criterion — LLM-proposed candidates when
`ANTHROPIC_API_KEY` is available, mutation fallback otherwise; see
`sigpa.sigpa_llm`).

Result on 2025-06-10 (12 real flights): the four exact/optimal methods
agree on every flight (day J = 1184.8, cross-validating the model);
SIGPA and SIGPA-LLM land at **+0.8%** with ~17 ms per flight vs ~90 ms
for CPLEX. Requires `pip install docplex cplex` for the exact column.

## Layout

| Piece | File |
|---|---|
| Route network data (airports, fixes, routes) | [data/network_gr.json](data/network_gr.json) |
| Airspace zones (CTR/TMA/P/R/D volumes) | [data/airspace_gr.json](data/airspace_gr.json), [aerialsim/airspace.py](aerialsim/airspace.py) |
| Layered route graph + semicircular rule | [aerialsim/routegraph.py](aerialsim/routegraph.py) |
| Aircraft performance models | [aerialsim/aircraft.py](aerialsim/aircraft.py) |
| Winds aloft / CAPE / METAR | [aerialsim/weather.py](aerialsim/weather.py) |
| Wind-optimal flight planning (A*) | [aerialsim/atm_planner.py](aerialsim/atm_planner.py) |
| 4D trajectories, separation, resolution | [aerialsim/traffic.py](aerialsim/traffic.py) |
| Sectors + flow regulation | [aerialsim/sectors.py](aerialsim/sectors.py), [data/sectors_gr.json](data/sectors_gr.json) |
| Live traffic (OpenSky) | [aerialsim/live.py](aerialsim/live.py), [run_live.py](run_live.py) |
| Scenario loading + report | [aerialsim/scenario_atm.py](aerialsim/scenario_atm.py) |
| 3D visualization | [aerialsim/viz_atm.py](aerialsim/viz_atm.py) |

A secondary UAV module (very-low-level grid graph, `run_scenario.py`,
`scenarios/aegina_medical.yaml` / `evia_inspection.yaml`) from an earlier
iteration is kept — it shares the airspace and weather infrastructure.

## Scenario YAML

```yaml
name: Morning wave over the central Aegean
flights:
  - { callsign: OAL301, aircraft: a320, from: LGAV, to: LGTS, dep_time_min: 0 }
  - { callsign: RYR184, aircraft: b738, from: LGRP, to: LGAV, dep_time_min: 0 }
active_zone_ids: []          # e.g. [GR-D-AEGEAN-HIGH] to activate a danger area
hour_offset: 0               # plan N hours ahead on the forecast
resolve_conflicts: true
metar_stations: [LGAV, LGTS]
```

Aircraft: `a320`, `b738`, `e190`, `atr72` (the ATR's FL250 ceiling puts it in
different levels than the jets). Airports: LGAV, LGTS, LGIR, LGRP, LGKR, LGSR.

## Data accuracy — important

Airport coordinates are real; waypoint coordinates are approximate; the route
designators and fix sequences are **representative, not the published ones**,
and airspace zone geometry is simplified. For accurate work replace the data
files with:

- **Greek eAIP** (HASP) — ENR 3 (ATS routes), ENR 4 (navaids/fixes),
  ENR 2/5 (TMA, CTR, P/R/D areas)
- **Eurocontrol NM B2B / RAD** — operationally accurate route availability
- **OpenAIP** (openaip.net) — free machine-readable exports

This is a simulation environment, not an operational planning tool. Notable
simplifications: true (not magnetic) track for the semicircular rule, one
pressure level per FL band, no SID/STAR geometry, no sector capacities,
terminal-area separation out of scope.

## Ideas for next steps

- Continuous traffic generation from a schedule (or replay real OpenSky
  trajectories through the simulation) rather than hand-listed flights
- Convective weather cells as moving polygons that block segments dynamically
- Fuel burn per aircraft type in kg (BADA-like) instead of the relative factor
- Import real route/fix data from the eAIP to replace the representative network
