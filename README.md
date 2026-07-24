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
