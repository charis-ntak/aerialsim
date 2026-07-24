"""ATM scenario definition (YAML) and end-to-end execution."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from .aircraft import AIRCRAFT_CATALOG
from .airspace import AirspaceIndex
from .atm_planner import plan_flight
from .routegraph import RouteNetwork
from .traffic import Flight, build_trajectory, detect_conflicts, resolve_conflicts
from .weather import WindsAloftProvider, fetch_metars

DATA_DIR = Path(__file__).resolve().parent.parent / "data"


@dataclass
class ATMScenario:
    name: str
    description: str
    flights: list[dict]
    active_zone_ids: set[str]
    hour_offset: int
    metar_stations: list[str]
    resolve: bool

    @classmethod
    def from_yaml(cls, path: str | Path) -> "ATMScenario":
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        for fl in raw["flights"]:
            if fl["aircraft"] not in AIRCRAFT_CATALOG:
                raise ValueError(f"unknown aircraft {fl['aircraft']!r}; "
                                 f"options: {sorted(AIRCRAFT_CATALOG)}")
        return cls(
            name=raw["name"],
            description=raw.get("description", ""),
            flights=raw["flights"],
            active_zone_ids=set(raw.get("active_zone_ids", [])),
            hour_offset=raw.get("hour_offset", 0),
            metar_stations=raw.get("metar_stations", []),
            resolve=raw.get("resolve_conflicts", True),
        )


def run_atm_scenario(sc: ATMScenario) -> dict:
    network = RouteNetwork(DATA_DIR / "network_gr.json")
    g = network.build_graph()
    airspace = AirspaceIndex.from_json(DATA_DIR / "airspace_gr.json")
    winds = WindsAloftProvider(hour_offset=sc.hour_offset)

    flights: list[Flight] = []
    for spec in sc.flights:
        ac = AIRCRAFT_CATALOG[spec["aircraft"]]
        plan = plan_flight(g, spec["from"], spec["to"], ac, winds, airspace,
                           sc.active_zone_ids)
        f = Flight(callsign=spec["callsign"], aircraft_id=spec["aircraft"],
                   dep=spec["from"], arr=spec["to"],
                   dep_time_min=float(spec.get("dep_time_min", 0)), plan=plan)
        if plan.ok:
            build_trajectory(f, g)
        flights.append(f)

    planned = [f for f in flights if f.plan.ok]
    conflicts_before = detect_conflicts(planned)

    actions: list[str] = []
    conflicts_after = conflicts_before
    if sc.resolve and conflicts_before:
        def replan(f: Flight, forbidden_fls: set[int]):
            return plan_flight(g, f.dep, f.arr, AIRCRAFT_CATALOG[f.aircraft_id],
                               winds, airspace, sc.active_zone_ids, forbidden_fls)
        conflicts_after, actions = resolve_conflicts(planned, g, replan)

    metars = fetch_metars(sc.metar_stations) if sc.metar_stations else []
    return {
        "scenario": sc, "graph": g, "network": network, "airspace": airspace,
        "flights": flights, "conflicts_before": conflicts_before,
        "conflicts_after": conflicts_after, "actions": actions,
        "metars": metars, "winds": winds,
    }


def format_atm_report(out: dict) -> str:
    sc: ATMScenario = out["scenario"]
    lines = [f"=== ATM scenario: {sc.name} ===", sc.description]
    if sc.active_zone_ids:
        lines.append(f"Active hazard zones: {', '.join(sorted(sc.active_zone_ids))}")
    if any(f.plan.weather_synthetic for f in out["flights"] if f.plan):
        lines.append("!! Live winds aloft unavailable — SYNTHETIC fallback in use.")
    lines.append("")

    for f in out["flights"]:
        p = f.plan
        if not p.ok:
            lines.append(f"{f.callsign:8s} {f.dep}->{f.arr}  PLANNING FAILED: {p.reason}")
            continue
        fixes = [n[0] for n in p.nodes if n[0] != "APT"]
        seen, seq = set(), []
        for x in fixes:
            if x not in seen:
                seq.append(x)
                seen.add(x)
        avg_hw = [leg.get("headwind_kt", 0.0) for leg in p.legs if "headwind_kt" in leg]
        hw = sum(avg_hw) / len(avg_hw) if avg_hw else 0.0
        lines.append(
            f"{f.callsign:8s} {f.dep}->{f.arr}  dep T+{f.dep_time_min:>4.0f} min  "
            f"{p.distance_km:6.0f} km  {p.time_min:5.1f} min  "
            f"FL{'/'.join(map(str, p.cruise_fls)):9s} via {'-'.join(seq)}  "
            f"avg wind comp {'+' if hw >= 0 else ''}{hw:.0f} kt "
            f"({'head' if hw >= 0 else 'tail'})")
        for w in p.convective_warnings:
            lines.append(f"          ! {w}")

    lines.append("")
    cb, ca = out["conflicts_before"], out["conflicts_after"]
    lines.append(f"Separation check (5 NM / 1000 ft, above FL050): "
                 f"{len(cb)} conflict(s) detected")
    for c in cb:
        lines.append(
            f"  {c.flight_a} x {c.flight_b}: T+{c.t_start_min:.0f}-{c.t_end_min:.0f} min, "
            f"min sep {c.min_lateral_nm:.1f} NM / {c.min_vertical_ft:.0f} ft "
            f"near {c.lat:.2f}N {c.lon:.2f}E {c.alt_ft/100:.0f}00 ft")
    for a in out["actions"]:
        lines.append(f"  ATC: {a}")
    if cb and not ca:
        lines.append("  All conflicts resolved.")
    elif ca and out["actions"]:
        lines.append(f"  {len(ca)} conflict(s) remain UNRESOLVED.")

    for m in out["metars"]:
        raw = m.get("rawOb") or m.get("raw_text") or ""
        if raw:
            lines.append(f"METAR {raw}")
    return "\n".join(lines)
