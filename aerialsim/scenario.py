"""Scenario definition (YAML) and end-to-end execution."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from .airspace import AirspaceIndex
from .graph3d import build_civil_graph, build_uav_grid
from .planner import PlanResult, plan_route
from .uav import UAV_CATALOG, UAVModel
from .weather import OpenMeteoProvider, fetch_metars

DATA_DIR = Path(__file__).resolve().parent.parent / "data"


@dataclass
class Scenario:
    name: str
    description: str
    uav: UAVModel
    start: dict          # {name, lat, lon}
    goal: dict
    bbox: tuple[float, float, float, float]
    alt_layers_m: list[float]
    grid_spacing_km: float
    allowed_zone_ids: set[str]
    hour_offset: int
    metar_stations: list[str]

    @classmethod
    def from_yaml(cls, path: str | Path) -> "Scenario":
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        uav_id = raw["uav"]
        if uav_id not in UAV_CATALOG:
            raise ValueError(f"unknown UAV {uav_id!r}; options: {sorted(UAV_CATALOG)}")
        if "bbox" in raw:
            bbox = tuple(raw["bbox"])
        else:
            lats = [raw["start"]["lat"], raw["goal"]["lat"]]
            lons = [raw["start"]["lon"], raw["goal"]["lon"]]
            m = raw.get("bbox_margin_deg", 0.15)
            bbox = (min(lats) - m, max(lats) + m, min(lons) - m, max(lons) + m)
        return cls(
            name=raw["name"],
            description=raw.get("description", ""),
            uav=UAV_CATALOG[uav_id],
            start=raw["start"], goal=raw["goal"], bbox=bbox,
            alt_layers_m=raw.get("alt_layers_m", [60, 100, 120]),
            grid_spacing_km=raw.get("grid_spacing_km", 1.5),
            allowed_zone_ids=set(raw.get("allowed_zone_ids", [])),
            hour_offset=raw.get("hour_offset", 0),
            metar_stations=raw.get("metar_stations", []),
        )


def run_scenario(sc: Scenario) -> dict:
    airspace = AirspaceIndex.from_json(DATA_DIR / "airspace_gr.json")
    weather = OpenMeteoProvider(hour_offset=sc.hour_offset)

    grid = build_uav_grid(sc.bbox, sc.grid_spacing_km, sc.alt_layers_m,
                          airspace, sc.allowed_zone_ids)
    civil = build_civil_graph(DATA_DIR / "routes_gr.json")

    result = plan_route(grid,
                        (sc.start["lat"], sc.start["lon"]),
                        (sc.goal["lat"], sc.goal["lon"]),
                        sc.uav, weather)

    metars = fetch_metars(sc.metar_stations) if sc.metar_stations else []

    return {
        "scenario": sc,
        "airspace": airspace,
        "grid": grid,
        "civil": civil,
        "result": result,
        "metars": metars,
        "weather": weather,
    }


def format_report(out: dict) -> str:
    sc: Scenario = out["scenario"]
    r: PlanResult = out["result"]
    lines = [
        f"=== Scenario: {sc.name} ===",
        sc.description,
        f"UAV: {sc.uav.name} ({sc.uav.kind}), cruise {sc.uav.cruise_speed_ms:.0f} m/s, "
        f"wind limit {sc.uav.max_wind_ms:.0f} m/s, endurance {sc.uav.endurance_min:.0f} min",
        f"Grid: {out['grid'].number_of_nodes()} nodes / {out['grid'].number_of_edges()} edges, "
        f"{out['grid'].graph['blocked_nodes']} lattice points blocked by airspace",
        "",
    ]
    if r.weather_synthetic:
        lines.append("!! Live weather unavailable — SYNTHETIC fallback weather in use.")
    if r.feasible:
        lines += [
            "MISSION FEASIBLE",
            f"  route distance : {r.distance_km:.1f} km",
            f"  flight time    : {r.flight_time_min:.1f} min "
            f"(endurance {sc.uav.endurance_min:.0f} min)",
            f"  max wind enroute: {r.max_wind_ms:.1f} m/s "
            f"(limit {sc.uav.max_wind_ms:.0f}), max gusts {r.max_gust_ms:.1f} m/s",
        ]
    else:
        lines += ["MISSION NOT FEASIBLE", f"  reason: {r.reason}"]
    if r.weather_warnings:
        lines.append("  weather limits hit somewhere in the area:")
        lines += [f"    - {w}" for w in r.weather_warnings[:8]]
    for m in out["metars"]:
        raw = m.get("rawOb") or m.get("raw_text") or ""
        if raw:
            lines.append(f"  METAR {raw}")
    return "\n".join(lines)
