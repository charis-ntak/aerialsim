"""En-route sectors: entry counting and demand-capacity flow regulation.

The regulation loop is a simplified CASA (Computer Assisted Slot Allocation):
while any sector's entries-per-window exceed its declared capacity, the
latest-entering flight of the worst-overloaded window receives a ground
delay, and the demand picture is recomputed.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from .geo import point_in_polygon


@dataclass
class Sector:
    id: str
    name: str
    fl_min: int
    fl_max: int
    polygon: list[tuple[float, float]]

    def contains(self, lat: float, lon: float, alt_ft: float) -> bool:
        if not (self.fl_min * 100 <= alt_ft <= self.fl_max * 100):
            return False
        return point_in_polygon(lat, lon, self.polygon)


def load_sectors(path: str | Path) -> list[Sector]:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    return [Sector(s["id"], s["name"], s["fl_min"], s["fl_max"],
                   [tuple(v) for v in s["polygon"]])
            for s in raw["sectors"]]


def sector_entries(flights, sectors: list[Sector]) -> list[tuple[str, float, str]]:
    """(sector_id, entry_time_min, callsign) for every sector entry event."""
    events = []
    for f in flights:
        if not f.trajectory:
            continue
        current: str | None = None
        for (t, lat, lon, alt) in f.trajectory:
            inside = next((s.id for s in sectors if s.contains(lat, lon, alt)), None)
            if inside != current:
                if inside is not None:
                    events.append((inside, t, f.callsign))
                current = inside
    return events


def demand_per_window(events, window_min: float) -> dict[tuple[str, int], list]:
    """Group entry events into (sector, window index) buckets."""
    buckets: dict[tuple[str, int], list] = {}
    for sector_id, t, callsign in events:
        key = (sector_id, int(t // window_min))
        buckets.setdefault(key, []).append((t, callsign))
    return buckets


def regulate_flow(flights, sectors: list[Sector], capacities: dict[str, int],
                  graph, window_min: float = 20.0, delay_step: float = 5.0,
                  max_iter: int = 60):
    """Apply ground delays until no (sector, window) exceeds capacity.

    Returns (delays: {callsign: total_delay_min}, log: [str]).
    Mutates flight dep_time_min and trajectories.
    """
    from .traffic import build_trajectory

    delays: dict[str, float] = {}
    log: list[str] = []
    by_cs = {f.callsign: f for f in flights}

    for _ in range(max_iter):
        buckets = demand_per_window(sector_entries(flights, sectors), window_min)
        overloaded = [(key, entrants) for key, entrants in buckets.items()
                      if key[0] in capacities and len(entrants) > capacities[key[0]]]
        if not overloaded:
            return delays, log
        (sector_id, win), entrants = max(
            overloaded, key=lambda kv: len(kv[1]) - capacities[kv[0][0]])
        entrants.sort()
        _, victim_cs = entrants[-1]           # latest entrant gets the delay
        f = by_cs[victim_cs]
        f.dep_time_min += delay_step
        build_trajectory(f, graph)
        delays[victim_cs] = delays.get(victim_cs, 0.0) + delay_step
        log.append(f"{victim_cs}: +{delay_step:.0f} min ground delay "
                   f"(sector {sector_id} window T+{win * window_min:.0f}-"
                   f"{(win + 1) * window_min:.0f} demand {len(entrants)} > "
                   f"capacity {capacities[sector_id]})")
    log.append("regulation iteration limit reached — demand still exceeds capacity")
    return delays, log


def peak_loads(flights, sectors: list[Sector], window_min: float = 20.0
               ) -> dict[str, int]:
    """Max entries in any window, per sector (for reporting)."""
    buckets = demand_per_window(sector_entries(flights, sectors), window_min)
    peaks: dict[str, int] = {}
    for (sector_id, _), entrants in buckets.items():
        peaks[sector_id] = max(peaks.get(sector_id, 0), len(entrants))
    return peaks
