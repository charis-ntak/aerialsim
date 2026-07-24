"""Multi-flight traffic simulation: 4D trajectories, separation monitoring,
and simple tactical conflict resolution by flight-level change."""
from __future__ import annotations

from dataclasses import dataclass, field

from .atm_planner import FlightPlan
from .geo import haversine_km

SEP_LATERAL_NM = 5.0
SEP_VERTICAL_FT = 1000.0
NM_PER_KM = 0.539957


@dataclass
class Flight:
    callsign: str
    aircraft_id: str
    dep: str
    arr: str
    dep_time_min: float
    plan: FlightPlan | None = None
    trajectory: list[tuple[float, float, float, float]] = field(default_factory=list)
    # (t_min absolute, lat, lon, alt_ft)


def build_trajectory(f: Flight, g, dt_min: float = 0.5) -> None:
    """Sample the planned route into 4D points every dt_min minutes."""
    pts = []
    t = f.dep_time_min
    for leg in f.plan.legs:
        u, v = leg["from"], leg["to"]
        du, dv = g.nodes[u], g.nodes[v]
        dur = max(leg["time_min"], 1e-6)
        n = max(1, int(dur / dt_min))
        for i in range(n):
            frac = i / n
            pts.append((
                t + frac * dur,
                du["lat"] + frac * (dv["lat"] - du["lat"]),
                du["lon"] + frac * (dv["lon"] - du["lon"]),
                du["alt_ft"] + frac * (dv["alt_ft"] - du["alt_ft"]),
            ))
        t += dur
    last = f.plan.nodes[-1]
    d = g.nodes[last]
    pts.append((t, d["lat"], d["lon"], d["alt_ft"]))
    f.trajectory = pts


@dataclass
class Conflict:
    flight_a: str
    flight_b: str
    t_start_min: float
    t_end_min: float
    min_lateral_nm: float
    min_vertical_ft: float
    lat: float
    lon: float
    alt_ft: float


def _positions_at(traj: list, t: float) -> tuple[float, float, float] | None:
    if not traj or t < traj[0][0] or t > traj[-1][0]:
        return None
    import bisect
    times = [p[0] for p in traj]
    i = bisect.bisect_right(times, t) - 1
    if i >= len(traj) - 1:
        p = traj[-1]
        return p[1], p[2], p[3]
    a, b = traj[i], traj[i + 1]
    f = (t - a[0]) / max(b[0] - a[0], 1e-9)
    return (a[1] + f * (b[1] - a[1]),
            a[2] + f * (b[2] - a[2]),
            a[3] + f * (b[3] - a[3]))


def detect_conflicts(flights: list[Flight], dt_min: float = 0.5,
                     ignore_below_ft: float = 5000.0) -> list[Conflict]:
    """Pairwise separation check on the sampled timeline.

    Encounters below ignore_below_ft are skipped (terminal phases share the
    airport by construction; tower/approach separation is out of scope).
    """
    conflicts = []
    for i in range(len(flights)):
        for j in range(i + 1, len(flights)):
            fa, fb = flights[i], flights[j]
            if not (fa.trajectory and fb.trajectory):
                continue
            t0 = max(fa.trajectory[0][0], fb.trajectory[0][0])
            t1 = min(fa.trajectory[-1][0], fb.trajectory[-1][0])
            active = None
            t = t0
            while t <= t1:
                pa = _positions_at(fa.trajectory, t)
                pb = _positions_at(fb.trajectory, t)
                if pa and pb and min(pa[2], pb[2]) >= ignore_below_ft:
                    lat_nm = haversine_km(pa[0], pa[1], pb[0], pb[1]) * NM_PER_KM
                    vert_ft = abs(pa[2] - pb[2])
                    if lat_nm < SEP_LATERAL_NM and vert_ft < SEP_VERTICAL_FT:
                        if active is None:
                            active = Conflict(fa.callsign, fb.callsign, t, t,
                                              lat_nm, vert_ft,
                                              (pa[0] + pb[0]) / 2,
                                              (pa[1] + pb[1]) / 2,
                                              (pa[2] + pb[2]) / 2)
                        active.t_end_min = t
                        if lat_nm < active.min_lateral_nm:
                            active.min_lateral_nm = lat_nm
                            active.lat = (pa[0] + pb[0]) / 2
                            active.lon = (pa[1] + pb[1]) / 2
                            active.alt_ft = (pa[2] + pb[2]) / 2
                        active.min_vertical_ft = min(active.min_vertical_ft, vert_ft)
                    elif active is not None:
                        conflicts.append(active)
                        active = None
                t += dt_min
            if active is not None:
                conflicts.append(active)
    return conflicts


def resolve_conflicts(flights: list[Flight], g, replan, max_rounds: int = 6,
                      dt_min: float = 0.5) -> tuple[list[Conflict], list[str]]:
    """Iteratively replan the later-departing flight of the worst conflict,
    excluding the flight level at which the conflict occurred.

    replan(flight, forbidden_fls) -> FlightPlan
    Returns (remaining_conflicts, action_log).
    """
    actions = []
    forbidden: dict[str, set[int]] = {f.callsign: set() for f in flights}
    for _ in range(max_rounds):
        conflicts = detect_conflicts(flights, dt_min)
        if not conflicts:
            return [], actions
        c = min(conflicts, key=lambda x: x.min_lateral_nm)
        by_cs = {f.callsign: f for f in flights}
        victim = max((by_cs[c.flight_a], by_cs[c.flight_b]),
                     key=lambda f: f.dep_time_min)
        bad_fl = int(round(c.alt_ft / 1000) * 10)
        forbidden[victim.callsign].update({bad_fl - 10, bad_fl, bad_fl + 10})
        new_plan = replan(victim, forbidden[victim.callsign])
        if not new_plan.ok:
            actions.append(f"{victim.callsign}: no alternative level available "
                           f"({new_plan.reason}) — conflict UNRESOLVED")
            return conflicts, actions
        victim.plan = new_plan
        build_trajectory(victim, g, dt_min)
        actions.append(
            f"{victim.callsign}: level change off FL{bad_fl} "
            f"(now cruising FL{'/'.join(map(str, new_plan.cruise_fls))}) to clear "
            f"conflict with {c.flight_a if victim.callsign == c.flight_b else c.flight_b}")
    return detect_conflicts(flights, dt_min), actions
