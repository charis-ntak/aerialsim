"""Interactive 3D visualization (plotly -> standalone HTML).

Axes: x = longitude, y = latitude, z = altitude in metres (log-like split:
UAV layers are metres AGL, civil routes at FL are metres too, so the true
scale would crush the UAV layer — a z-break factor compresses the civil
altitudes for readability; the hover text always shows true values).
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import plotly.graph_objects as go

from .airspace import AirspaceIndex
from .planner import PlanResult
from .scenario import Scenario

CIVIL_Z_COMPRESSION = 12.0   # civil altitudes divided by this for display


def _circle_pts(center: tuple[float, float], radius_km: float, n: int = 48):
    lat0, lon0 = center
    dlat = radius_km / 111.0
    dlon = radius_km / (111.0 * max(0.2, math.cos(math.radians(lat0))))
    ang = np.linspace(0, 2 * np.pi, n)
    return lat0 + dlat * np.sin(ang), lon0 + dlon * np.cos(ang)


def _zone_mesh(zone, color: str, opacity: float):
    g = zone.geometry
    if g["kind"] == "circle":
        lats, lons = _circle_pts(tuple(g["center"]), g["radius_km"])
    else:
        v = g["vertices"] + [g["vertices"][0]]
        lats = np.array([p[0] for p in v])
        lons = np.array([p[1] for p in v])
    z_lo = zone.lower_ft / 3.28084
    z_hi = min(zone.upper_ft, 8000) / 3.28084 / CIVIL_Z_COMPRESSION \
        if zone.upper_ft > 3000 else zone.upper_ft / 3.28084
    z_hi = max(z_hi, z_lo + 30)
    traces = []
    for z in (z_lo, z_hi):
        traces.append(go.Scatter3d(
            x=lons, y=lats, z=[z] * len(lats), mode="lines",
            line=dict(color=color, width=2), opacity=opacity,
            name=f"{zone.id} ({zone.type})",
            legendgroup=zone.id, showlegend=(z == z_lo),
            hovertext=f"{zone.name}<br>{zone.lower_ft:.0f}-{zone.upper_ft:.0f} ft",
            hoverinfo="text",
        ))
    # A few vertical ribs so the volume reads as 3D.
    for idx in range(0, len(lats), max(1, len(lats) // 8)):
        traces.append(go.Scatter3d(
            x=[lons[idx]] * 2, y=[lats[idx]] * 2, z=[z_lo, z_hi],
            mode="lines", line=dict(color=color, width=1), opacity=opacity * 0.6,
            legendgroup=zone.id, showlegend=False, hoverinfo="skip",
        ))
    return traces


ZONE_COLORS = {"P": "#d62728", "R": "#d62728", "D": "#ff7f0e",
               "CTR": "#1f77b4", "TMA": "#9467bd"}


def render(out: dict, html_path: str | Path) -> Path:
    sc: Scenario = out["scenario"]
    r: PlanResult = out["result"]
    airspace: AirspaceIndex = out["airspace"]
    fig = go.Figure()

    lat_min, lat_max, lon_min, lon_max = sc.bbox
    pad = 0.6
    for zone in airspace.zones:
        c = zone.geometry.get("center")
        if c and not (lat_min - pad < c[0] < lat_max + pad
                      and lon_min - pad < c[1] < lon_max + pad):
            continue
        if not c:
            vs = zone.geometry["vertices"]
            if not any(lat_min - pad < v[0] < lat_max + pad
                       and lon_min - pad < v[1] < lon_max + pad for v in vs):
                continue
        fig.add_traces(_zone_mesh(zone, ZONE_COLORS.get(zone.type, "#888"), 0.5))

    # Civil route layer (compressed altitude).
    civil = out["civil"]
    for a, b, d in civil.edges(data=True):
        na, nb = civil.nodes[a], civil.nodes[b]
        fig.add_trace(go.Scatter3d(
            x=[na["lon"], nb["lon"]], y=[na["lat"], nb["lat"]],
            z=[na["alt_m"] / CIVIL_Z_COMPRESSION] * 2,
            mode="lines+markers", marker=dict(size=3, color="#2ca02c"),
            line=dict(color="#2ca02c", width=3, dash="dash"),
            name=f"ATS {d['route']}", legendgroup=d["route"],
            showlegend=(a, b) == next(iter(civil.edges)),
            hovertext=f"{d['route']}: {na['fix']}-{nb['fix']} "
                      f"FL{a[1]} ({na['alt_m']:.0f} m true alt, display compressed)",
            hoverinfo="text",
        ))

    # UAV planned path.
    if r.path_coords:
        lats = [p[0] for p in r.path_coords]
        lons = [p[1] for p in r.path_coords]
        alts = [p[2] for p in r.path_coords]
        fig.add_trace(go.Scatter3d(
            x=lons, y=lats, z=alts, mode="lines+markers",
            line=dict(color="#e31a5f", width=6), marker=dict(size=2),
            name=f"UAV route ({r.distance_km:.1f} km, {r.flight_time_min:.0f} min)",
        ))

    for pt, label, color in ((sc.start, "START: " + sc.start.get("name", ""), "#000"),
                             (sc.goal, "GOAL: " + sc.goal.get("name", ""), "#000")):
        fig.add_trace(go.Scatter3d(
            x=[pt["lon"]], y=[pt["lat"]], z=[0],
            mode="markers+text", text=[label], textposition="top center",
            marker=dict(size=6, color=color, symbol="diamond"),
            showlegend=False,
        ))

    status = "FEASIBLE" if r.feasible else f"NOT FEASIBLE - {r.reason}"
    fig.update_layout(
        title=f"{sc.name} - {status}",
        scene=dict(
            xaxis_title="Longitude", yaxis_title="Latitude",
            zaxis_title=f"Altitude m (civil layer / {CIVIL_Z_COMPRESSION:.0f})",
            aspectmode="manual",
            aspectratio=dict(x=1.4, y=1, z=0.5),
        ),
        legend=dict(itemsizing="constant"),
        margin=dict(l=0, r=0, t=40, b=0),
    )
    html_path = Path(html_path)
    html_path.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(html_path, include_plotlyjs=True)
    return html_path
