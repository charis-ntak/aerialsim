"""3D visualization of the ATM scenario: route lattice, 4D trajectories, conflicts."""
from __future__ import annotations

from pathlib import Path

import plotly.graph_objects as go

PALETTE = ["#e31a5f", "#1f77b4", "#2ca02c", "#ff7f0e", "#9467bd",
           "#17becf", "#8c564b", "#bcbd22"]


def add_network_traces(fig: go.Figure, network) -> None:
    """Grey route skeleton + fix columns + airport markers (shared by views)."""
    wp = network.waypoints

    # Route skeleton at a single reference level (grey), fix stacks as columns.
    for route in network.routes:
        pts = route["waypoints"]
        fig.add_trace(go.Scatter3d(
            x=[wp[p]["lon"] for p in pts], y=[wp[p]["lat"] for p in pts],
            z=[0] * len(pts), mode="lines",
            line=dict(color="#bbbbbb", width=2), opacity=0.7,
            name="ATS routes (ground track)", legendgroup="routes",
            showlegend=route is network.routes[0], hovertext=route["name"],
            hoverinfo="text",
        ))
    for name, w in wp.items():
        fig.add_trace(go.Scatter3d(
            x=[w["lon"]] * 2, y=[w["lat"]] * 2, z=[0, 41000],
            mode="lines", line=dict(color="#dddddd", width=1), opacity=0.5,
            legendgroup="routes", showlegend=False,
            hovertext=f"{name} FL200-FL410", hoverinfo="text",
        ))
        fig.add_trace(go.Scatter3d(
            x=[w["lon"]], y=[w["lat"]], z=[0], mode="markers+text",
            marker=dict(size=3, color="#666"), text=[name],
            textposition="bottom center", textfont=dict(size=9, color="#666"),
            legendgroup="routes", showlegend=False, hoverinfo="skip",
        ))

    for icao, apt in network.airports.items():
        fig.add_trace(go.Scatter3d(
            x=[apt["lon"]], y=[apt["lat"]], z=[0], mode="markers+text",
            marker=dict(size=6, color="#000", symbol="square"),
            text=[icao], textposition="top center",
            showlegend=False, hovertext=apt["name"], hoverinfo="text",
        ))


def render_atm(out: dict, html_path: str | Path) -> Path:
    fig = go.Figure()
    add_network_traces(fig, out["network"])

    # Flight trajectories.
    for i, f in enumerate(out["flights"]):
        if not f.trajectory:
            continue
        color = PALETTE[i % len(PALETTE)]
        lats = [p[1] for p in f.trajectory]
        lons = [p[2] for p in f.trajectory]
        alts = [p[3] for p in f.trajectory]
        times = [p[0] for p in f.trajectory]
        fig.add_trace(go.Scatter3d(
            x=lons, y=lats, z=alts, mode="lines",
            line=dict(color=color, width=5),
            name=f"{f.callsign} {f.dep}->{f.arr} "
                 f"FL{'/'.join(map(str, f.plan.cruise_fls))}",
            hovertext=[f"{f.callsign} T+{t:.0f} min, {a/100:.0f}00 ft"
                       for t, a in zip(times, alts)],
            hoverinfo="text",
        ))

    # Conflicts: markers where separation was lost (before resolution).
    for c in out["conflicts_before"]:
        resolved = c not in out["conflicts_after"]
        fig.add_trace(go.Scatter3d(
            x=[c.lon], y=[c.lat], z=[c.alt_ft], mode="markers+text",
            marker=dict(size=9, color="#ffd400" if resolved else "#ff0000",
                        symbol="x"),
            text=[f"{'resolved' if resolved else 'CONFLICT'}"],
            textposition="top center",
            name=f"{c.flight_a} x {c.flight_b} ({c.min_lateral_nm:.1f} NM)",
            hovertext=f"{c.flight_a} x {c.flight_b} T+{c.t_start_min:.0f} min "
                      f"min sep {c.min_lateral_nm:.1f} NM / {c.min_vertical_ft:.0f} ft",
            hoverinfo="text",
        ))

    sc = out["scenario"]
    n_cb, n_ca = len(out["conflicts_before"]), len(out["conflicts_after"])
    fig.update_layout(
        title=f"{sc.name} — {n_cb} conflict(s) detected, {n_cb - n_ca} resolved",
        scene=dict(
            xaxis_title="Longitude", yaxis_title="Latitude",
            zaxis_title="Altitude (ft)",
            aspectmode="manual", aspectratio=dict(x=1.3, y=1, z=0.55),
        ),
        legend=dict(itemsizing="constant", font=dict(size=10)),
        margin=dict(l=0, r=0, t=40, b=0),
    )
    html_path = Path(html_path)
    html_path.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(html_path, include_plotlyjs=True)
    return html_path
