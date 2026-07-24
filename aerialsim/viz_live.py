"""3D view of live traffic over the route network."""
from __future__ import annotations

import math
from pathlib import Path

import plotly.graph_objects as go

from .live import LiveAircraft, Proximity
from .viz_atm import add_network_traces


def render_live(aircraft: list[LiveAircraft], proximities: list[Proximity],
                network, html_path: str | Path,
                heading_vector_min: float = 2.0) -> Path:
    fig = go.Figure()
    add_network_traces(fig, network)

    lats = [a.lat for a in aircraft]
    lons = [a.lon for a in aircraft]
    alts = [a.alt_ft for a in aircraft]
    fig.add_trace(go.Scatter3d(
        x=lons, y=lats, z=alts, mode="markers",
        marker=dict(size=3.5, color=alts, colorscale="Viridis",
                    cmin=0, cmax=41000,
                    colorbar=dict(title="ft", thickness=12, len=0.6)),
        name=f"live aircraft ({len(aircraft)})",
        hovertext=[f"{a.callsign}  {a.alt_ft:.0f} ft  {a.gs_kt:.0f} kt  "
                   f"trk {a.track_deg:.0f}  {a.vrate_fpm:+.0f} fpm"
                   for a in aircraft],
        hoverinfo="text",
    ))

    # Short heading vectors (~N minutes of flight at current GS).
    vx, vy, vz = [], [], []
    for a in aircraft:
        d_km = a.gs_kt * 1.852 / 60.0 * heading_vector_min
        dlat = d_km / 111.0 * math.cos(math.radians(a.track_deg))
        dlon = (d_km / (111.0 * max(0.2, math.cos(math.radians(a.lat))))
                * math.sin(math.radians(a.track_deg)))
        vx += [a.lon, a.lon + dlon, None]
        vy += [a.lat, a.lat + dlat, None]
        vz += [a.alt_ft, a.alt_ft + a.vrate_fpm * heading_vector_min, None]
    fig.add_trace(go.Scatter3d(
        x=vx, y=vy, z=vz, mode="lines",
        line=dict(color="#888", width=2), opacity=0.6,
        name=f"heading ({heading_vector_min:.0f} min)", hoverinfo="skip",
    ))

    for p in proximities:
        fig.add_trace(go.Scatter3d(
            x=[p.a.lon, p.b.lon], y=[p.a.lat, p.b.lat],
            z=[p.a.alt_ft, p.b.alt_ft], mode="lines+markers",
            line=dict(color="#ff0000", width=6),
            marker=dict(size=5, color="#ff0000"),
            name=f"{p.a.callsign} x {p.b.callsign} "
                 f"({p.lateral_nm:.1f} NM / {p.vertical_ft:.0f} ft)",
        ))

    fig.update_layout(
        title=f"Live traffic over Greece — {len(aircraft)} aircraft, "
              f"{len(proximities)} proximity pair(s)",
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
