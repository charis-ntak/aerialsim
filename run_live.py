"""Snapshot of real traffic over Greece (OpenSky) with proximity screening.

Usage:
    python run_live.py
    python run_live.py --min-fl 200      # only show cruise traffic
"""
from __future__ import annotations

import argparse
from pathlib import Path

from aerialsim.live import fetch_live_traffic, snapshot_proximities
from aerialsim.routegraph import RouteNetwork
from aerialsim.scenario import DATA_DIR
from aerialsim.viz_live import render_live


def main() -> None:
    ap = argparse.ArgumentParser(description="Live traffic over Greek airspace")
    ap.add_argument("--min-fl", type=int, default=0,
                    help="only include aircraft at/above this flight level")
    ap.add_argument("--out", default="output", help="output directory")
    args = ap.parse_args()

    aircraft = fetch_live_traffic()
    if args.min_fl:
        aircraft = [a for a in aircraft if a.alt_ft >= args.min_fl * 100]
    prox = snapshot_proximities(aircraft)

    print(f"=== Live traffic over Greece (OpenSky) ===")
    print(f"{len(aircraft)} airborne aircraft"
          + (f" at/above FL{args.min_fl}" if args.min_fl else ""))

    bands = {"below FL100": 0, "FL100-FL245": 0, "FL245-FL410": 0, "above FL410": 0}
    for a in aircraft:
        if a.alt_ft < 10000:
            bands["below FL100"] += 1
        elif a.alt_ft < 24500:
            bands["FL100-FL245"] += 1
        elif a.alt_ft <= 41000:
            bands["FL245-FL410"] += 1
        else:
            bands["above FL410"] += 1
    for k, v in bands.items():
        print(f"  {k:12s}: {v}")

    print(f"\nProximity screen (<5 NM & <1000 ft, above FL050): {len(prox)} pair(s)")
    for p in prox[:10]:
        print(f"  {p.a.callsign:8s} x {p.b.callsign:8s}  "
              f"{p.lateral_nm:.1f} NM / {p.vertical_ft:.0f} ft  "
              f"at {p.a.alt_ft:.0f}/{p.b.alt_ft:.0f} ft")
    if prox:
        print("  (snapshot screen — climbing/descending traffic can legitimately"
              " pass through these thresholds)")

    network = RouteNetwork(DATA_DIR / "network_gr.json")
    html = render_live(aircraft, prox, network,
                       Path(args.out) / "live_traffic_3d.html")
    print(f"\n3D view written to {html}")


if __name__ == "__main__":
    main()
