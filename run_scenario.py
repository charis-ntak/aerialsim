"""CLI entry point.

Usage:
    python run_scenario.py scenarios/aegina_medical.yaml
    python run_scenario.py scenarios/evia_inspection.yaml --hour-offset 6
"""
from __future__ import annotations

import argparse
from pathlib import Path

from aerialsim.scenario import Scenario, format_report, run_scenario
from aerialsim.viz import render


def main() -> None:
    ap = argparse.ArgumentParser(description="Run a UAV scenario over Greek airspace")
    ap.add_argument("scenario", help="path to a scenario YAML")
    ap.add_argument("--hour-offset", type=int, default=None,
                    help="plan N hours ahead using the forecast (overrides YAML)")
    ap.add_argument("--out", default="output", help="output directory for the 3D HTML view")
    args = ap.parse_args()

    sc = Scenario.from_yaml(args.scenario)
    if args.hour_offset is not None:
        sc.hour_offset = args.hour_offset

    out = run_scenario(sc)
    print(format_report(out))

    html = render(out, Path(args.out) / (Path(args.scenario).stem + "_3d.html"))
    print(f"\n3D view written to {html}")


if __name__ == "__main__":
    main()
