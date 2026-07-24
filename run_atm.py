"""CLI entry point for civil ATM scenarios.

Usage:
    python run_atm.py scenarios/atm_morning_wave.yaml
    python run_atm.py scenarios/atm_danger_active.yaml --hour-offset 6
"""
from __future__ import annotations

import argparse
from pathlib import Path

from aerialsim.scenario_atm import ATMScenario, format_atm_report, run_atm_scenario
from aerialsim.viz_atm import render_atm


def main() -> None:
    ap = argparse.ArgumentParser(description="Run a civil ATM scenario over Greek airspace")
    ap.add_argument("scenario", help="path to an ATM scenario YAML")
    ap.add_argument("--hour-offset", type=int, default=None,
                    help="plan N hours ahead using the forecast (overrides YAML)")
    ap.add_argument("--out", default="output", help="output directory for the 3D HTML view")
    args = ap.parse_args()

    sc = ATMScenario.from_yaml(args.scenario)
    if args.hour_offset is not None:
        sc.hour_offset = args.hour_offset

    out = run_atm_scenario(sc)
    print(format_atm_report(out))

    html = render_atm(out, Path(args.out) / (Path(args.scenario).stem + "_3d.html"))
    print(f"\n3D view written to {html}")


if __name__ == "__main__":
    main()
