"""Fetch real historical Greek domestic flights and build replay scenarios.

Data source: the EUROCONTROL Performance Review Commission's Open
Performance Data Initiative (OPDI, https://www.opdi.aero/), built on
OpenSky Network ADS-B data.  Monthly flight lists are public parquet
files, no authentication required:

    https://www.eurocontrol.int/performance/data/download/OPDI/v002/
        flight_list/flight_list_YYYYMM.parquet

Usage:
    python tools/fetch_history.py 202506                 # download + filter
    python tools/fetch_history.py 202506 --scenario 2025-06-10
        # additionally write scenarios/atm_real_YYYYMMDD.yaml replaying
        # every network-airport domestic flight of that day

Requires: pip install pyarrow pandas
"""
from __future__ import annotations

import argparse
import sys
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "history"
OPDI_URL = ("https://www.eurocontrol.int/performance/data/download/"
            "OPDI/v002/flight_list/flight_list_{month}.parquet")

NETWORK_AIRPORTS = ["LGAV", "LGTS", "LGIR", "LGRP", "LGKR", "LGSR"]

# OPDI typecode -> aerialsim aircraft catalog key (nearest performance class)
TYPE_MAP = {
    "A320": "a320", "A20N": "a320", "A21N": "a320", "A319": "a320",
    "B738": "b738", "B38M": "b738",
    "AT76": "atr72", "AT75": "atr72", "AT72": "atr72",
    "E120": "atr72",                       # EMB-120 turboprop: nearest class
    "E190": "e190", "E195": "e190", "E90": "e190",
}


def download(month: str) -> Path:
    DATA.mkdir(parents=True, exist_ok=True)
    dest = DATA / f"flight_list_{month}.parquet"
    if not dest.exists():
        url = OPDI_URL.format(month=month)
        print(f"downloading {url} ...")
        urllib.request.urlretrieve(url, dest)
    print(f"flight list: {dest} ({dest.stat().st_size/1e6:.0f} MB)")
    return dest


def filter_domestic(parquet: Path, month: str):
    import pyarrow.parquet as pq

    df = pq.read_table(parquet).to_pandas()
    dom = df[df["adep"].astype(str).str.startswith("LG")
             & df["ades"].astype(str).str.startswith("LG")]
    ours = dom[dom["adep"].isin(NETWORK_AIRPORTS)
               & dom["ades"].isin(NETWORK_AIRPORTS)
               & (dom["adep"] != dom["ades"])].copy()
    out = DATA / f"greek_domestic_{month}.csv"
    ours.to_csv(out, index=False)
    print(f"Greek domestic LG->LG flights: {len(dom)}")
    print(f"between network airports {NETWORK_AIRPORTS}: {len(ours)} -> {out}")
    print(ours.groupby(["adep", "ades"]).size().sort_values(ascending=False)
          .to_string())
    return ours


def write_scenario(flights, day: str) -> Path:
    sample = flights[flights["dof"].astype(str).str.startswith(day)] \
        .sort_values("first_seen")
    if sample.empty:
        sys.exit(f"no flights on {day}")

    t0 = sample["first_seen"].min()
    lines = [
        f"# Real traffic replay: every Greek domestic flight between the",
        f"# network airports on {day}, from the EUROCONTROL OPDI flight list",
        f"# (OpenSky ADS-B). Callsigns, aircraft types and departure times",
        f"# are the historical values; departure minutes are offsets from",
        f"# the first departure ({t0:%H:%M} UTC).",
        f"name: Real domestic traffic {day} (OPDI/OpenSky)",
        "description: >",
        f"  Replay of the {len(sample)} historical domestic flights between the six",
        "  network airports, planned with live winds aloft and separation",
        "  monitoring.",
        "flights:",
    ]
    for _, f in sample.iterrows():
        ac = TYPE_MAP.get(str(f["typecode"]), "a320")
        dep_min = round((f["first_seen"] - t0).total_seconds() / 60)
        callsign = str(f["flt_id"]).strip()
        lines.append(
            f"  - {{ callsign: {callsign:<8s}, aircraft: {ac:<6s}, "
            f"from: {f['adep']}, to: {f['ades']}, dep_time_min: {dep_min} }}"
        )
    lines += [
        "active_zone_ids: []",
        "hour_offset: 0",
        "resolve_conflicts: true",
        "sector_capacities: { ATH-E: 3, ATH-W: 3, MAK: 3, KRI: 3 }",
        "metar_stations: [LGAV, LGTS, LGIR, LGRP]",
        "",
    ]
    out = ROOT / "scenarios" / f"atm_real_{day.replace('-', '')}.yaml"
    out.write_text("\n".join(lines), encoding="utf-8")
    print(f"scenario written: {out}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("month", help="YYYYMM, e.g. 202506")
    ap.add_argument("--scenario", metavar="YYYY-MM-DD",
                    help="also write a replay scenario for this day")
    args = ap.parse_args()

    parquet = download(args.month)
    flights = filter_domestic(parquet, args.month)
    if args.scenario:
        write_scenario(flights, args.scenario)


if __name__ == "__main__":
    main()
