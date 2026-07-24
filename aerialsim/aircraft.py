"""Civil aircraft performance models (simplified point-mass profiles)."""
from __future__ import annotations

from dataclasses import dataclass

KT_PER_MS = 1.943844


@dataclass
class AircraftModel:
    name: str
    icao_type: str
    cruise_tas_kt: float      # true airspeed in cruise
    climb_fpm: float
    descent_fpm: float
    max_fl: int               # service ceiling as flight level (e.g. 390)
    opt_fl: int = 370         # fuel-optimal cruise level
    min_cruise_fl: int = 200

    def fuel_factor(self, fl: int) -> float:
        """Relative fuel burn vs the optimum level (~3% per 1000 ft off-optimum).

        Multiplies leg time in the planning cost so the optimizer trades the
        extra climb time against cheaper cruise, like a real FMS cost index.
        """
        return 1.0 + 0.045 * abs(fl - self.opt_fl) / 10.0


AIRCRAFT_CATALOG: dict[str, AircraftModel] = {
    "a320": AircraftModel(
        name="Airbus A320 class", icao_type="A320",
        cruise_tas_kt=450, climb_fpm=2000, descent_fpm=2200,
        max_fl=390, opt_fl=370),
    "b738": AircraftModel(
        name="Boeing 737-800 class", icao_type="B738",
        cruise_tas_kt=455, climb_fpm=2100, descent_fpm=2300,
        max_fl=410, opt_fl=380),
    "atr72": AircraftModel(
        name="ATR 72 turboprop", icao_type="AT76",
        cruise_tas_kt=270, climb_fpm=1350, descent_fpm=1500,
        max_fl=250, opt_fl=230, min_cruise_fl=200),
    "e190": AircraftModel(
        name="Embraer E190 class", icao_type="E190",
        cruise_tas_kt=440, climb_fpm=2200, descent_fpm=2300,
        max_fl=410, opt_fl=380),
}
