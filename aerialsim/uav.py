"""UAV performance models and weather flyability checks."""
from __future__ import annotations

from dataclasses import dataclass

from .weather import WeatherSample


@dataclass
class UAVModel:
    name: str
    kind: str                  # "multirotor" | "fixed_wing_vtol"
    cruise_speed_ms: float
    max_wind_ms: float         # sustained wind limit at cruise altitude
    max_gust_ms: float
    max_precip_mmh: float
    min_visibility_m: float
    endurance_min: float
    cruise_alt_agl_m: float    # nominal cruise altitude (EU open category cap: 120 m)

    def check_flyable(self, w: WeatherSample, alt_m: float | None = None) -> list[str]:
        """Return a list of limit violations at this weather sample (empty = OK)."""
        alt = alt_m if alt_m is not None else self.cruise_alt_agl_m
        wind_speed, _ = w.wind_at(alt)
        problems = []
        if wind_speed > self.max_wind_ms:
            problems.append(
                f"wind {wind_speed:.1f} m/s at {alt:.0f} m exceeds limit {self.max_wind_ms:.0f} m/s")
        if w.wind_gusts_10m > self.max_gust_ms:
            problems.append(
                f"gusts {w.wind_gusts_10m:.1f} m/s exceed limit {self.max_gust_ms:.0f} m/s")
        if w.precip_mmh > self.max_precip_mmh:
            problems.append(
                f"precipitation {w.precip_mmh:.1f} mm/h exceeds limit {self.max_precip_mmh:.1f} mm/h")
        if w.visibility_m < self.min_visibility_m:
            problems.append(
                f"visibility {w.visibility_m:.0f} m below minimum {self.min_visibility_m:.0f} m")
        return problems


# Representative platforms (specs in the ballpark of common commercial UAS).
UAV_CATALOG: dict[str, UAVModel] = {
    "multirotor_delivery": UAVModel(
        name="Delivery multirotor (5 kg payload class)",
        kind="multirotor",
        cruise_speed_ms=16.0,
        max_wind_ms=10.0,
        max_gust_ms=14.0,
        max_precip_mmh=1.0,
        min_visibility_m=3000.0,
        endurance_min=40.0,
        cruise_alt_agl_m=100.0,
    ),
    "vtol_longrange": UAVModel(
        name="Fixed-wing VTOL (long-range survey/cargo)",
        kind="fixed_wing_vtol",
        cruise_speed_ms=25.0,
        max_wind_ms=14.0,
        max_gust_ms=18.0,
        max_precip_mmh=2.0,
        min_visibility_m=2000.0,
        endurance_min=120.0,
        cruise_alt_agl_m=120.0,
    ),
    "inspection_quad": UAVModel(
        name="Inspection quadcopter (camera/LiDAR)",
        kind="multirotor",
        cruise_speed_ms=12.0,
        max_wind_ms=12.0,
        max_gust_ms=15.0,
        max_precip_mmh=0.5,
        min_visibility_m=5000.0,
        endurance_min=35.0,
        cruise_alt_agl_m=80.0,
    ),
}
