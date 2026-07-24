"""Weather providers for the simulation.

Primary source: Open-Meteo forecast API (free, no API key) — hourly wind at
10 m and 120 m, gusts, precipitation, visibility, temperature. Samples are
cached per ~0.25 deg grid cell so a whole route costs only a handful of
HTTP calls. If the network is unavailable, a synthetic fallback keeps the
simulation runnable (clearly flagged in results).

Secondary: live METARs from aviationweather.gov for airport ground truth.
"""
from __future__ import annotations

from dataclasses import dataclass

import requests

OPEN_METEO_URL = "https://api.open-meteo.com/v1/forecast"
METAR_URL = "https://aviationweather.gov/api/data/metar"

HOURLY_FIELDS = [
    "wind_speed_10m", "wind_direction_10m", "wind_gusts_10m",
    "wind_speed_120m", "wind_direction_120m",
    "precipitation", "visibility", "temperature_2m",
]


@dataclass
class WeatherSample:
    lat: float
    lon: float
    time_iso: str
    wind_speed_10m: float      # m/s
    wind_dir_10m: float        # deg (from)
    wind_gusts_10m: float      # m/s
    wind_speed_120m: float     # m/s
    wind_dir_120m: float       # deg (from)
    precip_mmh: float
    visibility_m: float
    temp_c: float
    synthetic: bool = False

    def wind_at(self, alt_m: float) -> tuple[float, float]:
        """(speed m/s, direction deg FROM) interpolated between 10 m and 120 m."""
        if alt_m <= 10:
            return self.wind_speed_10m, self.wind_dir_10m
        if alt_m >= 120:
            return self.wind_speed_120m, self.wind_dir_120m
        f = (alt_m - 10) / 110.0
        speed = self.wind_speed_10m + f * (self.wind_speed_120m - self.wind_speed_10m)
        d1, d2 = self.wind_dir_10m, self.wind_dir_120m
        delta = ((d2 - d1 + 180) % 360) - 180
        return speed, (d1 + f * delta) % 360


class OpenMeteoProvider:
    """Fetches hourly forecasts, cached per 0.25-degree cell."""

    def __init__(self, hour_offset: int = 0, timeout_s: float = 15.0):
        self.hour_offset = hour_offset   # 0 = current hour, 6 = six hours ahead, ...
        self.timeout_s = timeout_s
        self._cache: dict[tuple[float, float], WeatherSample | None] = {}
        self.used_fallback = False

    @staticmethod
    def _cell(lat: float, lon: float) -> tuple[float, float]:
        return round(lat * 4) / 4, round(lon * 4) / 4

    def sample(self, lat: float, lon: float) -> WeatherSample:
        cell = self._cell(lat, lon)
        if cell not in self._cache:
            self._cache[cell] = self._fetch(*cell)
        s = self._cache[cell]
        if s is None:
            self.used_fallback = True
            return _synthetic_sample(lat, lon)
        return s

    def _fetch(self, lat: float, lon: float) -> WeatherSample | None:
        try:
            r = requests.get(OPEN_METEO_URL, params={
                "latitude": lat, "longitude": lon,
                "hourly": ",".join(HOURLY_FIELDS),
                "wind_speed_unit": "ms",
                "forecast_days": 2,
                "timezone": "UTC",
            }, timeout=self.timeout_s)
            r.raise_for_status()
            h = r.json()["hourly"]
            # Index of "now" in the hourly series: Open-Meteo returns data
            # starting at 00:00 today (UTC), one entry per hour.
            from datetime import datetime, timezone
            idx = min(datetime.now(timezone.utc).hour + self.hour_offset,
                      len(h["time"]) - 1)
            g = lambda k, d=0.0: h[k][idx] if h[k][idx] is not None else d
            return WeatherSample(
                lat=lat, lon=lon, time_iso=h["time"][idx],
                wind_speed_10m=g("wind_speed_10m"),
                wind_dir_10m=g("wind_direction_10m"),
                wind_gusts_10m=g("wind_gusts_10m"),
                wind_speed_120m=g("wind_speed_120m", g("wind_speed_10m") * 1.3),
                wind_dir_120m=g("wind_direction_120m", g("wind_direction_10m")),
                precip_mmh=g("precipitation"),
                visibility_m=g("visibility", 20000.0),
                temp_c=g("temperature_2m", 15.0),
            )
        except (requests.RequestException, KeyError, ValueError, IndexError):
            return None


def _synthetic_sample(lat: float, lon: float) -> WeatherSample:
    """Deterministic plausible Aegean weather (moderate Meltemi-style NNW wind)."""
    import math
    base = 6.0 + 2.0 * math.sin(lat * 3.1) * math.cos(lon * 2.7)
    return WeatherSample(
        lat=lat, lon=lon, time_iso="synthetic",
        wind_speed_10m=base, wind_dir_10m=340.0, wind_gusts_10m=base * 1.5,
        wind_speed_120m=base * 1.35, wind_dir_120m=345.0,
        precip_mmh=0.0, visibility_m=20000.0, temp_c=24.0,
        synthetic=True,
    )


# --- Winds aloft (pressure-level data) for the civil / ATM layer -----------

# Approximate ISA mapping flight level -> Open-Meteo pressure level.
_FL_TO_HPA = [(140, 700), (210, 500), (270, 400), (320, 300), (360, 250), (999, 200)]
ALOFT_LEVELS_HPA = sorted({hpa for _, hpa in _FL_TO_HPA})


def fl_to_hpa(fl: int) -> int:
    for max_fl, hpa in _FL_TO_HPA:
        if fl <= max_fl:
            return hpa
    return 200


@dataclass
class AloftSample:
    time_iso: str
    winds: dict[int, tuple[float, float]]   # hPa -> (speed m/s, direction deg FROM)
    cape_jkg: float
    synthetic: bool = False

    def wind_at_fl(self, fl: int) -> tuple[float, float]:
        return self.winds[fl_to_hpa(fl)]


class WindsAloftProvider:
    """Open-Meteo pressure-level winds + CAPE, cached per 0.5-degree cell."""

    def __init__(self, hour_offset: int = 0, timeout_s: float = 15.0):
        self.hour_offset = hour_offset
        self.timeout_s = timeout_s
        self._cache: dict[tuple[float, float], AloftSample | None] = {}
        self.used_fallback = False

    @staticmethod
    def _cell(lat: float, lon: float) -> tuple[float, float]:
        return round(lat * 2) / 2, round(lon * 2) / 2

    def sample(self, lat: float, lon: float) -> AloftSample:
        cell = self._cell(lat, lon)
        if cell not in self._cache:
            self._cache[cell] = self._fetch(*cell)
        s = self._cache[cell]
        if s is None:
            self.used_fallback = True
            return _synthetic_aloft(lat, lon)
        return s

    def _fetch(self, lat: float, lon: float) -> AloftSample | None:
        fields = ["cape"]
        for hpa in ALOFT_LEVELS_HPA:
            fields += [f"wind_speed_{hpa}hPa", f"wind_direction_{hpa}hPa"]
        try:
            r = requests.get(OPEN_METEO_URL, params={
                "latitude": lat, "longitude": lon,
                "hourly": ",".join(fields),
                "wind_speed_unit": "ms",
                "forecast_days": 2,
                "timezone": "UTC",
            }, timeout=self.timeout_s)
            r.raise_for_status()
            h = r.json()["hourly"]
            from datetime import datetime, timezone
            idx = min(datetime.now(timezone.utc).hour + self.hour_offset,
                      len(h["time"]) - 1)
            winds = {}
            for hpa in ALOFT_LEVELS_HPA:
                spd = h[f"wind_speed_{hpa}hPa"][idx]
                drc = h[f"wind_direction_{hpa}hPa"][idx]
                if spd is None or drc is None:
                    return None
                winds[hpa] = (spd, drc)
            cape = h["cape"][idx] or 0.0
            return AloftSample(time_iso=h["time"][idx], winds=winds, cape_jkg=cape)
        except (requests.RequestException, KeyError, ValueError, IndexError):
            return None


def _synthetic_aloft(lat: float, lon: float) -> AloftSample:
    """Plausible NW upper flow strengthening with altitude (jet-like)."""
    import math
    base = 10.0 + 3.0 * math.sin(lat * 2.3) * math.cos(lon * 1.9)
    winds = {700: (base, 320.0), 500: (base * 1.6, 315.0), 400: (base * 2.1, 310.0),
             300: (base * 2.8, 305.0), 250: (base * 3.2, 300.0), 200: (base * 3.0, 300.0)}
    return AloftSample(time_iso="synthetic", winds=winds, cape_jkg=200.0, synthetic=True)


def fetch_metars(station_ids: list[str], timeout_s: float = 15.0) -> list[dict]:
    """Live METARs (e.g. ['LGAV', 'LGTS']). Returns [] on network failure."""
    try:
        r = requests.get(METAR_URL, params={
            "ids": ",".join(station_ids), "format": "json",
        }, timeout=timeout_s)
        r.raise_for_status()
        return r.json()
    except (requests.RequestException, ValueError):
        return []
