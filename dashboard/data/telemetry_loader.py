"""Shared telemetry data loading and caching for the dashboard.

Loads the Michigan 2025 AiM telemetry CSV once and caches it in memory.
All dashboard pages import from here instead of loading independently.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd

from fsae_sim.data.loader import load_aim_csv

# Path to the AiM telemetry CSV (relative to project root)
_PROJECT_ROOT = Path(__file__).parent.parent.parent
_TELEMETRY_PATH = _PROJECT_ROOT / "Real-Car-Data-And-Stats" / "2025 Endurance Data.csv"

# Module-level cache
_cache: dict = {}


def get_telemetry() -> pd.DataFrame:
    """Load and return the AiM telemetry DataFrame (cached)."""
    if "df" not in _cache:
        _metadata, df = load_aim_csv(_TELEMETRY_PATH)
        _cache["metadata"] = _metadata
        _cache["df"] = df
    return _cache["df"]


def get_metadata() -> dict[str, str]:
    """Return session metadata from the AiM CSV."""
    if "metadata" not in _cache:
        get_telemetry()  # populates cache
    return _cache["metadata"]


def get_laps() -> list[tuple[int, int, float]]:
    """Detect and return lap boundaries as (start_idx, end_idx, distance_m).

    Uses GPS latitude-crossing detection from the validation module.
    Results are cached after first call.
    """
    if "laps" not in _cache:
        from fsae_sim.analysis.validation import detect_lap_boundaries
        df = get_telemetry()
        _cache["laps"] = detect_lap_boundaries(df)
    return _cache["laps"]


def get_lap_data(lap_idx: int) -> pd.DataFrame:
    """Extract a single lap with distance normalized to start at 0.

    Args:
        lap_idx: 0-based index into the laps list.

    Returns:
        DataFrame for that lap with a ``lap_distance_m`` column added.
    """
    laps = get_laps()
    if lap_idx < 0 or lap_idx >= len(laps):
        raise IndexError(f"lap_idx {lap_idx} out of range (0-{len(laps) - 1})")

    start_idx, end_idx, _ = laps[lap_idx]
    df = get_telemetry()
    lap = df.iloc[start_idx:end_idx].copy()
    base_dist = lap["Distance on GPS Speed"].iloc[0]
    lap["lap_distance_m"] = lap["Distance on GPS Speed"] - base_dist
    return lap.reset_index(drop=True)


def compute_lap_metrics() -> pd.DataFrame:
    """Compute per-lap summary metrics from telemetry.

    Returns a DataFrame with columns: lap_number, lap_time_s, distance_m,
    mean_speed_kmh, peak_speed_kmh, energy_kwh, soc_start, soc_end.
    """
    if "lap_metrics" in _cache:
        return _cache["lap_metrics"]

    df = get_telemetry()
    laps = get_laps()
    rows = []

    for i, (start, end, dist) in enumerate(laps):
        lap_df = df.iloc[start:end]
        time_start = float(lap_df["Time"].iloc[0])
        time_end = float(lap_df["Time"].iloc[-1])
        lap_time = time_end - time_start

        speed = lap_df["GPS Speed"].values
        mean_speed = float(np.mean(speed[speed > 5.0])) if np.any(speed > 5.0) else 0.0
        peak_speed = float(speed.max())

        soc_start = float(lap_df["State of Charge"].iloc[0])
        soc_end = float(lap_df["State of Charge"].iloc[-1])

        # Energy: integrate V * I over time (positive power = discharge)
        voltage = lap_df["Pack Voltage"].values
        current = lap_df["Pack Current"].values
        power = voltage * current
        dt = np.diff(lap_df["Time"].values, prepend=lap_df["Time"].values[0])
        energy_j = float(np.sum(power[power > 0] * dt[power > 0]))
        energy_kwh = energy_j / 3.6e6

        rows.append({
            "lap_number": i + 1,
            "lap_time_s": lap_time,
            "distance_m": dist,
            "mean_speed_kmh": mean_speed,
            "peak_speed_kmh": peak_speed,
            "energy_kwh": energy_kwh,
            "soc_start": soc_start,
            "soc_end": soc_end,
        })

    result = pd.DataFrame(rows)
    _cache["lap_metrics"] = result
    return result
