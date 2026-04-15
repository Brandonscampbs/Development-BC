#!/usr/bin/env python3
"""Analyze GPS telemetry to detect start/finish line crossings and lap times.

Uses GPS Latitude/Longitude to find the start/finish line location and detect
each of the 22 lap crossings during the Michigan 2025 FSAE endurance event.

Approach: define a virtual timing gate as a line segment on the track, then
detect directional crossings through that gate. The gate must be placed where
the track only passes through once per lap (not at a point where the track
doubles back through the same GPS coordinates).

Usage:
    python scripts/analyze_gps_laps.py
"""

from __future__ import annotations

import csv
import io
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
REPO_ROOT = Path(__file__).resolve().parent.parent
RAW_CSV = REPO_ROOT / "Real-Car-Data-And-Stats" / "2025 Endurance Data.csv"
TRACK_MAP_PNG = Path(__file__).resolve().parent / "track_map.png"
LAPS_PNG = Path(__file__).resolve().parent / "lap_times.png"


def load_raw_data(path: Path) -> pd.DataFrame:
    """Load the raw AiM CSV with duplicate column handling."""
    with open(path, "r", encoding="utf-8") as f:
        all_lines = f.readlines()

    # Parse header (line 15, 0-indexed 14)
    reader = csv.reader(io.StringIO(all_lines[14]))
    raw_columns = next(reader)
    columns: list[str] = []
    seen: dict[str, int] = {}
    for col in raw_columns:
        if col in seen:
            seen[col] += 1
            columns.append(f"{col}.{seen[col]}")
        else:
            seen[col] = 0
            columns.append(col)

    # Data starts at line 18 (0-indexed 17)
    data_text = "".join(all_lines[17:])
    df = pd.read_csv(
        io.StringIO(data_text),
        header=None,
        names=columns,
        dtype=float,
        na_values=[""],
    )
    return df


def detect_lap_crossings_gate(
    time: np.ndarray,
    lat: np.ndarray,
    lon: np.ndarray,
    heading: np.ndarray,
    speed: np.ndarray,
    rpm: np.ndarray,
    nsat: np.ndarray,
) -> tuple[list[float], list[int], dict]:
    """Detect start/finish crossings using a virtual gate approach.

    Strategy:
    1. Find candidate gate locations by looking at where the car passes with
       consistent heading (same direction each lap).
    2. A gate is defined as a short segment of the track. We detect crossings
       by checking when the car crosses a latitude or longitude threshold while
       also being within a narrow band of the perpendicular coordinate AND
       traveling in a consistent heading direction.
    3. This eliminates false crossings where the track passes through the same
       lat or lon at a different part of the circuit.

    Returns:
        (crossing_times, crossing_indices, gate_info)
    """
    # Only consider samples with good GPS and car moving
    driving_mask = (rpm > 50) & (speed > 5.0) & (nsat >= 4)

    good_lat = lat[driving_mask]
    good_lon = lon[driving_mask]
    good_time = time[driving_mask]
    good_heading = heading[driving_mask]
    good_indices = np.where(driving_mask)[0]

    print(f"Driving samples with good GPS: {len(good_lat)}")
    print(f"  Lat range: {good_lat.min():.8f} to {good_lat.max():.8f}")
    print(f"  Lon range: {good_lon.min():.8f} to {good_lon.max():.8f}")

    # Expected lap length ~955m, expected lap time ~65-85s
    # We need 23 crossings for 22 laps
    TARGET_CROSSINGS = 23
    EXPECTED_LAP_DIST = 955  # meters
    MIN_LAP_TIME = 40.0  # seconds - minimum time between valid crossings
    MAX_LAP_TIME = 120.0  # seconds - maximum for normal lap (not DC)

    best_result = None
    best_score = -1

    # Try gates defined by latitude threshold + longitude band + heading band
    lat_min, lat_max = good_lat.min(), good_lat.max()
    lon_min, lon_max = good_lon.min(), good_lon.max()
    lat_range = lat_max - lat_min
    lon_range = lon_max - lon_min

    # We'll scan latitude thresholds and for each one, require the crossing
    # to also be within a longitude band and heading band
    n_lat_candidates = 200
    n_lon_bands = 20

    for i_lat in range(n_lat_candidates):
        frac_lat = (i_lat + 1) / (n_lat_candidates + 2)
        lat_thresh = lat_min + frac_lat * lat_range

        # Find raw crossings of this latitude (upward and downward)
        above = good_lat > lat_thresh

        for direction_val, direction_name in [(True, "northbound"), (False, "southbound")]:
            # direction_val=True means going from below to above (northbound)
            raw_crossing_mask = np.zeros(len(good_lat), dtype=bool)
            if direction_val:
                raw_crossing_mask[1:] = above[1:] & ~above[:-1]
            else:
                raw_crossing_mask[1:] = ~above[1:] & above[:-1]

            crossing_idx_in_good = np.where(raw_crossing_mask)[0]
            if len(crossing_idx_in_good) < 20:
                continue

            # Get lon and heading at crossings
            crossing_lons = good_lon[crossing_idx_in_good]
            crossing_headings = good_heading[crossing_idx_in_good]
            crossing_times_raw = good_time[crossing_idx_in_good]

            # Cluster crossings by longitude to find the main S/F crossing
            # (vs crossings at other parts of the track at the same latitude)
            # Use a simple binning approach
            lon_bins = np.linspace(crossing_lons.min() - 1e-6,
                                   crossing_lons.max() + 1e-6, n_lon_bands + 1)

            for j_lon in range(n_lon_bands):
                lon_lo = lon_bins[j_lon]
                lon_hi = lon_bins[j_lon + 1]
                # Expand the band slightly
                lon_center = (lon_lo + lon_hi) / 2
                lon_halfwidth = (lon_hi - lon_lo) * 2  # wider band
                lon_lo2 = lon_center - lon_halfwidth
                lon_hi2 = lon_center + lon_halfwidth

                in_band = (crossing_lons >= lon_lo2) & (crossing_lons <= lon_hi2)
                band_times = crossing_times_raw[in_band]
                band_indices = crossing_idx_in_good[in_band]

                if len(band_times) < 20:
                    continue

                # Filter by minimum time gap
                filtered_times = [band_times[0]]
                filtered_idx = [band_indices[0]]
                for k in range(1, len(band_times)):
                    if band_times[k] - filtered_times[-1] > MIN_LAP_TIME:
                        filtered_times.append(band_times[k])
                        filtered_idx.append(band_indices[k])

                n_crossings = len(filtered_times)

                # Score: how close to TARGET_CROSSINGS, plus consistency of lap times
                if n_crossings < 20 or n_crossings > 30:
                    continue

                lap_times_raw = np.diff(filtered_times)
                # Remove driver change (the one very long gap)
                driving_laps = lap_times_raw[lap_times_raw < 150]

                if len(driving_laps) < 15:
                    continue

                # Score by: closeness to 23 crossings + low variance in lap times
                count_score = -abs(n_crossings - TARGET_CROSSINGS) * 100
                variance_score = -np.std(driving_laps)
                # Bonus if driving laps are in expected range (60-85s)
                mean_lap = np.mean(driving_laps)
                mean_score = -abs(mean_lap - 73) * 2  # expected ~73s avg

                total_score = count_score + variance_score + mean_score

                if total_score > best_score:
                    best_score = total_score
                    best_result = {
                        "lat_thresh": lat_thresh,
                        "lon_band": (lon_lo2, lon_hi2),
                        "direction": direction_name,
                        "crossing_times": filtered_times,
                        "crossing_indices_good": filtered_idx,
                        "n_crossings": n_crossings,
                        "mean_lap": mean_lap,
                        "std_lap": np.std(driving_laps),
                    }

    # Also try longitude-based gates with latitude bands
    n_lon_candidates = 200
    n_lat_bands = 20

    for i_lon in range(n_lon_candidates):
        frac_lon = (i_lon + 1) / (n_lon_candidates + 2)
        lon_thresh = lon_min + frac_lon * lon_range

        above = good_lon > lon_thresh

        for direction_val, direction_name in [(True, "eastbound"), (False, "westbound")]:
            raw_crossing_mask = np.zeros(len(good_lon), dtype=bool)
            if direction_val:
                raw_crossing_mask[1:] = above[1:] & ~above[:-1]
            else:
                raw_crossing_mask[1:] = ~above[1:] & above[:-1]

            crossing_idx_in_good = np.where(raw_crossing_mask)[0]
            if len(crossing_idx_in_good) < 20:
                continue

            crossing_lats = good_lat[crossing_idx_in_good]
            crossing_times_raw = good_time[crossing_idx_in_good]

            lat_bins = np.linspace(crossing_lats.min() - 1e-6,
                                   crossing_lats.max() + 1e-6, n_lat_bands + 1)

            for j_lat in range(n_lat_bands):
                lat_lo = lat_bins[j_lat]
                lat_hi = lat_bins[j_lat + 1]
                lat_center = (lat_lo + lat_hi) / 2
                lat_halfwidth = (lat_hi - lat_lo) * 2
                lat_lo2 = lat_center - lat_halfwidth
                lat_hi2 = lat_center + lat_halfwidth

                in_band = (crossing_lats >= lat_lo2) & (crossing_lats <= lat_hi2)
                band_times = crossing_times_raw[in_band]
                band_indices = crossing_idx_in_good[in_band]

                if len(band_times) < 20:
                    continue

                filtered_times = [band_times[0]]
                filtered_idx = [band_indices[0]]
                for k in range(1, len(band_times)):
                    if band_times[k] - filtered_times[-1] > MIN_LAP_TIME:
                        filtered_times.append(band_times[k])
                        filtered_idx.append(band_indices[k])

                n_crossings = len(filtered_times)
                if n_crossings < 20 or n_crossings > 30:
                    continue

                lap_times_raw = np.diff(filtered_times)
                driving_laps = lap_times_raw[lap_times_raw < 150]

                if len(driving_laps) < 15:
                    continue

                count_score = -abs(n_crossings - TARGET_CROSSINGS) * 100
                variance_score = -np.std(driving_laps)
                mean_lap = np.mean(driving_laps)
                mean_score = -abs(mean_lap - 73) * 2

                total_score = count_score + variance_score + mean_score

                if total_score > best_score:
                    best_score = total_score
                    best_result = {
                        "lon_thresh": lon_thresh,
                        "lat_band": (lat_lo2, lat_hi2),
                        "direction": direction_name,
                        "crossing_times": filtered_times,
                        "crossing_indices_good": filtered_idx,
                        "n_crossings": n_crossings,
                        "mean_lap": mean_lap,
                        "std_lap": np.std(driving_laps),
                    }

    if best_result is None:
        raise RuntimeError("Could not find a suitable gate location")

    print(f"\nBest gate found (score={best_score:.1f}):")
    for k, v in best_result.items():
        if k not in ("crossing_times", "crossing_indices_good"):
            print(f"  {k}: {v}")

    # Map back to original indices
    crossing_times = best_result["crossing_times"]
    crossing_indices_good = best_result["crossing_indices_good"]
    crossing_indices = [good_indices[i] for i in crossing_indices_good]

    gate_info = {k: v for k, v in best_result.items()
                 if k not in ("crossing_times", "crossing_indices_good")}

    return crossing_times, crossing_indices, gate_info


def analyze_crossings(crossings: list[float]) -> dict:
    """Print lap analysis from crossing times. Returns summary dict."""
    n_laps = len(crossings) - 1
    print(f"\n{'='*70}")
    print(f"LAP ANALYSIS ({len(crossings)} crossings = {n_laps} laps)")
    print(f"{'='*70}")

    print(f"\n{'Crossing':>10}  {'Time (s)':>10}  {'Lap':>5}  {'Lap Time (s)':>13}  {'Notes'}")
    print(f"{'-'*10}  {'-'*10}  {'-'*5}  {'-'*13}  {'-'*20}")

    lap_times = []
    driver_change_idx = -1  # index in lap_times of the DC lap
    driver_change_gap = 0.0

    for i, t in enumerate(crossings):
        if i == 0:
            print(f"{i+1:>10}  {t:>10.2f}  {'':>5}  {'':>13}  Start")
        else:
            lap_time = t - crossings[i - 1]
            lap_times.append(lap_time)
            notes = ""
            if lap_time > 120:
                notes = f"*** DRIVER CHANGE ({lap_time:.1f}s) ***"
                driver_change_idx = i - 1  # index in lap_times
                driver_change_gap = lap_time
            elif lap_time > 85:
                notes = "slow lap"
            print(f"{i+1:>10}  {t:>10.2f}  {i:>5}  {lap_time:>13.2f}  {notes}")

    total_time = crossings[-1] - crossings[0]
    driving_time = total_time - driver_change_gap if driver_change_gap > 0 else total_time

    print(f"\n{'='*70}")
    print(f"SUMMARY")
    print(f"{'='*70}")
    print(f"  First crossing:  {crossings[0]:.2f}s")
    print(f"  Last crossing:   {crossings[-1]:.2f}s")
    print(f"  Total elapsed:   {total_time:.2f}s")
    print(f"  Driving time:    {driving_time:.2f}s (excluding driver change)")

    result = {
        "first_crossing": crossings[0],
        "last_crossing": crossings[-1],
        "total_elapsed": total_time,
        "driving_time": driving_time,
        "n_laps": n_laps,
        "lap_times": lap_times,
    }

    if driver_change_idx >= 0:
        # Stint 1: laps before DC
        stint1_times = lap_times[:driver_change_idx]
        # Stint 2: laps after DC (the DC "lap" itself is at driver_change_idx)
        stint2_times = lap_times[driver_change_idx + 1:]

        dc_lap_num = driver_change_idx + 1  # 1-based
        print(f"\n  Driver change is 'lap' {dc_lap_num} in the list")
        print(f"  Driver change duration: {driver_change_gap:.2f}s "
              f"({driver_change_gap / 60:.1f} min)")

        print(f"\n  Stint 1: {len(stint1_times)} driving laps")
        if stint1_times:
            print(f"    Total: {sum(stint1_times):.2f}s")
            print(f"    Avg lap: {np.mean(stint1_times):.2f}s")
            print(f"    Best lap: {min(stint1_times):.2f}s")
            print(f"    Worst lap: {max(stint1_times):.2f}s")

        print(f"\n  Stint 2: {len(stint2_times)} driving laps")
        if stint2_times:
            print(f"    Total: {sum(stint2_times):.2f}s")
            print(f"    Avg lap: {np.mean(stint2_times):.2f}s")
            print(f"    Best lap: {min(stint2_times):.2f}s")
            print(f"    Worst lap: {max(stint2_times):.2f}s")

        result["driver_change_idx"] = driver_change_idx
        result["driver_change_gap"] = driver_change_gap
        result["stint1_laps"] = len(stint1_times)
        result["stint2_laps"] = len(stint2_times)
        result["stint1_time"] = sum(stint1_times)
        result["stint2_time"] = sum(stint2_times)

    # All driving laps
    driving_laps = [lt for lt in lap_times if lt < 120]
    print(f"\n  All driving laps ({len(driving_laps)}):")
    print(f"    Total: {sum(driving_laps):.2f}s")
    print(f"    Avg: {np.mean(driving_laps):.2f}s")
    print(f"    Min: {min(driving_laps):.2f}s")
    print(f"    Max: {max(driving_laps):.2f}s")
    print(f"    Std: {np.std(driving_laps):.2f}s")

    return result


def plot_track_map(df: pd.DataFrame, output_path: Path,
                   crossings: list[float] | None = None,
                   crossing_indices: list[int] | None = None) -> None:
    """Plot GPS track map with optional crossing markers."""
    lat = df["GPS Latitude"].values
    lon = df["GPS Longitude"].values
    speed = df["GPS Speed"].values
    rpm = df["RPM"].values
    nsat = df["GPS Nsat"].values
    time_arr = df["Time"].values

    # Only plot when car is moving and GPS is good
    mask = (rpm > 50) & (speed > 3.0) & (nsat >= 4)

    fig, axes = plt.subplots(1, 2, figsize=(16, 8))

    # Left: full track colored by speed
    sc = axes[0].scatter(
        lon[mask], lat[mask], c=speed[mask], cmap="RdYlGn", s=1, alpha=0.5
    )
    if crossing_indices:
        axes[0].scatter(
            lon[crossing_indices], lat[crossing_indices],
            c="blue", s=50, marker="x", zorder=5, label="S/F crossings"
        )
        axes[0].legend()
    axes[0].set_xlabel("Longitude")
    axes[0].set_ylabel("Latitude")
    axes[0].set_title("Track Map (colored by speed, km/h)")
    axes[0].set_aspect("equal")
    plt.colorbar(sc, ax=axes[0], label="Speed (km/h)")

    # Right: full track colored by time
    sc2 = axes[1].scatter(
        lon[mask], lat[mask], c=time_arr[mask], cmap="viridis", s=1, alpha=0.5
    )
    if crossing_indices:
        axes[1].scatter(
            lon[crossing_indices], lat[crossing_indices],
            c="red", s=50, marker="x", zorder=5, label="S/F crossings"
        )
        axes[1].legend()
    axes[1].set_xlabel("Longitude")
    axes[1].set_ylabel("Latitude")
    axes[1].set_title("Track Map (colored by time, s)")
    axes[1].set_aspect("equal")
    plt.colorbar(sc2, ax=axes[1], label="Time (s)")

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Track map saved to: {output_path}")


def plot_lap_times(lap_times: list[float], output_path: Path) -> None:
    """Plot lap times bar chart."""
    fig, ax = plt.subplots(figsize=(12, 5))

    colors = []
    for lt in lap_times:
        if lt > 120:
            colors.append("red")  # driver change
        elif lt > 80:
            colors.append("orange")  # slow
        else:
            colors.append("steelblue")

    ax.bar(range(1, len(lap_times) + 1), lap_times, color=colors)
    ax.set_xlabel("Lap Number")
    ax.set_ylabel("Lap Time (s)")
    ax.set_title("Lap Times (red = driver change)")
    ax.axhline(y=np.mean([lt for lt in lap_times if lt < 120]),
               color="green", linestyle="--", label="Avg driving lap")
    ax.legend()
    ax.set_xticks(range(1, len(lap_times) + 1))

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Lap times chart saved to: {output_path}")


def main() -> None:
    """Main entry point."""
    print(f"Loading raw telemetry from: {RAW_CSV}")
    df = load_raw_data(RAW_CSV)

    time_arr = df["Time"].values
    print(f"Raw data: {len(df)} samples, {time_arr[-1]:.1f}s total")

    # Find start/finish crossings
    print("\n--- Start/Finish Line Detection ---")
    lat = df["GPS Latitude"].values
    lon = df["GPS Longitude"].values
    heading = df["GPS Heading"].values
    speed = df["GPS Speed"].values
    rpm = df["RPM"].values
    nsat = df["GPS Nsat"].values

    crossings, crossing_indices, gate_info = detect_lap_crossings_gate(
        time_arr, lat, lon, heading, speed, rpm, nsat
    )

    # Analyze laps
    result = analyze_crossings(crossings)

    # Plot track map with crossings
    print("\n--- Plotting ---")
    plot_track_map(df, TRACK_MAP_PNG, crossings, crossing_indices)

    if result.get("lap_times"):
        plot_lap_times(result["lap_times"], LAPS_PNG)

    # Distance validation
    dist_col = "Distance on GPS Speed"
    if dist_col in df.columns:
        dist = df[dist_col].values
        print(f"\n--- Distance at Crossings ---")
        for i, ct in enumerate(crossings):
            idx = np.argmin(np.abs(time_arr - ct))
            d = dist[idx]
            if i > 0:
                prev_idx = np.argmin(np.abs(time_arr - crossings[i - 1]))
                lap_dist = d - dist[prev_idx]
                print(f"  Crossing {i+1}: t={ct:.2f}s, dist={d:.0f}m, "
                      f"lap_dist={lap_dist:.0f}m")
            else:
                print(f"  Crossing {i+1}: t={ct:.2f}s, dist={d:.0f}m")

    # Print key values for the cleaning script
    print(f"\n--- VALUES FOR CLEANING SCRIPT ---")
    print(f"  Start time (first S/F crossing): {crossings[0]:.2f}s")
    print(f"  End time (last S/F crossing):    {crossings[-1]:.2f}s")
    print(f"  Total elapsed: {crossings[-1] - crossings[0]:.2f}s")

    if "driver_change_idx" in result:
        dc_idx = result["driver_change_idx"]
        dc_start = crossings[dc_idx]
        dc_end = crossings[dc_idx + 1]
        print(f"  Driver change start: {dc_start:.2f}s (crossing {dc_idx + 1})")
        print(f"  Driver change end:   {dc_end:.2f}s (crossing {dc_idx + 2})")
        print(f"  Driver change gap:   {dc_end - dc_start:.2f}s")
        driving = (crossings[-1] - crossings[0]) - (dc_end - dc_start)
        print(f"  Driving time (excluding DC): {driving:.2f}s")


if __name__ == "__main__":
    main()
