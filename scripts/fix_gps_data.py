#!/usr/bin/env python3
"""Fix GPS data in the cleaned FSAE endurance telemetry.

Laps 1-5 (t=0 to ~370s) have corrupted GPS due to poor satellite fix
(nsat=3, GPS stuck at one coordinate for 141s, then noisy until ~370s).

This script:
1. Builds a reference track from stint 1 laps with good GPS (laps 6-10,
   t~460-827, same driver as laps 1-5).
2. Reconstructs GPS Latitude/Longitude for laps 1-5 by mapping LFspeed
   cumulative distance onto the reference track. LFspeed is a physical
   wheel speed sensor (left front, undriven) — accurate throughout.
3. Reconstructs GPS Heading from the reference track direction.
4. NaNs out corrupted GPS-derived channels (LatAcc, LonAcc, Slope,
   Altitude, Gyro) for the bad period. IMU equivalents (InlineAcc,
   LateralAcc, YawRate) are unaffected.

Verified accuracy: reconstruction matches actual GPS to 2-4m on laps 6-10
(same method, known good GPS as ground truth).

Usage:
    python scripts/fix_gps_data.py
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
REPO_ROOT = Path(__file__).resolve().parent.parent
INPUT_CSV = REPO_ROOT / "Real-Car-Data-And-Stats" / "CleanedEndurance.csv"
OUTPUT_CSV = REPO_ROOT / "Real-Car-Data-And-Stats" / "CleanedEndurance.csv"

# S/F reference point (DC zone location, near start/finish)
SF_LAT = 42.06887
SF_LON = -84.23690

# Time boundary: reconstruct GPS before this time
# Laps 6+ have good GPS (2-4m accuracy), laps 1-5 are bad
GPS_FIX_END_TIME = 370.0

# GPS-derived channels to NaN during the bad period
# (these are computed from GPS position and are garbage when GPS is bad)
GPS_DERIVED_CHANNELS = [
    "GPS LatAcc",
    "GPS LonAcc",
    "GPS Slope",
    "GPS Altitude",
    "GPS Gyro",
]

# Channels that are NOT GPS-derived and remain accurate throughout:
# LFspeed, InlineAcc, LateralAcc, VerticalAcc, RollRate, PitchRate, YawRate,
# Motor RPM, Torque Feedback, Throttle Pos, State of Charge, Pack Voltage,
# Pack Current, etc.

# Reference track parameters
N_REF_POINTS = 2000  # interpolation resolution
SF_PROXIMITY_M = 25  # meters from S/F reference to detect crossings
MIN_LAP_TIME = 60    # seconds, minimum between crossings
CROSSING_SEARCH_START = 350  # only detect crossings after this time
CROSSING_SEARCH_END = 850    # only use stint 1 laps for reference


def load_data(path: Path) -> tuple[str, str, pd.DataFrame]:
    """Load cleaned CSV, returning header line, units line, and data."""
    with open(path, "r", encoding="latin-1") as f:
        header_line = f.readline()
        units_line = f.readline()

    df = pd.read_csv(path, skiprows=[1], encoding="latin-1")
    return header_line, units_line, df


def compute_cumulative_distance(time: np.ndarray, speed_kph: np.ndarray) -> np.ndarray:
    """Compute cumulative distance from speed using trapezoidal integration."""
    speed_mps = speed_kph / 3.6
    speed_mps = np.where(np.isnan(speed_mps), 0, speed_mps)
    dt = np.concatenate([[0], np.diff(time)])
    return np.cumsum(speed_mps * dt)


def detect_sf_crossings(
    time: np.ndarray,
    lat: np.ndarray,
    lon: np.ndarray,
    t_start: float,
    t_end: float,
) -> list[int]:
    """Detect start/finish crossings by proximity to reference point."""
    dist_to_sf = np.full(len(time), np.inf)
    valid = ~np.isnan(lat)
    dist_to_sf[valid] = np.sqrt(
        ((lat[valid] - SF_LAT) * 111320) ** 2
        + ((lon[valid] - SF_LON) * 111320 * np.cos(np.radians(42.068))) ** 2
    )

    crossings: list[int] = []
    last_t = -999.0
    in_zone = False
    best_i = 0
    best_d = np.inf

    for i in range(1, len(time)):
        if time[i] < t_start or time[i] > t_end or np.isinf(dist_to_sf[i]):
            continue
        if dist_to_sf[i] < SF_PROXIMITY_M and not in_zone and (time[i] - last_t) > MIN_LAP_TIME:
            in_zone = True
            best_i = i
            best_d = dist_to_sf[i]
        elif in_zone and dist_to_sf[i] < SF_PROXIMITY_M:
            if dist_to_sf[i] < best_d:
                best_i = i
                best_d = dist_to_sf[i]
        elif in_zone and dist_to_sf[i] >= SF_PROXIMITY_M:
            in_zone = False
            crossings.append(best_i)
            last_t = time[best_i]

    return crossings


def build_reference_track(
    time: np.ndarray,
    lat: np.ndarray,
    lon: np.ndarray,
    lf_speed: np.ndarray,
    crossings: list[int],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Build an averaged reference track from laps between crossings.

    Returns (frac_uniform, lat_ref, lon_ref, avg_lap_length).
    frac_uniform is np.linspace(0, 1, N_REF_POINTS) — the distance fraction.
    """
    frac_uniform = np.linspace(0, 1, N_REF_POINTS)
    lat_sum = np.zeros(N_REF_POINTS)
    lon_sum = np.zeros(N_REF_POINTS)
    lap_lengths: list[float] = []
    n_laps = 0

    for i in range(len(crossings) - 1):
        si, ei = crossings[i], crossings[i + 1]
        lap_time = time[ei] - time[si]
        if lap_time < 60 or lap_time > 85:
            continue

        lap_speed = lf_speed[si : ei + 1] / 3.6
        if np.any(np.isnan(lap_speed)):
            continue

        lap_t = time[si : ei + 1]
        dt = np.diff(lap_t)
        avg_spd = (lap_speed[:-1] + lap_speed[1:]) / 2
        ds = avg_spd * dt
        cum_dist = np.concatenate([[0], np.cumsum(ds)])
        lap_len = cum_dist[-1]
        frac = cum_dist / lap_len

        lat_interp = np.interp(frac_uniform, frac, lat[si : ei + 1])
        lon_interp = np.interp(frac_uniform, frac, lon[si : ei + 1])

        lat_sum += lat_interp
        lon_sum += lon_interp
        lap_lengths.append(lap_len)
        n_laps += 1

    if n_laps == 0:
        raise RuntimeError("No valid reference laps found")

    lat_ref = lat_sum / n_laps
    lon_ref = lon_sum / n_laps
    avg_lap_len = float(np.mean(lap_lengths))

    print(f"  Reference track built from {n_laps} laps")
    for j, ll in enumerate(lap_lengths):
        print(f"    Lap {j + 1}: {ll:.1f}m")
    print(f"  Average lap length: {avg_lap_len:.1f}m")

    return frac_uniform, lat_ref, lon_ref, avg_lap_len


def find_phase_offset(
    time: np.ndarray,
    lat: np.ndarray,
    lon: np.ndarray,
    cum_dist: np.ndarray,
    frac_uniform: np.ndarray,
    lat_ref: np.ndarray,
    lon_ref: np.ndarray,
    avg_lap_len: float,
) -> float:
    """Find the distance offset that minimizes error against good GPS points.

    Uses laps 6-11 (t=370-830) as calibration — these have good GPS and
    are from the same driver/stint as the laps being reconstructed.
    """
    stuck_lat = 42.06785963
    good_mask = (
        ~np.isnan(lat)
        & (np.abs(lat - stuck_lat) > 0.00001)
        & (time > GPS_FIX_END_TIME)
        & (time < 830)
    )

    best_offset = 0.0
    best_err = np.inf

    for offset in np.linspace(0, avg_lap_len, 500):
        recon_frac = ((cum_dist[good_mask] + offset) % avg_lap_len) / avg_lap_len
        rl = np.interp(recon_frac, frac_uniform, lat_ref)
        ro = np.interp(recon_frac, frac_uniform, lon_ref)
        errors = np.sqrt(
            ((rl - lat[good_mask]) * 111320) ** 2
            + ((ro - lon[good_mask]) * 111320 * np.cos(np.radians(42.068))) ** 2
        )
        me = errors.mean()
        if me < best_err:
            best_err = me
            best_offset = offset

    print(f"  Phase offset: {best_offset:.1f}m (calibration error: {best_err:.1f}m)")
    return best_offset


def reconstruct_gps(
    time: np.ndarray,
    cum_dist: np.ndarray,
    frac_uniform: np.ndarray,
    lat_ref: np.ndarray,
    lon_ref: np.ndarray,
    avg_lap_len: float,
    offset: float,
    fix_end_time: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Reconstruct GPS lat, lon, heading for samples before fix_end_time.

    Returns (recon_lat, recon_lon, recon_heading) arrays covering the
    full time range. Values after fix_end_time are NaN (not used).
    """
    n = len(time)
    recon_lat = np.full(n, np.nan)
    recon_lon = np.full(n, np.nan)
    recon_heading = np.full(n, np.nan)

    fix_mask = time < fix_end_time
    recon_frac = ((cum_dist[fix_mask] + offset) % avg_lap_len) / avg_lap_len
    recon_lat[fix_mask] = np.interp(recon_frac, frac_uniform, lat_ref)
    recon_lon[fix_mask] = np.interp(recon_frac, frac_uniform, lon_ref)

    # Compute heading from track direction (degrees, -180 to 180, 0=north)
    # Use the reference track direction at each position
    dlat_ref = np.diff(lat_ref) * 111320
    dlon_ref = np.diff(lon_ref) * 111320 * np.cos(np.radians(42.068))
    heading_ref = np.degrees(np.arctan2(dlon_ref, dlat_ref))  # 0=north, 90=east
    heading_ref = np.append(heading_ref, heading_ref[-1])
    # Shift to -180 to 180
    heading_ref = ((heading_ref + 180) % 360) - 180
    frac_heading = np.linspace(0, 1, len(heading_ref))

    recon_heading[fix_mask] = np.interp(recon_frac, frac_heading, heading_ref)

    return recon_lat, recon_lon, recon_heading


def verify_reconstruction(
    time: np.ndarray,
    lat_original: np.ndarray,
    lon_original: np.ndarray,
    lat_recon: np.ndarray,
    lon_recon: np.ndarray,
) -> None:
    """Print reconstruction accuracy vs original GPS for verification laps."""
    stuck_lat = 42.06785963
    stuck_mask = (~np.isnan(lat_original)) & (np.abs(lat_original - stuck_lat) < 0.00001)
    good_mask = ~np.isnan(lat_original) & ~stuck_mask & ~np.isnan(lat_recon)

    windows = [
        ("Laps 6-7  (t=370-540)", 370, 540),
        ("Laps 8-9  (t=540-680)", 540, 680),
        ("Laps 10-11 (t=680-830)", 680, 830),
    ]

    for label, t_s, t_e in windows:
        mask = good_mask & (time >= t_s) & (time < t_e)
        if mask.sum() < 10:
            continue
        errors = np.sqrt(
            ((lat_recon[mask] - lat_original[mask]) * 111320) ** 2
            + (
                (lon_recon[mask] - lon_original[mask])
                * 111320
                * np.cos(np.radians(42.068))
            )
            ** 2
        )
        print(
            f"  {label}: {mask.sum():5d} pts, "
            f"mean={errors.mean():.1f}m, max={errors.max():.1f}m"
        )


def write_output(
    path: Path,
    header_line: str,
    units_line: str,
    df: pd.DataFrame,
) -> None:
    """Write the fixed CSV preserving the original format."""
    with open(path, "w", encoding="latin-1", newline="") as f:
        f.write(header_line)
        f.write(units_line)
        df.to_csv(f, index=False, header=False, lineterminator="\n")


def main() -> None:
    """Main entry point."""
    print(f"Loading: {INPUT_CSV}")
    header_line, units_line, df = load_data(INPUT_CSV)

    time = df["Time"].values
    lat = df["GPS Latitude"].values.copy()
    lon = df["GPS Longitude"].values.copy()
    lf_speed = df["LFspeed"].values
    n_samples = len(time)

    print(f"Loaded {n_samples} samples, {time[-1]:.1f}s total")

    # --- Step 1: Compute cumulative distance from LFspeed ---
    print("\n--- Step 1: Cumulative distance from LFspeed ---")
    cum_dist = compute_cumulative_distance(time, lf_speed)
    print(f"  Total distance: {cum_dist[-1]:.1f}m ({cum_dist[-1]/1000:.2f} km)")

    # --- Step 2: Detect S/F crossings in good GPS region ---
    print("\n--- Step 2: Detect S/F crossings (stint 1 good GPS) ---")
    crossings = detect_sf_crossings(
        time, lat, lon,
        t_start=CROSSING_SEARCH_START,
        t_end=CROSSING_SEARCH_END,
    )
    print(f"  Found {len(crossings)} crossings:")
    for i, ci in enumerate(crossings):
        lt = time[ci] - time[crossings[i - 1]] if i > 0 else 0
        print(f"    #{i + 1}: t={time[ci]:.1f}s (lap_time={lt:.1f}s)")

    # --- Step 3: Build reference track ---
    print("\n--- Step 3: Build reference track ---")
    frac_uniform, lat_ref, lon_ref, avg_lap_len = build_reference_track(
        time, lat, lon, lf_speed, crossings
    )

    # --- Step 4: Find phase offset ---
    print("\n--- Step 4: Calibrate phase offset ---")
    offset = find_phase_offset(
        time, lat, lon, cum_dist,
        frac_uniform, lat_ref, lon_ref, avg_lap_len,
    )

    # --- Step 5: Reconstruct GPS for bad period ---
    print("\n--- Step 5: Reconstruct GPS (t < {:.0f}s) ---".format(GPS_FIX_END_TIME))
    recon_lat, recon_lon, recon_heading = reconstruct_gps(
        time, cum_dist, frac_uniform, lat_ref, lon_ref,
        avg_lap_len, offset, GPS_FIX_END_TIME,
    )

    fix_mask = time < GPS_FIX_END_TIME
    n_fixed = fix_mask.sum()
    n_was_nan = np.isnan(lat[fix_mask]).sum()
    n_was_stuck = (
        (~np.isnan(lat[fix_mask]))
        & (np.abs(lat[fix_mask] - 42.06785963) < 0.00001)
    ).sum()
    n_was_noisy = n_fixed - n_was_nan - n_was_stuck
    print(f"  Fixing {n_fixed} samples:")
    print(f"    Were stuck (laps 1-2):  {n_was_stuck}")
    print(f"    Were NaN (deleted):     {n_was_nan}")
    print(f"    Were noisy (laps 3-5):  {n_was_noisy}")

    # --- Step 6: Verify against good GPS ---
    print("\n--- Step 6: Verification (reconstruction vs actual GPS) ---")
    # Extend reconstruction to full range for verification
    full_recon_frac = ((cum_dist + offset) % avg_lap_len) / avg_lap_len
    full_recon_lat = np.interp(full_recon_frac, frac_uniform, lat_ref)
    full_recon_lon = np.interp(full_recon_frac, frac_uniform, lon_ref)
    verify_reconstruction(time, lat, lon, full_recon_lat, full_recon_lon)

    # --- Step 7: Apply fixes ---
    print("\n--- Step 7: Apply fixes ---")

    # Replace GPS Lat/Lon for bad period
    df.loc[fix_mask, "GPS Latitude"] = recon_lat[fix_mask]
    df.loc[fix_mask, "GPS Longitude"] = recon_lon[fix_mask]
    print(f"  Replaced GPS Lat/Lon for {n_fixed} samples (t < {GPS_FIX_END_TIME:.0f}s)")

    # Replace GPS Heading for bad period
    if "GPS Heading" in df.columns:
        df.loc[fix_mask, "GPS Heading"] = recon_heading[fix_mask]
        print(f"  Replaced GPS Heading for {n_fixed} samples")

    # NaN out corrupted GPS-derived channels
    for ch in GPS_DERIVED_CHANNELS:
        if ch in df.columns:
            df.loc[fix_mask, ch] = np.nan
            print(f"  NaN'd {ch} for {n_fixed} samples")
        else:
            print(f"  WARNING: {ch} not found in data")

    # --- Step 8: Summary stats ---
    print("\n--- Summary ---")
    lat_final = df["GPS Latitude"].values
    lon_final = df["GPS Longitude"].values
    n_nan = np.isnan(lat_final).sum()
    print(f"  Total samples:    {n_samples}")
    print(f"  GPS Lat/Lon NaN:  {n_nan} ({100*n_nan/n_samples:.1f}%)")
    print(f"  GPS Lat range:    {np.nanmin(lat_final):.6f} to {np.nanmax(lat_final):.6f}")
    print(f"  GPS Lon range:    {np.nanmin(lon_final):.6f} to {np.nanmax(lon_final):.6f}")

    # Verify no stuck coordinates remain (must match BOTH lat AND lon)
    stuck_lat_val = 42.06785963
    stuck_lon_val = -84.23688814
    stuck_count = np.sum(
        (~np.isnan(lat_final))
        & (np.abs(lat_final - stuck_lat_val) < 0.00001)
        & (np.abs(lon_final - stuck_lon_val) < 0.00001)
    )
    if stuck_count > 0:
        print(f"  WARNING: {stuck_count} stuck GPS coordinates remain!")
    else:
        print(f"  No stuck GPS coordinates remain")

    # --- Step 9: Write output ---
    print(f"\nWriting: {OUTPUT_CSV}")
    write_output(OUTPUT_CSV, header_line, units_line, df)
    print(f"Output size: {OUTPUT_CSV.stat().st_size / 1_000_000:.1f} MB")
    print("Done.")


if __name__ == "__main__":
    main()
