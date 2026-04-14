# Dashboard Redesign Spec

**Date:** 2026-04-14
**Status:** Draft — ready for review

---

## Problem

The dashboard was scaffolded early in Phase 1 as empty placeholders. The project has since completed nearly all of Phase 2 — validated simulation engine, calibrated driver model, FSAE scoring, real telemetry — but the dashboard reflects none of this. 5 of 6 pages are a single alert banner saying "available later." The Overview shows `"--"` for every metric.

The page *concepts* (Overview, Strategy, Cars, Sweep, Pareto, Lap Detail) still align with the roadmap. The gap is: **nothing is useful today, and the pages that could be useful today don't exist yet.**

---

## Goals

1. Make the dashboard useful *now* with Phase 2 data (telemetry, validation results, baseline sim).
2. Prepare data loading infrastructure for Phase 3 sweep results.
3. Add pages that support driver coaching (Phase 4) early — zone visualization on the track map.
4. Remove fictional CLI commands and placeholder content that would confuse team members.

---

## Proposed Page Structure

### Tier 1 — Immediately useful (Phase 2 data exists today)

#### 1. Overview (rework existing)
**What changes:** Replace placeholder cards and fictional commands with real data.
- Run a baseline sim on page load (or read cached results) for CT-16EV Michigan 2025.
- Populate metric cards: Best Lap Time, Total Energy, Final SOC, Predicted Points (from `FSAEScoring`).
- SOC depletion chart with real data (SOC vs distance over 22 laps).
- Quick summary: "Simulation validated against Michigan 2025 telemetry — 2% energy error, 8/8 metrics pass."
- Remove the fake "Getting Started" CLI commands.

**Data source:** Single baseline simulation result (can be pre-computed and cached as Parquet/JSON).

#### 2. Telemetry Explorer (new page)
**Why:** The AiM telemetry is the foundation everything validates against. The team should be able to see it.
- Load `2025 Endurance Data.csv` (20Hz, ~37k samples).
- Channels: GPS Speed, Throttle Pos, Brake Pressure, RPM, Pack Current, SOC, Pack Temp.
- X-axis toggle: time vs. distance.
- Lap selector (dropdown or slider) to isolate individual laps.
- Vertical crosshair that syncs across all channel plots.
- GPS track map colored by speed (or any selected channel).

**Data source:** `Real-Car-Data-And-Stats/2025 Endurance Data.csv` — pure visualization, no simulation needed.

#### 3. Validation Report (new page)
**Why:** Phase 2's achievement is ~2% energy error and 8/8 metrics passing. This is what proves the sim is trustworthy before Phase 3 sweeps.
- Side-by-side: simulated vs. real speed trace (overlaid on same axes).
- Energy comparison: simulated vs. real cumulative energy over distance.
- Metric scorecard table: the 8 validation metrics with pass/fail status and error percentages.
- Per-lap breakdown: lap time and energy error for each of the 22 laps.

**Data source:** `analysis/telemetry_analysis.py` outputs + a baseline simulation run. Can be pre-computed.

#### 4. Driver Zones / Track Map (new page)
**Why:** The CalibratedStrategy with ~30-40 zones is a Phase 2 deliverable and the foundation for Phase 3 zone overrides and Phase 4 driver coaching.
- GPS track map with zones color-coded by action: green = throttle, blue = coast, red = brake.
- Zone intensity shown by color saturation or line thickness.
- Click a zone to see details: zone ID, action, intensity, distance span, speed range.
- Overlay: comparison between CalibratedStrategy zones and raw telemetry behavior.

**Data source:** `CalibratedStrategy` zone data + GPS coordinates from telemetry.

### Tier 2 — Phase 3 readiness (build infrastructure, populate when sweep data arrives)

#### 5. Strategy Comparison (rework existing placeholder)
- Build the data loading layer for simulation results.
- When results exist: overlaid speed traces, energy-per-lap bars, cumulative SOC curves, scoring breakdown.
- Dropdown to select which strategies to compare.
- Pre-populate with at least 2 baseline strategies (e.g., CalibratedStrategy vs. coast-only) from a quick sim run.

#### 6. Car Comparison (rework existing placeholder)
- Side-by-side parameter table (CT-16EV vs CT-17EV) loaded from `configs/ct16ev.yaml` and `configs/ct17ev.yaml`.
- When simulation results exist: performance comparison on same strategy.
- This page can show the parameter table immediately (no simulation needed).

#### 7. Parameter Sweep (rework existing placeholder)
- Define the results data contract: Parquet schema, manifest.json format.
- Build shared `results_loader.py` that all pages use.
- When sweep data arrives: heatmaps (param1 vs param2, colored by score), sensitivity plots (single param vs. metric), top-N configurations table.

#### 8. Pareto Frontier (rework existing placeholder)
- Same data loading layer as Sweep.
- When data arrives: scatter plot (lap time vs. energy), Pareto frontier line, click-to-inspect details.
- Color points by combined score.

### Tier 3 — Deep analysis

#### 9. Lap Detail (rework existing placeholder)
- Segment-by-segment view of a single simulation run.
- Speed, torque, current, SOC, tire forces vs. distance.
- Dropdown to select which run to inspect.
- Useful for debugging simulation issues and understanding zone behavior.

---

## Shared Infrastructure to Build

### Results data contract
```
results/
  sweep_{param}_{car}_{timestamp}/
    manifest.json          # sweep metadata, parameter ranges, car config
    summary.parquet        # one row per run: params + key metrics + score
    run_{NNN}.parquet      # full time-series for each run (optional, for Lap Detail)
```

### `dashboard/data/` module
- `telemetry_loader.py` — loads and caches AiM CSV, provides lap splitting.
- `results_loader.py` — discovers results directories, reads manifests and summaries.
- `sim_runner.py` — runs a single baseline sim and caches the result (for Overview, Validation).
- `zone_loader.py` — extracts CalibratedStrategy zones for the track map page.

### Shared components
- `components/metric_card.py` — the existing metric card, but wired to real data.
- `components/track_map.py` — reusable GPS track map (used by Telemetry Explorer, Driver Zones, Lap Detail).
- `components/channel_plot.py` — reusable multi-channel time/distance plot with synced crosshair.

---

## Build Order

| Priority | Page | Depends on | Estimated complexity |
|----------|------|------------|---------------------|
| 1 | Telemetry Explorer | `telemetry_loader.py` only | Low — pure data viz |
| 2 | Overview (rework) | Baseline sim runner + cache | Medium |
| 3 | Driver Zones / Track Map | `zone_loader.py` + track map component | Medium |
| 4 | Validation Report | Baseline sim + telemetry comparison | Medium |
| 5 | Car Comparison (params table) | YAML config loading | Low |
| 6 | Strategy Comparison (baseline) | 2 sim runs + comparison view | Medium |
| 7 | Results loader infrastructure | Parquet schema definition | Medium |
| 8 | Parameter Sweep | Results loader + sweep data | High (Phase 3) |
| 9 | Pareto Frontier | Results loader + sweep data | Medium (Phase 3) |
| 10 | Lap Detail | Results loader + run data | Medium |

---

## What to Remove / Fix

- **Overview:** Remove the fictional CLI commands (`python -m fsae_sim.sim.engine --config ...`). Replace with real instructions or remove entirely.
- **All placeholder pages:** Replace "available after X" alerts with either real content (Tier 1) or a more informative stub explaining what data is needed and how it will look.
- **Dashboard.md (Obsidian):** Update status from "Stub (Phase 3)" once Tier 1 pages are built.

---

## Open Questions

1. **Caching strategy:** Should baseline sim results be pre-computed at build time (baked into Docker image) or computed on first page load? Pre-computed is simpler and faster.
2. **Telemetry file size:** The AiM CSV is ~37k rows at 20Hz. Should we pre-process into a lighter format, or is pandas fast enough for interactive use?
3. **Track map library:** Plotly `scattermapbox` (needs Mapbox token) vs. `scattergeo` vs. simple XY scatter of lat/lon? Simple XY scatter is probably fine for a closed circuit.
4. **Results directory location:** Hardcode `results/` or make configurable via env var?
