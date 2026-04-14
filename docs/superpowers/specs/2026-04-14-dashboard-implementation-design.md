# Dashboard Implementation Design

**Date:** 2026-04-14
**Status:** Approved for implementation

---

## Design Decision

Simplify the dashboard from 6 placeholder pages to 3 functional pages focused on **simulation output, validation, and optimization results**. The user has other tools for raw telemetry analysis — this dashboard exists to visualize what the simulation produces and prove it's trustworthy.

### Design Principles

Every feature must serve one of these goals (from CLAUDE.md roadmap):
1. **Show simulation results** — the sim is the product
2. **Prove the sim is accurate** — validation against real data (Phase 2)
3. **Support parameter optimization** — sweep results and scoring (Phase 3)

Features that don't serve these goals are cut.

### Industry Research Summary

Motorsport engineering dashboards consistently prioritize:
- **Line charts** for time-series data (speed, SOC, energy over distance)
- **Metric cards** for at-a-glance KPIs
- **Comparative overlays** (sim vs real on shared axes) for validation
- **Heatmaps and sensitivity plots** for parameter exploration
- **Scoring breakdowns** for competition strategy

Sources: SciChart motorsport telemetry, Motorsport Engineer data visualization article, Formula E AVA dashboard, F1 Dash community examples.

---

## Page Structure

### 1. Simulation Overview (`/`)

**Purpose:** Show what the baseline simulation produces and how it scores.

**Justification:** The sim is the core deliverable. A team member should be able to open the dashboard and immediately see: how fast, how much energy, what score. This is the answer to "is our sim working?"

**Metric cards:**
- Total Time (s) — endurance driving time
- Energy Used (kWh) — total electrical energy consumed
- Final SOC (%) — battery state at end of run
- FSAE Score — combined endurance + efficiency points (out of 375)

**Charts:**
- SOC vs Distance (line) — shows energy depletion pattern, critical for strategy
- Speed vs Distance (line) — shows sim's speed profile over full endurance
- Scoring breakdown table — endurance time score, laps score, efficiency factor, efficiency score

**Data source:** Runs baseline ReplayStrategy simulation of Michigan 2025 on first load, caches in memory.

### 2. Validation (`/validation`)

**Purpose:** Prove the simulation matches reality. Build trust before Phase 3 optimization.

**Justification:** CLAUDE.md architecture guidance: "validate every model against real data before adding complexity." Phase 3 sweeps are meaningless if the sim doesn't match reality. This page IS that proof.

**Visualizations:**
- Speed overlay: sim speed trace vs telemetry speed trace on same distance axis
- SOC overlay: sim SOC vs telemetry SOC on same distance axis
- Validation metric table: 8 metrics with pass/fail status, error %, target threshold
- Summary badge: "N/8 metrics pass"

**Data source:** Baseline sim result + AiM telemetry CSV. Uses `validate_full_endurance()` from analysis module.

### 3. Sweep Results (`/sweeps`)

**Purpose:** Visualize parameter sweep results when they exist (Phase 3).

**Justification:** Phase 3 is next per the roadmap. The dashboard needs to be ready to show sweep results as soon as they're generated. Building the data loading infrastructure now means sweeps can be visualized immediately.

**When sweep data exists:**
- Heatmap: param vs metric, colored by combined score
- Top-N configurations table
- Best result details

**When no sweep data exists:**
- Clear message explaining Phase 3 status and what will appear here

**Data source:** `results/` directory — reads Parquet summary files and JSON manifests.

---

## Data Layer

### `dashboard/data/telemetry_loader.py`
Loads and caches the AiM telemetry CSV. Used by validation page for the "real" side of sim vs real comparison.

### `dashboard/data/sim_runner.py`
Runs the baseline simulation pipeline and caches results. Provides:
- `get_baseline_result()` → SimResult
- `get_baseline_score()` → FSAEScoreResult
- `get_validation_report()` → ValidationReport

Pipeline: VehicleConfig → Track → BatteryModel → ReplayStrategy → SimulationEngine → SimResult → FSAEScoring

### Dependencies (all existing, validated code)
- `fsae_sim.sim.engine.SimulationEngine`
- `fsae_sim.vehicle.VehicleConfig`
- `fsae_sim.vehicle.battery_model.BatteryModel`
- `fsae_sim.track.track.Track`
- `fsae_sim.driver.strategies.ReplayStrategy`
- `fsae_sim.analysis.scoring.FSAEScoring`
- `fsae_sim.analysis.validation.validate_full_endurance`

---

## Build Order

1. `dashboard/data/sim_runner.py` — simulation pipeline + caching
2. `dashboard/data/telemetry_loader.py` — telemetry loading (already written)
3. `dashboard/pages/overview.py` — sim results + scoring
4. `dashboard/pages/validation.py` — sim vs reality
5. `dashboard/pages/sweeps.py` — sweep results viewer
6. `dashboard/app.py` — update nav to 3 pages
7. Delete old placeholder pages
