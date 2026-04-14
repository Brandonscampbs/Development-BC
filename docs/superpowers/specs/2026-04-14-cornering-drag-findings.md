# Cornering Drag: Missing Physics in Force-Based Simulation

**Date**: 2026-04-14
**Status**: Findings documented, implementation pending
**Context**: CalibratedStrategy validation against Michigan 2025 telemetry

## Problem Statement

The calibrated driver model (Phase 2) produces correct zone classifications and torque intensities, but when run through the force-based simulation engine, it under-predicts both lap time and energy consumption by large margins:

| Metric | Telemetry | Sim | Error |
|---|---|---|---|
| Driving time | 1664.5 s | 1467.0 s | -12% |
| Energy consumed | 3.34 kWh | 2.09 kWh | -37% |
| SOC consumed | 34.5% | 20.5% | -41% |
| Mean pack current | 18.0 A | 13.1 A | -27% |

The sim is too fast and too efficient. This is not a calibration issue — the zone actions and torque intensities are correct. The problem is missing physics in the force balance.

## Root Cause Analysis

### Energy Budget

The sim under-predicts energy by 1.25 kWh over 22 laps (~22 km). Converting to force:

```
Missing energy:  1.25 kWh = 4,500,000 J
Total distance:  ~22,000 m
Missing force:   4,500,000 / 22,000 ≈ 205 N average
```

The currently modeled resistance at typical FSAE speeds (~40 km/h = 11.1 m/s):
- Aero drag (CdA=1.50): `0.5 * 1.225 * 1.50 * 11.1² = 113 N`
- Rolling resistance (288 kg, Crr≈0.015): `0.015 * 288 * 9.81 = 42 N`
- **Total modeled: ~155 N**

We need **~205 N more** resistance than currently modeled. That's more than the entire existing resistance budget.

### The Missing Force: Cornering Drag

When a tire generates lateral force (cornering), it operates at a slip angle. This slip angle causes a longitudinal drag component — energy dissipated through rubber deformation and sliding at the contact patch. This is a well-known tire phenomenon captured by the Pacejka model but **never applied to the force balance in the current sim**.

The Michigan endurance track is ~70%+ corners (199 segments, most with nonzero curvature). The cornering drag is therefore present for most of the lap.

### Quantification

For a representative corner segment (curvature = 0.02 1/m, speed = 40 km/h):

```
Required lateral force:  F_lat = m * v² * κ = 288 * 11.1² * 0.02 = 710 N
Per tire (4 tires):      ~178 N
Slip angle needed:       α ≈ F_lat / C_α ≈ 178 / 800 ≈ 0.22 rad ≈ 12.5°
Drag per tire:           F_drag ≈ F_lat * tan(α) ≈ 178 * tan(12.5°) ≈ 39 N
Total cornering drag:    4 * 39 ≈ 156 N
```

This ~156 N of cornering drag in corners is nearly as large as the entire existing resistance model (~155 N). It directly explains the missing energy.

### Verification with Pacejka Model

The existing `PacejkaTireModel` can compute this properly. The model outputs both:
- `Fy(α)`: lateral force as a function of slip angle
- `Fx(α, κ)`: longitudinal force under combined slip conditions

The combined-slip friction circle means that when the tire is generating lateral force, its available longitudinal force is reduced AND there's a self-aligning drag component. The Pacejka model encodes all of this.

### Why This Also Fixes Lap Time

Adding cornering drag slows the car in corners:
- Additional deceleration: `156 N / 288 kg = 0.54 m/s²`
- In a 30m corner at 40 km/h: exit speed drops from 13.9 to ~12.7 m/s
- Per-corner time increase: ~5%
- With 70% of track in corners: ~3.5% overall lap time increase

Combined with the energy-correct torque calibration, this should bring time error from 12% down to ~5-8%.

## Confirmed Calibration Results

Before implementing cornering drag, the following calibration issues were found and fixed:

### Fix 1: Per-Lap Classification (Critical)

**Problem**: `extract_per_segment_actions` was using `dist % lap_dist` to map all telemetry samples to segment bins. This mixed coast and throttle samples from different laps into the same bin, averaging out to ~20% throttle everywhere. Result: 0 coast zones detected.

**Fix**: Classify per-lap independently (using `detect_lap_boundaries` to slice laps), then aggregate across laps with majority vote for action type. Skips first lap and outlier-distance laps (driver change).

**Result**: 48 zones detected with proper coast/brake/throttle mix (~11 coast, 3 brake, 34 throttle zones).

### Fix 2: Torque-Based Intensity (Critical)

**Problem**: Throttle intensity was `pedal_position / 100`. But the real car maps pedal position to LVCU torque request (0-150 Nm), and the inverter clips to 85 Nm. The sim maps `throttle_pct * 85 Nm`. So at 40% pedal: real car requests 60 Nm, sim applies 34 Nm (43% deficit).

**Evidence from telemetry**:
| Pedal Range | LVCU Torque Req | Sim Torque (old) | Error |
|---|---|---|---|
| 5-20% | 19.0 Nm | 10.6 Nm | -44% |
| 20-40% | 40.0 Nm | 25.5 Nm | -36% |
| 40-60% | 64.5 Nm | 42.5 Nm | -34% |
| 60-80% | 91.1 Nm | 59.5 Nm | -35% |

**Fix**: Use `LVCU Torque Req` from telemetry as intensity source, normalized to the 85 Nm inverter limit. This produces `intensity = min(torque_req / 85, 1.0)`, which when multiplied by 85 Nm in the sim reproduces the actual clamped torque.

**Torque Feedback confirmation**: The `Torque Feedback` channel (actual motor output) correlates with LVCU Torque Req at r=0.92 (r=0.98 when cleaned of sensor spikes). At 60-100% pedal, LVCU requests 95 Nm but Torque Feedback shows 83.6 Nm — confirming the 85 Nm inverter clip. The MCU Torque Limit channel is constant at 86.7 Nm.

### Fix 3: Merge Tolerance (Minor)

**Problem**: Default merge tolerance of 0.05 produced 85 zones (too noisy).

**Fix**: Increased to 0.15, producing 48 zones (within 25-50 target range).

## Current State: What to Remove

The `CalibratedStrategy.decide()` method currently has a speed governor that coasts when `speed > max_speed_ms`. **This should be removed** once cornering drag is implemented — it's a bandaid that constrains the sim to telemetry speeds instead of letting the physics produce the right answer.

The `max_speed_ms` field on `DriverZone` can be kept for diagnostic purposes but should not influence `decide()`.

## Implementation Plan: Cornering Drag

### Where to Add It

**File**: `src/fsae_sim/vehicle/dynamics.py` — `VehicleDynamics.total_resistance()`

Currently `total_resistance(speed, grade)` computes aero drag + rolling resistance + grade. It needs a `curvature` parameter to add cornering drag.

### Algorithm

```python
def cornering_drag(self, speed_ms: float, curvature: float) -> float:
    """Compute tire cornering drag force (N) from slip angle losses.
    
    When the car corners, tires operate at slip angles that generate
    longitudinal drag. Uses the Pacejka tire model if available,
    otherwise falls back to an analytical approximation.
    """
    if abs(curvature) < 1e-6 or speed_ms < 1.0:
        return 0.0
    
    # Required lateral acceleration
    a_lat = speed_ms ** 2 * abs(curvature)  # m/s²
    
    # Total lateral force needed
    f_lat_total = self.mass_kg * a_lat  # N
    
    if self._tire_model is not None:
        # Use Pacejka: find slip angle that produces needed lateral force,
        # then compute the drag component at that slip angle.
        # Split load across 4 tires (with load transfer if available).
        # Cornering drag ≈ sum of Fx(α) across all 4 tires.
        ...
    else:
        # Analytical fallback: drag ≈ F_lat² / (C_α * num_tires)
        # C_α ≈ peak_lateral_force / peak_slip_angle
        c_alpha_total = self.mass_kg * 9.81 * 1.5 / 0.15  # rough estimate
        return f_lat_total ** 2 / c_alpha_total
```

### Sim Engine Changes

**File**: `src/fsae_sim/sim/engine.py`

In the force-based mode, change:
```python
resist_f = self.dynamics.total_resistance(speed, segment.grade)
```
to:
```python
resist_f = self.dynamics.total_resistance(speed, segment.grade, segment.curvature)
```

### Validation Targets

After implementing cornering drag, re-run `scripts/validate_driver_model.py`:

| Metric | Target | Rationale |
|---|---|---|
| Driving time | < 8% error | Cornering drag adds ~3.5% lap time; remaining gap is driver conservatism |
| Energy consumed | < 10% error | Cornering drag adds ~150 N average; accounts for bulk of missing energy |
| Mean pack current | < 15% error | Higher resistance → more current draw |
| Combined FSAE score | < 10% error | Time and energy corrections compound |

### Test Plan

1. Unit test `cornering_drag()` against analytical solutions (known curvature, known tire properties)
2. Verify `total_resistance()` with curvature produces higher forces than without
3. Verify cornering drag is zero on straight segments (curvature ≈ 0)
4. Run full 22-lap validation and check energy/time convergence
5. Existing 383 tests must continue to pass (cornering drag should default to 0 when no tire model)

## Other Observations

### Telemetry Data Quality

- **Torque Feedback** has occasional sensor spikes (max 4976 Nm). 99.7% of values are in 0-150 Nm range. Use cleaned values for any comparison.
- **Torque Command** ≈ **LVCU Torque Req** (r=0.98). Either can be used for calibration.
- **MCU Torque Limit** is constant at 86.7 Nm (confirms inverter clip point; 85 Nm in config is close enough).
- Throttle pedal position reaches 100% in rare bursts but p95 = 62%. Driver mostly operated at 20-50% pedal.

### Track Characteristics

- 199 segments at 5m each = 995m per lap
- 18 laps detected from telemetry (some laps merged due to GPS lap detection)
- First lap and driver-change lap automatically excluded from calibration
- The track is almost entirely corners — very few straight segments. This makes cornering drag the dominant missing physics.

### Secondary Missing Physics (Lower Priority)

- **Weight transfer effect on rolling resistance**: loaded tires have higher Crr. Second-order, ~5-10 N.
- **Aerodynamic yaw drag**: car at yaw angle has higher CdA. Second-order.
- **Residual brake drag**: friction from brake pads. 5-20 N. Small.
- **Tire temperature effects**: warmer tires have different grip and rolling resistance. Already partially modeled in battery temp but not tire.

These are all much smaller than cornering drag and can be deferred.
