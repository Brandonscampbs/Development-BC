# Physics Validation Upgrade Design

**Date:** 2026-04-14
**Status:** Draft
**Context:** After comparing our simulation physics against 7+ professional open-source simulators (OpenLAP, OptimumLap, TORCS, fastest-lap, TUM, Purdue EV), we identified 4 high-impact improvements that close the gaps between our sim and the best-in-class tools without introducing disproportionate complexity.

**Validation baseline:** Our sim currently achieves ~2% energy error and passes 8/8 validation metrics against Michigan 2025 telemetry. These upgrades target the synthetic strategy path (CoastOnly, ThresholdBraking, CalibratedStrategy) used for Phase 3 parameter sweeps. ReplayStrategy is unaffected.

---

## Scope

### In Scope
1. Forward-backward speed envelope for synthetic strategies
2. Effective rotational inertia in F=ma calculations
3. Combined slip reduction in cornering solver
4. Corner speed caching for sweep performance

### Out of Scope (intentionally excluded)
- **Longitudinal tire data** — requires external TTC data, can't fix in code
- **Tire thermal model** — no FSAE sim does this, Hoosier LC0s are thermally stable
- **Transient dynamics** — 3x complexity for 2-5% gain
- **Ground effect aero** — only relevant for underbody diffusers at high speed
- **ReplayStrategy changes** — replay stays faithful to recorded telemetry

---

## 1. Forward-Backward Speed Envelope

### Problem

The current sim uses a single forward pass. When a car arrives at a corner too fast, `resolve_exit_speed()` clamps `v_exit` to the corner speed limit — an instantaneous deceleration that violates F=ma. This produces optimistic lap times and underestimates energy use in braking zones.

Every professional quasi-static lap sim (OpenLAP, OptimumLap, TUM) uses a forward-backward solver instead.

### Design

**New module:** `src/fsae_sim/sim/speed_envelope.py`

**Class:** `SpeedEnvelope`

**Constructor args:**
- `dynamics: VehicleDynamics` — for corner speeds and resistance forces
- `powertrain: PowertrainModel` — for max drive/regen force at each speed
- `track: Track` — segment geometry

**Algorithm: `compute() -> np.ndarray`** returning `v_max[i]` for each segment:

**Pass 1 — Corner speeds:**
```
for each segment i:
    v_corner[i] = dynamics.max_cornering_speed(segment.curvature, segment.grip_factor)
```

**Pass 2 — Backward pass (braking feasibility):**
```
v_back[last] = v_corner[last]
for i from (last-1) down to 0:
    # Max deceleration at v_back[i+1]
    v = v_back[i+1]
    F_resist = dynamics.total_resistance(v)
    F_regen = abs(powertrain.regen_force(1.0, v))
    F_tire_limit = dynamics.max_braking_force(v)
    # Total braking = aero drag + rolling resistance + regen + mechanical braking
    # Capped by tire grip limit (all 4 tires)
    F_brake = min(F_resist + F_regen, F_tire_limit)
    a_brake = F_brake / m_effective
    
    # What speed could we have entered this segment at and still braked to v_back[i+1]?
    v_entry_sq = v_back[i+1]**2 + 2 * a_brake * segment[i].length_m
    v_back[i] = min(v_corner[i], sqrt(v_entry_sq))
```

**Pass 3 — Forward pass (acceleration feasibility):**
```
v_fwd[0] = min(v_back[0], initial_speed)
for i from 1 to last:
    # Max acceleration at v_fwd[i-1]
    v = v_fwd[i-1]
    F_drive = powertrain.drive_force(1.0, v)
    F_drive = min(F_drive, dynamics.max_traction_force(v))
    F_net = F_drive - dynamics.total_resistance(v, segment[i-1].grade)
    a_accel = F_net / m_effective
    
    v_exit_sq = v_fwd[i-1]**2 + 2 * a_accel * segment[i-1].length_m
    v_fwd[i] = min(v_back[i], sqrt(max(0, v_exit_sq)))
```

**Result:** `v_max = v_fwd` — the physically valid speed envelope.

### Integration with SimulationEngine

In `SimulationEngine.run()`, before the segment loop, for synthetic strategies only:

```python
if not is_replay:
    envelope = SpeedEnvelope(self.dynamics, self.powertrain, self.track)
    v_max = envelope.compute(initial_speed=speed)
```

In the segment loop, replace:
```python
corner_limit = self.dynamics.max_cornering_speed(segment.curvature, segment.grip_factor)
```
with:
```python
corner_limit = v_max[lap * num_segments + seg_idx]  # or indexed per-lap
```

The envelope wraps around for multi-lap sims — the last segment's exit speed must be feasible for the first segment's entry speed. For multi-lap, the envelope is computed once for the full track loop (segment N-1 feeds back to segment 0).

### Lap-Wrapping for Multi-Lap Endurance

The backward pass must account for the track being a loop. After the initial backward pass, check if `v_back[last]` > `v_back[0]` (the loop-around constraint). If so, run a second backward pass from `v_back[0]` until convergence (typically 1-2 extra passes). This ensures the car doesn't arrive at the start/finish at a speed it can't sustain into lap 2.

### What Changes in the Sim Loop

The driver strategy still makes all decisions (throttle/coast/brake). The envelope is a **ceiling**, not a controller. The driver can choose to go slower than the envelope (coasting, partial throttle) but cannot exceed it. This means:

- `resolve_exit_speed()` still runs with the driver's actual forces
- But the clamp changes from `v_corner[i]` to `v_max[i]` (which is always <= `v_corner[i]`)
- Braking zones are now physically real — the car decelerates over multiple segments instead of snapping

---

## 2. Effective Rotational Inertia

### Problem

The current sim uses `a = F_net / mass_kg`, ignoring the rotational inertia of the motor rotor, gears, and wheels. These spinning components resist acceleration just like extra translational mass.

### Physics

For a single-speed drivetrain:
```
J_eff = J_motor * G^2 * eta + J_wheels
m_effective = m + J_eff / r^2
```

Where:
- `J_motor` = motor rotor inertia (kg*m^2)
- `G` = gear ratio (3.6363 for CT-16EV)
- `eta` = drivetrain efficiency
- `J_wheels` = total wheel+tire inertia (4 wheels)
- `r` = tire radius (0.228 m)

**Estimated values for CT-16EV:**
| Component | Inertia | Source |
|-----------|---------|--------|
| EMRAX 228 rotor | 0.06 kg*m^2 | Datasheet |
| 4x wheel+tire assemblies | 4 x 0.3 = 1.2 kg*m^2 | Estimate (10" Hoosier + aluminum rim) |
| J_eff at wheel | 0.06 * 3.636^2 * 0.92 + 1.2 = 1.93 kg*m^2 | Computed |
| m_effective | 288 + 1.93 / 0.228^2 = 325 kg | Computed |
| **Effective mass increase** | **~13%** | |

### Design

**Config changes:** Add to `VehicleParams` (or a new `InertiaConfig`):
- `rotor_inertia_kg_m2: float = 0.06` (EMRAX 228 default)
- `wheel_inertia_kg_m2: float = 0.3` (per wheel, default estimate)

**VehicleDynamics changes:**

**Constructor change:** `VehicleDynamics.__init__()` gains an optional `powertrain_config: PowertrainConfig | None = None` parameter. When provided, it computes `m_effective`; when `None`, falls back to `m_effective = mass_kg` (backward compatible).

In `__init__()`:
```python
if powertrain_config is not None:
    G = powertrain_config.gear_ratio
    eta = powertrain_config.drivetrain_efficiency
    r = PowertrainModel.TIRE_RADIUS_M
    J_eff = vehicle.rotor_inertia_kg_m2 * G**2 * eta + 4 * vehicle.wheel_inertia_kg_m2
    self.m_effective = vehicle.mass_kg + J_eff / r**2
else:
    self.m_effective = vehicle.mass_kg
```

`SimulationEngine.__init__()` already has access to `vehicle.powertrain` — pass it through when constructing `VehicleDynamics`.

In `acceleration()`:
```python
return net_force_n / self.m_effective  # was: self.vehicle.mass_kg
```

**Where m_effective is used:**
- `VehicleDynamics.acceleration()` — all F=ma longitudinal calculations
- `SpeedEnvelope` forward/backward passes — acceleration and braking limits
- `resolve_exit_speed()` — indirectly through `acceleration()`

**Where bare mass_kg is still used (correctly):**
- `cornering_solver._can_sustain()` — centripetal force is `m * v^2 * kappa` (rotational inertia doesn't resist lateral acceleration)
- `load_transfer.tire_loads()` — weight is `m * g` (gravitational, not inertial)
- `rolling_resistance_force()` — normal force is gravitational
- `grade_force()` — gravitational

---

## 3. Combined Slip in Cornering Solver

### Problem

`CorneringSolver._can_sustain()` currently checks peak *pure lateral* force — assuming 100% of tire capacity goes to cornering. When the driver is accelerating or braking mid-corner, some capacity is consumed longitudinally, reducing the maximum sustainable corner speed.

### Design

**Modified method signature:**
```python
def max_cornering_speed(
    self,
    curvature: float,
    mu_scale: float = 1.0,
    longitudinal_g: float = 0.0,  # NEW
) -> float:
```

**Modified `_can_sustain()`:**

After computing per-tire normal loads and camber, before summing lateral capacity:

```python
if abs(longitudinal_g) > 0.01:
    # Distribute longitudinal demand to tires
    F_x_total = self._mass_kg * abs(longitudinal_g) * self.GRAVITY
    
    if longitudinal_g > 0:
        # Traction: rear tires only
        fx_rl = F_x_total / 2.0
        fx_rr = F_x_total / 2.0
        fx_fl = fx_fr = 0.0
    else:
        # Braking: all 4 tires, proportional to normal load
        total_fz = fl + fr + rl + rr
        fx_fl = F_x_total * (fl / total_fz)
        fx_fr = F_x_total * (fr / total_fz)
        fx_rl = F_x_total * (rl / total_fz)
        fx_rr = F_x_total * (rr / total_fz)
    
    # Reduce lateral capacity via friction ellipse
    for each tire (fz, camber, fx):
        fx_peak = tire.peak_longitudinal_force(fz)
        fx_ratio = min(abs(fx) / fx_peak, 0.99)  # clamp to avoid sqrt(negative)
        fy_peak = tire.peak_lateral_force(fz, camber)
        fy_available = fy_peak * sqrt(1 - fx_ratio**2)  # elliptical reduction
        total_capacity += fy_available * mu_scale
else:
    # Pure cornering (current behavior, no change)
    total_capacity = sum of peak_lateral_force per tire * mu_scale
```

### Usage in SpeedEnvelope

The speed envelope uses `longitudinal_g` at two points:

1. **Forward pass (accelerating out of corners):** When computing the acceleration-limited exit speed near a corner, also check: "at this acceleration level, can the car still corner?" Pass `longitudinal_g = a_accel / g` to `max_cornering_speed()`. If the combined demand exceeds grip, reduce exit speed.

2. **Backward pass (braking into corners):** When computing the braking-limited entry speed, pass `longitudinal_g = -a_brake / g`. This naturally widens braking zones when approaching tight corners.

**Implementation approach (two-phase):**

Phase 1: Compute the base speed envelope with `longitudinal_g=0.0` (the standard 3-pass algorithm from Section 1). This gives the pure-cornering envelope.

Phase 2 (correction pass): Walk the base envelope and, at each segment where the car is accelerating or braking (speed changing between adjacent segments), compute `longitudinal_g = (v[i+1]^2 - v[i]^2) / (2 * segment_length * g)` from the base envelope. Re-query `max_cornering_speed()` with this `longitudinal_g`. If the corrected corner speed is lower than the base envelope, update it and re-propagate locally (forward or backward as needed). One correction pass is sufficient — full iterative convergence is the 3x complexity version we're avoiding.

This keeps the base 3-pass clean and adds combined slip as a bounded refinement.

### Fallback

When `longitudinal_g=0.0` (default), behavior is identical to current code. All existing tests pass without modification.

---

## 4. Corner Speed Caching

### Problem

The cornering solver runs 30-iteration bisection with 4 tire evaluations per iteration, per segment. A 200-segment track costs ~24,000 tire evaluations for corner speeds alone. In Phase 3 sweeps, this repeats identically when only driver strategy parameters change (not vehicle or track).

### Design

**Cache location:** Inside `SpeedEnvelope`, private dict.

**Cache key:** Tuple of `(track.name, num_segments, vehicle_mass_kg, grip_factor_tuple)` where `grip_factor_tuple` is the per-segment grip factors frozen as a tuple.

**Cache value:** `np.ndarray` of corner speeds, one per segment.

**Lifecycle:**
- Created when `SpeedEnvelope` is constructed
- Populated on first `compute()` call
- Hit on subsequent calls if key matches
- No disk persistence — lives for the duration of the sweep

**API:**

```python
class SpeedEnvelope:
    def __init__(self, dynamics, powertrain, track):
        self._dynamics = dynamics
        self._powertrain = powertrain  
        self._track = track
        self._corner_speed_cache: dict[tuple, np.ndarray] = {}
    
    def _get_corner_speeds(self) -> np.ndarray:
        key = self._cache_key()
        if key in self._corner_speed_cache:
            return self._corner_speed_cache[key]
        
        v_corner = np.array([
            self._dynamics.max_cornering_speed(seg.curvature, seg.grip_factor)
            for seg in self._track.segments
        ])
        self._corner_speed_cache[key] = v_corner
        return v_corner
```

**Sweep usage pattern:**

```python
# Construct once per track+vehicle combo
envelope = SpeedEnvelope(dynamics, powertrain, track)

# Strategy sweeps: corner cache hits every time
for params in strategy_sweep_grid:
    strategy = CalibratedStrategy.with_zone_override(base, **params)
    v_max = envelope.compute(initial_speed=0.5)
    # ... run sim with v_max envelope ...

# Tune sweeps: new envelope per vehicle config (cache misses, correct)
for torque_limit in torque_sweep:
    powertrain = PowertrainModel(modified_config)
    envelope = SpeedEnvelope(dynamics, powertrain, track)
    v_max = envelope.compute(initial_speed=0.5)
```

**Estimated impact:** For a 1000-run strategy-only sweep on a 200-segment track, eliminates ~24 million redundant tire evaluations. Forward/backward passes (~200 segments of basic arithmetic each) remain fast.

---

## Files Changed Summary

| File | Change | Lines |
|------|--------|-------|
| `src/fsae_sim/sim/speed_envelope.py` | **NEW** — SpeedEnvelope class with forward-backward solver and corner speed cache | ~150 |
| `src/fsae_sim/sim/engine.py` | Compute envelope before segment loop for synthetic strategies; use `v_max[i]` as speed limit | ~20 |
| `src/fsae_sim/vehicle/dynamics.py` | Add `m_effective` property; use in `acceleration()` | ~15 |
| `src/fsae_sim/vehicle/cornering_solver.py` | Add `longitudinal_g` parameter to `max_cornering_speed()` and `_can_sustain()`; friction ellipse reduction | ~40 |
| `src/fsae_sim/vehicle/vehicle.py` | Add `rotor_inertia_kg_m2` and `wheel_inertia_kg_m2` fields | ~5 |
| `configs/ct16ev.yaml` | Add inertia parameters | ~3 |
| `configs/ct17ev.yaml` | Add inertia parameters | ~3 |
| `tests/test_speed_envelope.py` | **NEW** — tests for envelope correctness, caching, multi-lap wrap | ~100 |
| `tests/test_dynamics.py` | Add tests for m_effective acceleration | ~20 |
| `tests/test_cornering_solver.py` | Add tests for combined slip reduction | ~30 |

**Total:** ~1 new module, ~6 modified files, ~3 new test files, ~400 lines

---

## Expected Impact

| Metric | Before | After | Why |
|--------|--------|-------|-----|
| Braking zone physics | Instant speed snap at corner entry | Multi-segment deceleration respecting F=ma | Forward-backward solver |
| Acceleration accuracy | ~13% overestimate (missing rotational inertia) | Correct effective mass | m_effective |
| Corner exit speed near throttle zones | Slightly optimistic (ignores longitudinal grip consumption) | Accounts for friction ellipse reduction | Combined slip |
| Strategy sweep runtime | Redundant corner speed computation every run | Cached after first run | Corner speed cache |
| Lap time prediction | Optimistic in braking zones | More conservative, more physically correct | All three physics fixes |
| Energy prediction | Underestimates braking energy, overestimates accel | Corrected force balance | All three physics fixes |

---

## Testing Strategy

1. **Speed envelope unit tests:**
   - Straight-only track: envelope = max speed (powertrain limited)
   - Single corner: braking zone appears before corner, acceleration zone after
   - Symmetric chicane: braking zones are symmetric
   - Multi-lap wrap: entry speed for lap 2 matches exit speed from lap 1

2. **Rotational inertia tests:**
   - `m_effective > mass_kg` always
   - Acceleration is lower with inertia vs without (same force)
   - Corner speed unchanged (lateral, not longitudinal)

3. **Combined slip tests:**
   - `max_cornering_speed(kappa, longitudinal_g=0)` matches current behavior
   - `max_cornering_speed(kappa, longitudinal_g=0.3)` < pure cornering speed
   - Higher longitudinal_g = lower cornering speed (monotonic)

4. **Integration test:**
   - Run full endurance sim with CalibratedStrategy, compare lap time and energy vs current sim
   - Lap time should increase slightly (more realistic braking + slower acceleration)
   - Energy should shift between braking and acceleration zones

5. **Cache tests:**
   - Second `compute()` call returns same array (identity check)
   - Changing track invalidates cache
   - Changing grip factor invalidates cache
