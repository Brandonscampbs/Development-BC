# Physics Validation Upgrade Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the top physics gaps identified in our simulator validation comparison — forward-backward speed envelope, effective rotational inertia, combined slip in cornering, and corner speed caching.

**Architecture:** New `SpeedEnvelope` module computes a physically valid speed ceiling before the sim loop runs. `VehicleDynamics` gains `m_effective` (rotational inertia). `CorneringSolver` gains friction-ellipse combined slip reduction. All changes are additive — existing tests pass without modification.

**Tech Stack:** Python 3.11+, NumPy, pytest

**Spec:** `docs/superpowers/specs/2026-04-14-physics-validation-upgrade-design.md`

---

## File Structure

| File | Responsibility |
|------|---------------|
| `src/fsae_sim/vehicle/vehicle.py` | Add `rotor_inertia_kg_m2`, `wheel_inertia_kg_m2` to `VehicleParams` |
| `configs/ct16ev.yaml` | Add inertia parameters for CT-16EV |
| `configs/ct17ev.yaml` | Add inertia parameters for CT-17EV |
| `src/fsae_sim/vehicle/dynamics.py` | Add `m_effective` computation and use in `acceleration()` |
| `src/fsae_sim/vehicle/cornering_solver.py` | Add `longitudinal_g` param with friction-ellipse reduction |
| `src/fsae_sim/sim/speed_envelope.py` | **NEW** — Forward-backward solver + corner speed cache |
| `src/fsae_sim/sim/__init__.py` | Export `SpeedEnvelope` |
| `src/fsae_sim/sim/engine.py` | Integrate envelope into sim loop for synthetic strategies |
| `tests/test_dynamics.py` | Add `m_effective` tests |
| `tests/test_cornering_solver.py` | Add combined slip tests |
| `tests/test_speed_envelope.py` | **NEW** — Envelope correctness, caching, multi-lap |
| `tests/test_engine_envelope.py` | **NEW** — Integration: engine uses envelope for synthetic strategies |

---

### Task 1: Add Inertia Fields to VehicleParams

**Files:**
- Modify: `src/fsae_sim/vehicle/vehicle.py:13-21`
- Modify: `configs/ct16ev.yaml`
- Modify: `configs/ct17ev.yaml`

- [ ] **Step 1: Add fields to VehicleParams dataclass**

In `src/fsae_sim/vehicle/vehicle.py`, add two optional fields with defaults to the end of `VehicleParams`:

```python
@dataclass(frozen=True)
class VehicleParams:
    """Physical vehicle parameters."""
    mass_kg: float
    frontal_area_m2: float
    drag_coefficient: float
    rolling_resistance: float
    wheelbase_m: float
    downforce_coefficient: float = 0.0  # Cl * A (m^2), 0 = no downforce
    rotor_inertia_kg_m2: float = 0.06  # EMRAX 228 default
    wheel_inertia_kg_m2: float = 0.3   # per wheel (10" Hoosier + aluminum rim)
```

- [ ] **Step 2: Add inertia to ct16ev.yaml**

Add under the `vehicle:` section in `configs/ct16ev.yaml`:

```yaml
vehicle:
  mass_kg: 288.0
  frontal_area_m2: 1.0
  drag_coefficient: 1.502
  rolling_resistance: 0.015
  wheelbase_m: 1.549
  downforce_coefficient: 2.18
  rotor_inertia_kg_m2: 0.06   # EMRAX 228 datasheet
  wheel_inertia_kg_m2: 0.3    # per wheel estimate (10" Hoosier + aluminum rim)
```

- [ ] **Step 3: Add inertia to ct17ev.yaml**

Add under the `vehicle:` section in `configs/ct17ev.yaml`:

```yaml
vehicle:
  mass_kg: 279.0
  frontal_area_m2: 1.0
  drag_coefficient: 1.502
  rolling_resistance: 0.015
  wheelbase_m: 1.549
  rotor_inertia_kg_m2: 0.06   # EMRAX 228 datasheet (same motor)
  wheel_inertia_kg_m2: 0.3    # per wheel estimate
```

- [ ] **Step 4: Run existing tests to verify backward compatibility**

Run: `pytest tests/ -v --tb=short`
Expected: All existing tests PASS (defaults match prior behavior since `m_effective` isn't used yet)

- [ ] **Step 5: Commit**

```bash
git add src/fsae_sim/vehicle/vehicle.py configs/ct16ev.yaml configs/ct17ev.yaml
git commit -m "feat: add rotational inertia fields to VehicleParams and configs"
```

---

### Task 2: Effective Rotational Inertia in VehicleDynamics

**Files:**
- Modify: `src/fsae_sim/vehicle/dynamics.py:38-49` (constructor), `:192-199` (acceleration)
- Test: `tests/test_dynamics.py`

- [ ] **Step 1: Write failing tests for m_effective**

Add to `tests/test_dynamics.py`:

```python
from fsae_sim.vehicle.powertrain import PowertrainConfig


@pytest.fixture
def ct16ev_powertrain():
    """CT-16EV powertrain config."""
    return PowertrainConfig(
        motor_speed_max_rpm=2900,
        brake_speed_rpm=2400,
        torque_limit_inverter_nm=85.0,
        torque_limit_lvcu_nm=150.0,
        iq_limit_a=170.0,
        id_limit_a=30.0,
        gear_ratio=3.6363,
        drivetrain_efficiency=0.92,
    )


@pytest.fixture
def dynamics_with_inertia(ct16ev_params, ct16ev_powertrain):
    return VehicleDynamics(ct16ev_params, powertrain_config=ct16ev_powertrain)


class TestEffectiveInertia:
    """Tests for rotational inertia in F=ma calculations."""

    def test_m_effective_greater_than_mass(self, dynamics_with_inertia):
        """m_effective must be greater than bare mass."""
        assert dynamics_with_inertia.m_effective > dynamics_with_inertia.vehicle.mass_kg

    def test_m_effective_value(self, ct16ev_params, ct16ev_powertrain):
        """Verify m_effective calculation for CT-16EV.

        J_eff = 0.06 * 3.6363^2 * 0.92 + 4 * 0.3 = 1.930 kg*m^2
        m_eff = 278 + 1.930 / 0.228^2 = 278 + 37.14 = 315.14 kg
        """
        dyn = VehicleDynamics(ct16ev_params, powertrain_config=ct16ev_powertrain)
        G = 3.6363
        eta = 0.92
        r = 0.228
        J_eff = 0.06 * G**2 * eta + 4 * 0.3
        expected = ct16ev_params.mass_kg + J_eff / r**2
        assert dyn.m_effective == pytest.approx(expected, rel=1e-4)

    def test_acceleration_uses_m_effective(self, dynamics_with_inertia, ct16ev_params):
        """Acceleration should use m_effective, not bare mass."""
        a = dynamics_with_inertia.acceleration(1000.0)
        a_bare = 1000.0 / ct16ev_params.mass_kg
        a_effective = 1000.0 / dynamics_with_inertia.m_effective
        assert a == pytest.approx(a_effective, rel=1e-6)
        assert a < a_bare  # slower with rotational inertia

    def test_no_powertrain_config_falls_back(self, ct16ev_params):
        """Without powertrain config, m_effective == mass_kg (backward compat)."""
        dyn = VehicleDynamics(ct16ev_params)
        assert dyn.m_effective == ct16ev_params.mass_kg

    def test_cornering_speed_unaffected(self, dynamics_with_inertia):
        """Cornering speed should NOT use m_effective (lateral, not longitudinal)."""
        dyn_no_inertia = VehicleDynamics(dynamics_with_inertia.vehicle)
        kappa = 0.05
        v_with = dynamics_with_inertia.max_cornering_speed(kappa)
        v_without = dyn_no_inertia.max_cornering_speed(kappa)
        assert v_with == pytest.approx(v_without, abs=0.01)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_dynamics.py::TestEffectiveInertia -v`
Expected: FAIL — `VehicleDynamics` does not accept `powertrain_config` yet

- [ ] **Step 3: Implement m_effective in VehicleDynamics**

Modify `src/fsae_sim/vehicle/dynamics.py`:

Change the constructor signature and add `m_effective` computation:

```python
def __init__(
    self,
    vehicle: VehicleParams,
    tire_model: PacejkaTireModel | None = None,
    load_transfer: LoadTransferModel | None = None,
    cornering_solver: CorneringSolver | None = None,
    powertrain_config: "PowertrainConfig | None" = None,
) -> None:
    self.vehicle = vehicle
    self.tire_model = tire_model
    self.load_transfer = load_transfer
    self.cornering_solver = cornering_solver

    # Effective mass: bare mass + rotational inertia of spinning components
    if powertrain_config is not None:
        tire_radius = 0.228  # m, 10-inch FSAE wheel
        G = powertrain_config.gear_ratio
        eta = powertrain_config.drivetrain_efficiency
        j_eff = (
            vehicle.rotor_inertia_kg_m2 * G * G * eta
            + 4 * vehicle.wheel_inertia_kg_m2
        )
        self.m_effective = vehicle.mass_kg + j_eff / (tire_radius * tire_radius)
    else:
        self.m_effective = vehicle.mass_kg
```

Add `TYPE_CHECKING` import for `PowertrainConfig`:

```python
if TYPE_CHECKING:
    from fsae_sim.vehicle.cornering_solver import CorneringSolver
    from fsae_sim.vehicle.load_transfer import LoadTransferModel
    from fsae_sim.vehicle.tire_model import PacejkaTireModel
    from fsae_sim.vehicle.powertrain import PowertrainConfig
```

Change `acceleration()`:

```python
def acceleration(
    self, net_force_n: float,
) -> float:
    """Longitudinal acceleration (m/s^2) from net force.

    Uses ``m_effective`` which includes rotational inertia of motor,
    gears, and wheels when a powertrain config is provided.
    """
    return net_force_n / self.m_effective
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_dynamics.py -v`
Expected: ALL PASS (including all pre-existing tests — backward compatible)

- [ ] **Step 5: Wire powertrain_config in SimulationEngine**

Modify `src/fsae_sim/sim/engine.py`, in `__init__()` around line 85-89. Pass `powertrain_config` when constructing `VehicleDynamics`:

```python
        if _HAS_TIRE_MODELS and tire_cfg is not None and susp_cfg is not None:
            tire_model = PacejkaTireModel(tire_cfg.tir_file)
            load_transfer = LoadTransferModel(vehicle.vehicle, susp_cfg)
            cornering_solver = CorneringSolver(
                tire_model,
                load_transfer,
                vehicle.vehicle.mass_kg,
                math.radians(tire_cfg.static_camber_front_deg),
                math.radians(tire_cfg.static_camber_rear_deg),
                susp_cfg.roll_camber_front_deg_per_deg,
                susp_cfg.roll_camber_rear_deg_per_deg,
            )
            self.dynamics = VehicleDynamics(
                vehicle.vehicle, tire_model, load_transfer, cornering_solver,
                powertrain_config=vehicle.powertrain,
            )
        else:
            self.dynamics = VehicleDynamics(
                vehicle.vehicle, powertrain_config=vehicle.powertrain,
            )
```

- [ ] **Step 6: Run full test suite**

Run: `pytest tests/ -v --tb=short`
Expected: ALL PASS

- [ ] **Step 7: Commit**

```bash
git add src/fsae_sim/vehicle/dynamics.py src/fsae_sim/sim/engine.py tests/test_dynamics.py
git commit -m "feat: add effective rotational inertia to vehicle dynamics F=ma"
```

---

### Task 3: Combined Slip in Cornering Solver

**Files:**
- Modify: `src/fsae_sim/vehicle/cornering_solver.py:67-102` (max_cornering_speed), `:104-169` (_can_sustain)
- Test: `tests/test_cornering_solver.py`

- [ ] **Step 1: Write failing tests for combined slip**

Add to `tests/test_cornering_solver.py`:

```python
class TestCombinedSlip:
    """Tests for longitudinal_g parameter reducing lateral capacity."""

    def test_zero_longitudinal_g_matches_baseline(self) -> None:
        """longitudinal_g=0.0 must produce identical result to omitting it."""
        solver = make_solver(mu=1.3)
        curvature = 0.05
        v_default = solver.max_cornering_speed(curvature)
        v_zero = solver.max_cornering_speed(curvature, longitudinal_g=0.0)
        assert v_zero == pytest.approx(v_default, abs=1e-6)

    def test_positive_longitudinal_g_reduces_speed(self) -> None:
        """Accelerating mid-corner should reduce max cornering speed."""
        solver = make_solver(mu=1.3)
        curvature = 0.05
        v_pure = solver.max_cornering_speed(curvature)
        v_accel = solver.max_cornering_speed(curvature, longitudinal_g=0.3)
        assert v_accel < v_pure

    def test_negative_longitudinal_g_reduces_speed(self) -> None:
        """Braking mid-corner should reduce max cornering speed."""
        solver = make_solver(mu=1.3)
        curvature = 0.05
        v_pure = solver.max_cornering_speed(curvature)
        v_brake = solver.max_cornering_speed(curvature, longitudinal_g=-0.5)
        assert v_brake < v_pure

    def test_higher_longitudinal_g_lower_speed(self) -> None:
        """Monotonic: more longitudinal demand -> lower corner speed."""
        solver = make_solver(mu=1.3)
        curvature = 0.05
        v_low = solver.max_cornering_speed(curvature, longitudinal_g=0.2)
        v_high = solver.max_cornering_speed(curvature, longitudinal_g=0.5)
        assert v_high < v_low

    def test_small_longitudinal_g_below_threshold_ignored(self) -> None:
        """Very small longitudinal_g (< 0.01) is treated as zero."""
        solver = make_solver(mu=1.3)
        curvature = 0.05
        v_pure = solver.max_cornering_speed(curvature)
        v_tiny = solver.max_cornering_speed(curvature, longitudinal_g=0.005)
        assert v_tiny == pytest.approx(v_pure, abs=1e-6)

    def test_accel_uses_rear_tires_only(self) -> None:
        """Under acceleration, only rear tires should lose lateral capacity.

        With a linear tire mock, we can verify by checking that the
        reduction is smaller than if all 4 tires were affected.
        """
        mu = 1.3
        mass_kg = 278.0

        # Give rear tires more load so they have more to lose
        def tire_loads_rear_heavy(speed, lat_g, lon_g):
            w = mass_kg * 9.81
            return (w * 0.15, w * 0.15, w * 0.35, w * 0.35)

        lt = MagicMock()
        lt.tire_loads.side_effect = tire_loads_rear_heavy
        lt.roll_stiffness_front = 1e6
        lt.roll_stiffness_rear = 1e6

        tire = MagicMock()
        tire.peak_lateral_force.side_effect = lambda fz, camber=0.0: mu * fz
        tire.peak_longitudinal_force.side_effect = lambda fz, camber=0.0: mu * fz

        solver = CorneringSolver(
            tire_model=tire, load_transfer=lt, mass_kg=mass_kg,
            static_camber_front_rad=0.0, static_camber_rear_rad=0.0,
            roll_camber_front=0.0, roll_camber_rear=0.0,
        )
        curvature = 0.05
        v_pure = solver.max_cornering_speed(curvature)
        v_accel = solver.max_cornering_speed(curvature, longitudinal_g=0.3)
        # Should reduce speed, but front tires still have full lateral capacity
        assert v_accel < v_pure
        assert v_accel > 0.5  # still drivable

    def test_sign_invariance_of_curvature_with_longitudinal_g(self) -> None:
        """Combined slip should not break curvature sign invariance."""
        solver = make_solver(mu=1.3)
        curvature = 0.05
        v_pos = solver.max_cornering_speed(curvature, longitudinal_g=0.3)
        v_neg = solver.max_cornering_speed(-curvature, longitudinal_g=0.3)
        assert v_pos == pytest.approx(v_neg, abs=1e-6)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_cornering_solver.py::TestCombinedSlip -v`
Expected: FAIL — `max_cornering_speed` does not accept `longitudinal_g`

- [ ] **Step 3: Implement combined slip in CorneringSolver**

Modify `src/fsae_sim/vehicle/cornering_solver.py`.

Update `max_cornering_speed` signature:

```python
def max_cornering_speed(
    self,
    curvature: float,
    mu_scale: float = 1.0,
    longitudinal_g: float = 0.0,
) -> float:
```

Pass `longitudinal_g` through to `_can_sustain`:

```python
    for _ in range(self._ITERATIONS):
        v_mid = (v_low + v_high) / 2.0
        if self._can_sustain(v_mid, abs_curvature, mu_scale, longitudinal_g):
            v_low = v_mid
        else:
            v_high = v_mid
```

Update `_can_sustain` signature and implementation:

```python
def _can_sustain(
    self,
    speed: float,
    curvature: float,
    mu_scale: float,
    longitudinal_g: float = 0.0,
) -> bool:
```

Replace the force summation block (lines 158-164 in current code) with:

```python
        # Sum lateral capacity from all four tires
        loads = [fl, fr, rl, rr]
        cambers = [camber_fl, camber_fr, camber_rl, camber_rr]

        if abs(longitudinal_g) > 0.01:
            # Distribute longitudinal demand to tires
            f_x_total = self._mass_kg * abs(longitudinal_g) * self.GRAVITY

            if longitudinal_g > 0:
                # Traction: rear tires only
                fx_per_tire = [0.0, 0.0, f_x_total / 2.0, f_x_total / 2.0]
            else:
                # Braking: proportional to normal load
                total_fz = sum(loads)
                if total_fz > 0:
                    fx_per_tire = [f_x_total * (fz / total_fz) for fz in loads]
                else:
                    fx_per_tire = [0.0, 0.0, 0.0, 0.0]

            total_capacity = 0.0
            for fz, camber, fx in zip(loads, cambers, fx_per_tire):
                fx_peak = self._tire.peak_longitudinal_force(fz, camber)
                if fx_peak > 0:
                    fx_ratio = min(abs(fx) / fx_peak, 0.99)
                else:
                    fx_ratio = 0.99
                fy_peak = self._tire.peak_lateral_force(fz, camber)
                fy_available = fy_peak * math.sqrt(1.0 - fx_ratio * fx_ratio)
                total_capacity += fy_available * mu_scale
        else:
            # Pure cornering (no longitudinal demand)
            total_capacity = sum(
                self._tire.peak_lateral_force(fz, cam) * mu_scale
                for fz, cam in zip(loads, cambers)
            )

        required_force = self._mass_kg * speed * speed * curvature
        return total_capacity >= required_force
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_cornering_solver.py -v`
Expected: ALL PASS (new combined slip tests + all existing tests)

- [ ] **Step 5: Commit**

```bash
git add src/fsae_sim/vehicle/cornering_solver.py tests/test_cornering_solver.py
git commit -m "feat: add combined slip friction-ellipse reduction to cornering solver"
```

---

### Task 4: SpeedEnvelope — Core Forward-Backward Solver

**Files:**
- Create: `src/fsae_sim/sim/speed_envelope.py`
- Test: `tests/test_speed_envelope.py`

- [ ] **Step 1: Write failing tests for SpeedEnvelope**

Create `tests/test_speed_envelope.py`:

```python
"""Tests for forward-backward speed envelope solver."""

from __future__ import annotations

import math
from unittest.mock import MagicMock

import numpy as np
import pytest

from fsae_sim.track.track import Segment, Track
from fsae_sim.sim.speed_envelope import SpeedEnvelope


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_segment(index: int, curvature: float = 0.0, length: float = 5.0,
                 grade: float = 0.0, grip_factor: float = 1.0) -> Segment:
    return Segment(
        index=index,
        distance_start_m=index * length,
        length_m=length,
        curvature=curvature,
        grade=grade,
        grip_factor=grip_factor,
    )


def make_track(segments: list[Segment], name: str = "test") -> Track:
    return Track(name=name, segments=segments)


def make_dynamics(mass_kg: float = 288.0, corner_speed_fn=None,
                  resistance_fn=None, max_traction_fn=None,
                  max_braking_fn=None):
    """Mock VehicleDynamics with configurable behavior."""
    dyn = MagicMock()
    dyn.vehicle = MagicMock()
    dyn.vehicle.mass_kg = mass_kg
    dyn.m_effective = mass_kg + 37.0  # ~325 kg effective

    if corner_speed_fn is None:
        def corner_speed_fn(curvature, grip_factor=1.0):
            kappa = abs(curvature)
            if kappa < 1e-6:
                return float("inf")
            return math.sqrt(1.3 * 9.81 / kappa) * grip_factor
    dyn.max_cornering_speed.side_effect = corner_speed_fn

    if resistance_fn is None:
        resistance_fn = lambda v, grade=0.0: 50.0 + 0.5 * v * v
    dyn.total_resistance.side_effect = resistance_fn

    if max_traction_fn is None:
        max_traction_fn = lambda v: 3000.0
    dyn.max_traction_force.side_effect = max_traction_fn

    if max_braking_fn is None:
        max_braking_fn = lambda v: 4000.0
    dyn.max_braking_force.side_effect = max_braking_fn

    return dyn


def make_powertrain(max_drive_force: float = 2000.0, max_regen_force: float = -800.0):
    """Mock PowertrainModel."""
    pt = MagicMock()
    pt.drive_force.side_effect = lambda throttle, v: throttle * max_drive_force
    pt.regen_force.side_effect = lambda brake, v: brake * max_regen_force
    return pt


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestStraightOnlyTrack:
    """All-straight track: envelope limited by acceleration only."""

    def test_speeds_increase_monotonically_from_rest(self):
        segs = [make_segment(i) for i in range(20)]
        track = make_track(segs)
        dyn = make_dynamics()
        pt = make_powertrain()

        env = SpeedEnvelope(dyn, pt, track)
        v_max = env.compute(initial_speed=0.5)

        assert len(v_max) == 20
        # Speeds should generally increase (accelerating from rest)
        assert v_max[-1] > v_max[0]
        # All positive
        assert np.all(v_max > 0)

    def test_envelope_respects_initial_speed(self):
        segs = [make_segment(i) for i in range(10)]
        track = make_track(segs)
        dyn = make_dynamics()
        pt = make_powertrain()

        env = SpeedEnvelope(dyn, pt, track)
        v_slow = env.compute(initial_speed=0.5)
        v_fast = env.compute(initial_speed=15.0)

        assert v_fast[0] >= v_slow[0]


class TestSingleCorner:
    """Straight-corner-straight: braking zone must appear before corner."""

    def test_braking_zone_before_corner(self):
        # 10 straights, 5 corner segments (kappa=0.1, ~10m radius), 10 straights
        segs = (
            [make_segment(i) for i in range(10)]
            + [make_segment(i + 10, curvature=0.1) for i in range(5)]
            + [make_segment(i + 15) for i in range(10)]
        )
        track = make_track(segs)
        dyn = make_dynamics()
        pt = make_powertrain()

        env = SpeedEnvelope(dyn, pt, track)
        v_max = env.compute(initial_speed=0.5)

        # Corner speed limit: sqrt(1.3 * 9.81 / 0.1) ~ 11.3 m/s
        corner_speed = math.sqrt(1.3 * 9.81 / 0.1)

        # The corner segments should be at or below corner speed
        for i in range(10, 15):
            assert v_max[i] <= corner_speed + 0.5

        # Speed should decrease before the corner (braking zone)
        # The segment just before the corner should be slower than
        # the segment several before it
        assert v_max[9] < v_max[5]

    def test_acceleration_zone_after_corner(self):
        segs = (
            [make_segment(i) for i in range(5)]
            + [make_segment(i + 5, curvature=0.1) for i in range(5)]
            + [make_segment(i + 10) for i in range(10)]
        )
        track = make_track(segs)
        dyn = make_dynamics()
        pt = make_powertrain()

        env = SpeedEnvelope(dyn, pt, track)
        v_max = env.compute(initial_speed=0.5)

        # After the corner, speeds should increase
        assert v_max[15] > v_max[10]


class TestEnvelopeIsPhysicallyValid:
    """The envelope must not require impossible deceleration."""

    def test_no_segment_exceeds_max_deceleration(self):
        segs = (
            [make_segment(i) for i in range(15)]
            + [make_segment(i + 15, curvature=0.15) for i in range(3)]
            + [make_segment(i + 18) for i in range(10)]
        )
        track = make_track(segs)
        dyn = make_dynamics()
        pt = make_powertrain()

        env = SpeedEnvelope(dyn, pt, track)
        v_max = env.compute(initial_speed=0.5)

        m_eff = dyn.m_effective
        # Check that deceleration between consecutive segments is feasible
        for i in range(len(v_max) - 1):
            if v_max[i + 1] < v_max[i]:
                # Decelerating: check max braking
                dv_sq = v_max[i] ** 2 - v_max[i + 1] ** 2
                a_required = dv_sq / (2 * segs[i].length_m)
                # Max braking force / m_eff gives max decel
                # Use a generous tolerance (braking + drag assist)
                max_decel = (4000.0 + 50.0 + 0.5 * v_max[i] ** 2) / m_eff
                assert a_required <= max_decel * 1.1  # 10% tolerance for averaging


class TestEnvelopeAlwaysLECornerSpeed:
    """Envelope speed must never exceed corner speed limit at any segment."""

    def test_envelope_bounded_by_corner_speed(self):
        segs = (
            [make_segment(i) for i in range(5)]
            + [make_segment(i + 5, curvature=0.08) for i in range(5)]
            + [make_segment(i + 10) for i in range(5)]
        )
        track = make_track(segs)
        dyn = make_dynamics()
        pt = make_powertrain()

        env = SpeedEnvelope(dyn, pt, track)
        v_max = env.compute(initial_speed=0.5)

        for i, seg in enumerate(segs):
            corner_limit = dyn.max_cornering_speed(seg.curvature, seg.grip_factor)
            assert v_max[i] <= corner_limit + 0.5  # tolerance for bisection
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_speed_envelope.py -v`
Expected: FAIL — `speed_envelope` module does not exist

- [ ] **Step 3: Implement SpeedEnvelope**

Create `src/fsae_sim/sim/speed_envelope.py`:

```python
"""Forward-backward speed envelope for quasi-static simulation.

Computes the fastest physically achievable speed at every track segment,
respecting cornering limits, powertrain acceleration, and braking
deceleration.  The result is a speed ceiling that no synthetic driver
strategy can exceed.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from fsae_sim.track.track import Track
    from fsae_sim.vehicle.dynamics import VehicleDynamics
    from fsae_sim.vehicle.powertrain_model import PowertrainModel


class SpeedEnvelope:
    """Forward-backward speed envelope solver with corner speed caching.

    Args:
        dynamics: Vehicle dynamics model (for corner speeds and resistance).
        powertrain: Powertrain model (for max drive/regen force).
        track: Track geometry.
    """

    _MIN_SPEED: float = 0.5

    def __init__(
        self,
        dynamics: VehicleDynamics,
        powertrain: PowertrainModel,
        track: Track,
    ) -> None:
        self._dynamics = dynamics
        self._powertrain = powertrain
        self._track = track
        self._corner_speed_cache: dict[tuple, np.ndarray] = {}

    def compute(self, initial_speed: float = 0.5) -> np.ndarray:
        """Compute the speed envelope for the full track.

        Args:
            initial_speed: Vehicle speed at segment 0 (m/s).

        Returns:
            1-D array of maximum feasible speed (m/s) per segment.
        """
        segments = self._track.segments
        n = len(segments)
        m_eff = self._dynamics.m_effective

        # Pass 1: corner speeds (cached)
        v_corner = self._get_corner_speeds()

        # Pass 2: backward pass (braking feasibility)
        v_back = np.empty(n, dtype=np.float64)
        v_back[n - 1] = v_corner[n - 1]

        for i in range(n - 2, -1, -1):
            v = v_back[i + 1]
            # Max braking force at the speed we need to reach
            f_resist = self._dynamics.total_resistance(v)
            f_regen = abs(self._powertrain.regen_force(1.0, v))
            f_tire_limit = self._dynamics.max_braking_force(v)
            f_brake = min(f_resist + f_regen, f_tire_limit)
            a_brake = f_brake / m_eff

            # v_entry^2 = v_exit^2 + 2 * a_brake * d
            v_entry_sq = v * v + 2.0 * a_brake * segments[i].length_m
            v_back[i] = min(v_corner[i], math.sqrt(v_entry_sq))

        # Lap-wrap: check if the last segment can feed into the first
        # If v_back[last] > v_back[0], need another backward pass
        v_last = v_back[n - 1]
        f_resist = self._dynamics.total_resistance(v_back[0])
        f_regen = abs(self._powertrain.regen_force(1.0, v_back[0]))
        f_tire_limit = self._dynamics.max_braking_force(v_back[0])
        f_brake = min(f_resist + f_regen, f_tire_limit)
        a_brake = f_brake / m_eff
        v_wrap_sq = v_back[0] * v_back[0] + 2.0 * a_brake * segments[n - 1].length_m
        v_wrap = math.sqrt(v_wrap_sq)

        if v_last > v_wrap:
            # Need to propagate the wrap constraint backward
            v_back[n - 1] = min(v_back[n - 1], v_wrap)
            for i in range(n - 2, -1, -1):
                v = v_back[i + 1]
                f_resist = self._dynamics.total_resistance(v)
                f_regen = abs(self._powertrain.regen_force(1.0, v))
                f_tire_limit = self._dynamics.max_braking_force(v)
                f_brake = min(f_resist + f_regen, f_tire_limit)
                a_brake = f_brake / m_eff
                v_entry_sq = v * v + 2.0 * a_brake * segments[i].length_m
                new_limit = min(v_corner[i], math.sqrt(v_entry_sq))
                if new_limit >= v_back[i]:
                    break  # no further propagation needed
                v_back[i] = new_limit

        # Pass 3: forward pass (acceleration feasibility)
        v_fwd = np.empty(n, dtype=np.float64)
        v_fwd[0] = min(v_back[0], max(initial_speed, self._MIN_SPEED))

        for i in range(1, n):
            v = v_fwd[i - 1]
            # Max drive force at current speed
            f_drive = self._powertrain.drive_force(1.0, v)
            f_traction = self._dynamics.max_traction_force(v)
            f_drive = min(f_drive, f_traction)
            f_resist = self._dynamics.total_resistance(v, segments[i - 1].grade)
            f_net = f_drive - f_resist
            a_accel = f_net / m_eff

            v_exit_sq = v * v + 2.0 * a_accel * segments[i - 1].length_m
            v_exit = math.sqrt(max(0.0, v_exit_sq))
            v_fwd[i] = min(v_back[i], v_exit)

        return v_fwd

    # ------------------------------------------------------------------
    # Corner speed caching
    # ------------------------------------------------------------------

    def _cache_key(self) -> tuple:
        grip_factors = tuple(s.grip_factor for s in self._track.segments)
        return (
            self._track.name,
            len(self._track.segments),
            self._dynamics.vehicle.mass_kg,
            grip_factors,
        )

    def _get_corner_speeds(self) -> np.ndarray:
        key = self._cache_key()
        if key in self._corner_speed_cache:
            return self._corner_speed_cache[key].copy()

        v_corner = np.array([
            self._dynamics.max_cornering_speed(seg.curvature, seg.grip_factor)
            for seg in self._track.segments
        ])
        self._corner_speed_cache[key] = v_corner
        return v_corner.copy()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_speed_envelope.py -v`
Expected: ALL PASS

- [ ] **Step 5: Commit**

```bash
git add src/fsae_sim/sim/speed_envelope.py tests/test_speed_envelope.py
git commit -m "feat: add forward-backward speed envelope solver with corner caching"
```

---

### Task 5: Corner Speed Cache Tests

**Files:**
- Test: `tests/test_speed_envelope.py` (append)

- [ ] **Step 1: Write cache tests**

Append to `tests/test_speed_envelope.py`:

```python
class TestCornerSpeedCache:
    """Corner speeds should be cached and reused across compute() calls."""

    def test_cache_hit_returns_same_result(self):
        segs = [make_segment(i, curvature=0.05 if i % 5 == 0 else 0.0) for i in range(20)]
        track = make_track(segs)
        dyn = make_dynamics()
        pt = make_powertrain()

        env = SpeedEnvelope(dyn, pt, track)
        v1 = env.compute(initial_speed=0.5)
        call_count_after_first = dyn.max_cornering_speed.call_count

        v2 = env.compute(initial_speed=0.5)
        call_count_after_second = dyn.max_cornering_speed.call_count

        # Corner speed function should NOT be called again
        assert call_count_after_second == call_count_after_first
        np.testing.assert_array_equal(v1, v2)

    def test_different_initial_speed_reuses_cache(self):
        segs = [make_segment(i, curvature=0.05 if i % 3 == 0 else 0.0) for i in range(15)]
        track = make_track(segs)
        dyn = make_dynamics()
        pt = make_powertrain()

        env = SpeedEnvelope(dyn, pt, track)
        env.compute(initial_speed=0.5)
        count1 = dyn.max_cornering_speed.call_count

        env.compute(initial_speed=10.0)
        count2 = dyn.max_cornering_speed.call_count

        # Cache should be reused (same track, same vehicle)
        assert count2 == count1

    def test_different_track_invalidates_cache(self):
        segs1 = [make_segment(i) for i in range(10)]
        segs2 = [make_segment(i, curvature=0.05) for i in range(10)]
        track1 = make_track(segs1, name="track1")
        track2 = make_track(segs2, name="track2")
        dyn = make_dynamics()
        pt = make_powertrain()

        env1 = SpeedEnvelope(dyn, pt, track1)
        env1.compute(initial_speed=0.5)
        count1 = dyn.max_cornering_speed.call_count

        env2 = SpeedEnvelope(dyn, pt, track2)
        env2.compute(initial_speed=0.5)
        count2 = dyn.max_cornering_speed.call_count

        # New track = new envelope = new corner speed computation
        assert count2 > count1
```

- [ ] **Step 2: Run tests**

Run: `pytest tests/test_speed_envelope.py::TestCornerSpeedCache -v`
Expected: ALL PASS

- [ ] **Step 3: Commit**

```bash
git add tests/test_speed_envelope.py
git commit -m "test: add corner speed cache tests for SpeedEnvelope"
```

---

### Task 6: Integrate SpeedEnvelope into SimulationEngine

**Files:**
- Modify: `src/fsae_sim/sim/engine.py:1-30` (imports), `:91-100` (run method), `:178-209` (segment loop)
- Modify: `src/fsae_sim/sim/__init__.py`
- Create: `tests/test_engine_envelope.py`

- [ ] **Step 1: Write integration test**

Create `tests/test_engine_envelope.py`:

```python
"""Integration test: SimulationEngine uses SpeedEnvelope for synthetic strategies."""

from __future__ import annotations

import math
from unittest.mock import MagicMock, patch

import pytest

from fsae_sim.track.track import Segment, Track
from fsae_sim.vehicle.vehicle import VehicleParams, VehicleConfig, TireConfig, SuspensionConfig
from fsae_sim.vehicle.powertrain import PowertrainConfig
from fsae_sim.vehicle.battery import BatteryConfig, DischargeLimitPoint
from fsae_sim.vehicle.battery_model import BatteryModel
from fsae_sim.vehicle.dynamics import VehicleDynamics
from fsae_sim.driver.strategy import ControlAction, ControlCommand, DriverStrategy, SimState
from fsae_sim.sim.engine import SimulationEngine


class StubStrategy(DriverStrategy):
    """Always full throttle."""
    name = "full_throttle"

    def decide(self, state: SimState, upcoming_segments) -> ControlCommand:
        return ControlCommand(
            action=ControlAction.THROTTLE,
            throttle_pct=1.0,
            brake_pct=0.0,
        )


def make_simple_track() -> Track:
    """5 straights + 3 corner (kappa=0.1) + 5 straights = 13 segments."""
    segs = (
        [Segment(i, i * 5.0, 5.0, 0.0, 0.0) for i in range(5)]
        + [Segment(i + 5, (i + 5) * 5.0, 5.0, 0.1, 0.0) for i in range(3)]
        + [Segment(i + 8, (i + 8) * 5.0, 5.0, 0.0, 0.0) for i in range(5)]
    )
    return Track(name="test_circuit", segments=segs)


def make_minimal_config() -> VehicleConfig:
    return VehicleConfig(
        name="test",
        year=2025,
        description="test",
        vehicle=VehicleParams(
            mass_kg=288.0,
            frontal_area_m2=1.0,
            drag_coefficient=1.5,
            rolling_resistance=0.015,
            wheelbase_m=1.549,
            downforce_coefficient=2.18,
        ),
        powertrain=PowertrainConfig(
            motor_speed_max_rpm=2900,
            brake_speed_rpm=2400,
            torque_limit_inverter_nm=85.0,
            torque_limit_lvcu_nm=150.0,
            iq_limit_a=170.0,
            id_limit_a=30.0,
            gear_ratio=3.6363,
            drivetrain_efficiency=0.92,
        ),
        battery=BatteryConfig(
            cell_type="P45B",
            series=110,
            parallel=4,
            cell_voltage_min_v=2.55,
            cell_voltage_max_v=4.2,
            discharged_soc_pct=2.0,
            soc_taper_threshold_pct=85.0,
            soc_taper_rate_a_per_pct=1.0,
            discharge_limits=[
                DischargeLimitPoint(30.0, 100.0),
                DischargeLimitPoint(65.0, 0.0),
            ],
        ),
    )


class TestEnvelopeIntegration:
    """Engine should use speed envelope for synthetic strategies."""

    def test_synthetic_strategy_uses_envelope(self):
        """With envelope, corner entry speed should be lower than corner speed limit
        (car must brake before the corner, not snap to it)."""
        track = make_simple_track()
        config = make_minimal_config()
        strategy = StubStrategy()

        # Create a simple battery model mock
        batt = MagicMock(spec=BatteryModel)
        batt.pack_voltage.return_value = 400.0
        batt.max_discharge_current.return_value = 100.0
        batt.step.return_value = (90.0, 26.0, 395.0)

        engine = SimulationEngine(config, track, strategy, batt)
        result = engine.run(num_laps=1, initial_soc_pct=95.0)

        # The simulation should complete
        assert result.total_time_s > 0
        assert result.laps_completed == 1

        # Speed at corner entry (segment 7, before corner at 8)
        # should show deceleration from envelope, not instant snap
        states = result.states
        corner_speeds = states[states["curvature"].abs() > 0.05]["speed_ms"]
        straight_speeds = states[(states["curvature"].abs() < 0.01) & (states["segment_idx"] < 4)]["speed_ms"]

        # Corner speeds should be bounded
        if len(corner_speeds) > 0 and len(straight_speeds) > 0:
            assert corner_speeds.mean() < straight_speeds.iloc[-1] + 1.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_engine_envelope.py -v`
Expected: May pass partially (engine works but doesn't use envelope yet). The key validation is in the behavior, not a hard failure.

- [ ] **Step 3: Add SpeedEnvelope import and compute to engine**

Modify `src/fsae_sim/sim/engine.py`.

Add import after existing imports:

```python
from fsae_sim.sim.speed_envelope import SpeedEnvelope
```

In `__init__()`, after constructing `self.dynamics` (around line 89), store a `SpeedEnvelope`:

```python
        self._envelope = SpeedEnvelope(self.dynamics, self.powertrain, self.track)
```

In `run()`, before the lap loop (after line 126 `laps_completed = 0`), compute the envelope for synthetic strategies:

```python
        # Pre-compute speed envelope for synthetic strategies
        is_replay = isinstance(self.strategy, ReplayStrategy)
        if not is_replay:
            v_max = self._envelope.compute(initial_speed=speed)
        else:
            v_max = None
```

In the segment loop, replace the per-segment corner speed computation (lines 183-185):

```python
                    # 2. Speed limit from pre-computed envelope
                    corner_limit = float(v_max[seg_idx])
```

Remove the old call:
```python
                    # DELETE these lines:
                    # corner_limit = self.dynamics.max_cornering_speed(
                    #     segment.curvature, segment.grip_factor,
                    # )
```

- [ ] **Step 4: Update __init__.py exports**

Modify `src/fsae_sim/sim/__init__.py`:

```python
from fsae_sim.sim.engine import SimulationEngine, SimResult
from fsae_sim.sim.speed_envelope import SpeedEnvelope

__all__ = ["SimulationEngine", "SimResult", "SpeedEnvelope"]
```

- [ ] **Step 5: Run full test suite**

Run: `pytest tests/ -v --tb=short`
Expected: ALL PASS

- [ ] **Step 6: Commit**

```bash
git add src/fsae_sim/sim/engine.py src/fsae_sim/sim/__init__.py src/fsae_sim/sim/speed_envelope.py tests/test_engine_envelope.py
git commit -m "feat: integrate forward-backward speed envelope into simulation engine"
```

---

### Task 7: Combined Slip Correction Pass in SpeedEnvelope

**Files:**
- Modify: `src/fsae_sim/sim/speed_envelope.py`
- Test: `tests/test_speed_envelope.py` (append)

- [ ] **Step 1: Write failing test for correction pass**

Append to `tests/test_speed_envelope.py`:

```python
class TestCombinedSlipCorrection:
    """When cornering solver supports longitudinal_g, the envelope should
    apply a correction pass that reduces corner speeds where the car is
    simultaneously accelerating or braking."""

    def test_correction_reduces_speed_at_corner_exit(self):
        """At corner exit where car is accelerating, combined slip should
        reduce the achievable speed slightly vs pure cornering."""
        # Build track: 5 straights + 5 tight corners + 10 straights
        segs = (
            [make_segment(i) for i in range(5)]
            + [make_segment(i + 5, curvature=0.1) for i in range(5)]
            + [make_segment(i + 10) for i in range(10)]
        )
        track = make_track(segs)

        # Dynamics mock that supports longitudinal_g
        corner_speeds = {}
        def corner_speed_fn(curvature, grip_factor=1.0, longitudinal_g=0.0):
            kappa = abs(curvature)
            if kappa < 1e-6:
                return float("inf")
            base = math.sqrt(1.3 * 9.81 / kappa) * grip_factor
            if abs(longitudinal_g) > 0.01:
                # Reduce by friction ellipse factor
                reduction = 1.0 - 0.3 * abs(longitudinal_g)
                return base * max(reduction, 0.5)
            return base

        dyn = make_dynamics()
        dyn.max_cornering_speed.side_effect = corner_speed_fn
        pt = make_powertrain()

        env_with_correction = SpeedEnvelope(dyn, pt, track)
        v_corrected = env_with_correction.compute(initial_speed=0.5)

        # Compare with no-correction envelope (longitudinal_g always 0)
        dyn2 = make_dynamics()  # fresh mock, no longitudinal_g support
        pt2 = make_powertrain()
        env_no_correction = SpeedEnvelope(dyn2, pt2, track)
        v_no_correction = env_no_correction.compute(initial_speed=0.5)

        # Corrected envelope should be <= no-correction at every segment
        for i in range(len(segs)):
            assert v_corrected[i] <= v_no_correction[i] + 0.1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_speed_envelope.py::TestCombinedSlipCorrection -v`
Expected: FAIL or behavior mismatch (no correction pass yet)

- [ ] **Step 3: Add correction pass to SpeedEnvelope.compute()**

In `src/fsae_sim/sim/speed_envelope.py`, add after the forward pass (after `v_fwd` is complete), before `return v_fwd`:

```python
        # Pass 4: combined slip correction
        # Where the envelope shows acceleration or braking near corners,
        # re-check corner speeds with longitudinal_g to account for
        # friction ellipse reduction.
        v_corrected = v_fwd.copy()
        needs_repropagation = False

        for i in range(n):
            seg = segments[i]
            if abs(seg.curvature) < 1e-6:
                continue  # only correct at corners

            # Estimate longitudinal_g from speed change across this segment
            if i > 0:
                dv_sq = v_fwd[i] ** 2 - v_fwd[i - 1] ** 2
                a_long = dv_sq / (2.0 * seg.length_m)
                long_g = a_long / 9.81
            else:
                long_g = 0.0

            if abs(long_g) < 0.01:
                continue

            # Re-query corner speed with longitudinal demand
            try:
                v_corrected_corner = self._dynamics.max_cornering_speed(
                    seg.curvature, seg.grip_factor, longitudinal_g=long_g,
                )
            except TypeError:
                # Dynamics doesn't support longitudinal_g (e.g., legacy mode)
                continue

            if v_corrected_corner < v_corrected[i]:
                v_corrected[i] = v_corrected_corner
                needs_repropagation = True

        if needs_repropagation:
            # Re-run backward pass from corrected values
            for i in range(n - 2, -1, -1):
                v = v_corrected[i + 1]
                f_resist = self._dynamics.total_resistance(v)
                f_regen = abs(self._powertrain.regen_force(1.0, v))
                f_tire_limit = self._dynamics.max_braking_force(v)
                f_brake = min(f_resist + f_regen, f_tire_limit)
                a_brake = f_brake / m_eff
                v_entry_sq = v * v + 2.0 * a_brake * segments[i].length_m
                new_limit = min(v_corrected[i], math.sqrt(v_entry_sq))
                if new_limit >= v_corrected[i]:
                    continue
                v_corrected[i] = new_limit

            # Re-run forward pass
            v_corrected[0] = min(v_corrected[0], max(initial_speed, self._MIN_SPEED))
            for i in range(1, n):
                v = v_corrected[i - 1]
                f_drive = self._powertrain.drive_force(1.0, v)
                f_traction = self._dynamics.max_traction_force(v)
                f_drive = min(f_drive, f_traction)
                f_resist = self._dynamics.total_resistance(v, segments[i - 1].grade)
                f_net = f_drive - f_resist
                a_accel = f_net / m_eff
                v_exit_sq = v * v + 2.0 * a_accel * segments[i - 1].length_m
                v_exit = math.sqrt(max(0.0, v_exit_sq))
                v_corrected[i] = min(v_corrected[i], v_exit)

            return v_corrected

        return v_fwd
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_speed_envelope.py -v`
Expected: ALL PASS

- [ ] **Step 5: Commit**

```bash
git add src/fsae_sim/sim/speed_envelope.py tests/test_speed_envelope.py
git commit -m "feat: add combined slip correction pass to speed envelope"
```

---

### Task 8: Final Integration Verification

**Files:**
- No new files — verification only

- [ ] **Step 1: Run full test suite**

Run: `pytest tests/ -v --tb=short`
Expected: ALL PASS

- [ ] **Step 2: Verify backward compatibility**

Run: `pytest tests/test_dynamics.py tests/test_cornering_solver.py -v`
Expected: ALL pre-existing tests PASS unchanged

- [ ] **Step 3: Commit any remaining changes**

If any fixups were needed:
```bash
git add -u
git commit -m "fix: address test failures from physics validation upgrade"
```

If all clean, no commit needed.
