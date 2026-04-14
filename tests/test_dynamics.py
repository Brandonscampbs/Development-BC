"""Tests for vehicle dynamics force-balance model."""

import math

import pytest

from fsae_sim.vehicle.vehicle import VehicleParams
from fsae_sim.vehicle.dynamics import VehicleDynamics


@pytest.fixture
def ct16ev_params():
    """CT-16EV vehicle parameters (DSS values)."""
    return VehicleParams(
        mass_kg=278.0,
        frontal_area_m2=1.0,
        drag_coefficient=1.502,
        rolling_resistance=0.015,
        wheelbase_m=1.549,
        downforce_coefficient=2.18,
    )


@pytest.fixture
def dynamics(ct16ev_params):
    return VehicleDynamics(ct16ev_params)


class TestDragForce:

    def test_zero_speed_zero_drag(self, dynamics):
        assert dynamics.drag_force(0.0) == 0.0

    def test_drag_positive(self, dynamics):
        assert dynamics.drag_force(10.0) > 0.0

    def test_drag_scales_with_v_squared(self, dynamics):
        f10 = dynamics.drag_force(10.0)
        f20 = dynamics.drag_force(20.0)
        assert abs(f20 / f10 - 4.0) < 0.01

    def test_drag_at_40kmh(self, dynamics):
        """40 km/h = 11.11 m/s. F = 0.5*1.225*1.502*1.0*11.11^2 ≈ 113.6N"""
        v = 40 / 3.6
        f = dynamics.drag_force(v)
        assert 100 < f < 130


class TestRollingResistance:

    def test_rolling_resistance_positive(self, dynamics):
        assert dynamics.rolling_resistance_force() > 0.0

    def test_rolling_resistance_value(self, dynamics):
        """At 0 speed: F = 278 * 9.81 * 0.015 ≈ 40.9N"""
        f = dynamics.rolling_resistance_force(0.0)
        assert abs(f - 278 * 9.81 * 0.015) < 0.5


class TestGradeForce:

    def test_flat_grade_zero(self, dynamics):
        assert abs(dynamics.grade_force(0.0)) < 1e-6

    def test_uphill_positive(self, dynamics):
        assert dynamics.grade_force(0.05) > 0.0

    def test_downhill_negative(self, dynamics):
        assert dynamics.grade_force(-0.05) < 0.0

    def test_grade_magnitude(self, dynamics):
        """5% grade: F ≈ 278*9.81*sin(atan(0.05)) ≈ 136N"""
        f = dynamics.grade_force(0.05)
        assert 130 < f < 145


class TestTotalResistance:

    def test_at_rest_equals_rolling_resistance(self, dynamics):
        f = dynamics.total_resistance(0.0, 0.0)
        rr = dynamics.rolling_resistance_force()
        assert abs(f - rr) < 1e-6

    def test_increases_with_speed(self, dynamics):
        f10 = dynamics.total_resistance(10.0)
        f20 = dynamics.total_resistance(20.0)
        assert f20 > f10


class TestCorneringSpeed:

    def test_straight_is_infinite(self, dynamics):
        v = dynamics.max_cornering_speed(0.0)
        assert v == float("inf")

    def test_tighter_corner_slower(self, dynamics):
        v_wide = dynamics.max_cornering_speed(0.01)
        v_tight = dynamics.max_cornering_speed(0.05)
        assert v_wide > v_tight

    def test_typical_fsae_corner(self, dynamics):
        """~15m radius (κ=0.067): with downforce, higher corner speed."""
        v = dynamics.max_cornering_speed(1.0 / 15.0)
        assert 13.0 < v < 20.0  # downforce increases corner speed

    def test_grip_factor_increases_speed(self, dynamics):
        v_normal = dynamics.max_cornering_speed(0.05, grip_factor=1.0)
        v_high = dynamics.max_cornering_speed(0.05, grip_factor=1.2)
        assert v_high > v_normal


class TestAcceleration:

    def test_positive_force_positive_accel(self, dynamics):
        a = dynamics.acceleration(1000.0)
        assert a > 0.0

    def test_f_equals_ma(self, dynamics):
        """1000N on 278kg → 3.60 m/s²"""
        a = dynamics.acceleration(1000.0)
        assert abs(a - 1000.0 / 278.0) < 0.01


class TestResolveExitSpeed:

    def test_constant_speed_no_force(self, dynamics):
        """Zero net force: exit speed = entry speed."""
        v_exit, t = dynamics.resolve_exit_speed(10.0, 5.0, 0.0, float("inf"))
        assert abs(v_exit - 10.0) < 0.01
        assert abs(t - 0.5) < 0.01  # 5m / 10 m/s

    def test_accelerating(self, dynamics):
        v_exit, _ = dynamics.resolve_exit_speed(10.0, 50.0, 500.0, float("inf"))
        assert v_exit > 10.0

    def test_decelerating(self, dynamics):
        v_exit, _ = dynamics.resolve_exit_speed(15.0, 50.0, -300.0, float("inf"))
        assert v_exit < 15.0

    def test_corner_speed_clamp(self, dynamics):
        """Exit speed should not exceed corner limit."""
        v_exit, _ = dynamics.resolve_exit_speed(5.0, 100.0, 2000.0, 10.0)
        assert v_exit <= 10.0 + 0.01

    def test_stall_protection(self, dynamics):
        """Huge braking force shouldn't produce NaN/negative speed."""
        v_exit, _ = dynamics.resolve_exit_speed(2.0, 5.0, -5000.0, float("inf"))
        assert v_exit >= 0.0
        assert math.isfinite(v_exit)


# ---------------------------------------------------------------------------
# Integration layer tests  (Tasks 16-17)
# ---------------------------------------------------------------------------

from unittest.mock import MagicMock


class TestLegacyBackwardCompat:
    """Verify the legacy single-arg constructor still works."""

    def test_legacy_constructor(self, ct16ev_params):
        dyn = VehicleDynamics(ct16ev_params)
        assert dyn.vehicle is ct16ev_params

    def test_legacy_cornering(self, ct16ev_params):
        dyn = VehicleDynamics(ct16ev_params)
        assert 13.0 < dyn.max_cornering_speed(1.0 / 15.0) < 20.0


class TestCorneringSolverDelegation:
    """Verify max_cornering_speed delegates to solver when present."""

    def test_delegates_to_solver(self, ct16ev_params):
        solver = MagicMock()
        solver.max_cornering_speed.return_value = 12.5
        dyn = VehicleDynamics(ct16ev_params, cornering_solver=solver)
        assert dyn.max_cornering_speed(0.05, grip_factor=0.9) == 12.5
        solver.max_cornering_speed.assert_called_once_with(0.05, mu_scale=0.9)

    def test_straight_skips_solver(self, ct16ev_params):
        solver = MagicMock()
        dyn = VehicleDynamics(ct16ev_params, cornering_solver=solver)
        assert dyn.max_cornering_speed(0.0) == float("inf")
        solver.max_cornering_speed.assert_not_called()

    def test_none_solver_legacy(self, ct16ev_params):
        dyn = VehicleDynamics(ct16ev_params, cornering_solver=None)
        assert 13.0 < dyn.max_cornering_speed(1.0 / 15.0) < 20.0


class TestMaxTractionForce:
    """Verify max_traction_force behavior in legacy and model modes."""

    def test_legacy_inf(self, ct16ev_params):
        assert VehicleDynamics(ct16ev_params).max_traction_force(10.0) == float("inf")

    def test_with_models(self, ct16ev_params):
        tire = MagicMock()
        tire.peak_longitudinal_force.return_value = 1200.0
        lt = MagicMock()
        lt.tire_loads.return_value = (500, 500, 900, 900)
        dyn = VehicleDynamics(ct16ev_params, tire_model=tire, load_transfer=lt)
        assert dyn.max_traction_force(10.0) == pytest.approx(2400.0)

    def test_uses_rear_loads(self, ct16ev_params):
        tire = MagicMock()
        tire.peak_longitudinal_force.side_effect = lambda n: n
        lt = MagicMock()
        lt.tire_loads.return_value = (400, 400, 700, 800)
        dyn = VehicleDynamics(ct16ev_params, tire_model=tire, load_transfer=lt)
        assert dyn.max_traction_force(15.0) == pytest.approx(1500.0)


class TestMaxBrakingForce:
    """Verify max_braking_force behavior in legacy and model modes."""

    def test_legacy_inf(self, ct16ev_params):
        assert VehicleDynamics(ct16ev_params).max_braking_force(10.0) == float("inf")

    def test_all_four(self, ct16ev_params):
        tire = MagicMock()
        tire.peak_longitudinal_force.side_effect = lambda n: n * 1.2
        lt = MagicMock()
        lt.tire_loads.return_value = (800, 750, 500, 550)
        dyn = VehicleDynamics(ct16ev_params, tire_model=tire, load_transfer=lt)
        assert dyn.max_braking_force(20.0) == pytest.approx(3120.0)

    def test_requires_both(self, ct16ev_params):
        assert VehicleDynamics(
            ct16ev_params, tire_model=MagicMock()
        ).max_braking_force(10) == float("inf")
        assert VehicleDynamics(
            ct16ev_params, load_transfer=MagicMock()
        ).max_braking_force(10) == float("inf")


# ---------------------------------------------------------------------------
# Effective rotational inertia tests  (Task: m_effective)
# ---------------------------------------------------------------------------

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
