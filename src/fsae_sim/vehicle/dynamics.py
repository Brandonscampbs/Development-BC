"""Vehicle dynamics force-balance model.

Computes resistive forces, cornering speed limits, and longitudinal
acceleration for a quasi-static endurance simulation.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from fsae_sim.vehicle.vehicle import VehicleParams

if TYPE_CHECKING:
    from fsae_sim.vehicle.cornering_solver import CorneringSolver
    from fsae_sim.vehicle.load_transfer import LoadTransferModel
    from fsae_sim.vehicle.tire_model import PacejkaTireModel


class VehicleDynamics:
    """Longitudinal and lateral force-balance model.

    All forces follow the convention: positive = in the direction of travel
    (or resistance to it, returned as positive magnitude).

    When ``tire_model``, ``load_transfer``, and ``cornering_solver`` are
    provided, the model delegates cornering speed limits and
    traction/braking force limits to the physics-based subsystems.
    When they are ``None`` (the default), legacy analytical formulas are
    used, preserving backward compatibility.
    """

    AIR_DENSITY_KG_M3: float = 1.225  # ISA sea level, 15 C
    GRAVITY_M_S2: float = 9.81
    # Lateral grip limit for FSAE on dry asphalt (Hoosier R25B / LC0)
    _LEGACY_MAX_LATERAL_G: float = 1.3

    def __init__(
        self,
        vehicle: VehicleParams,
        tire_model: PacejkaTireModel | None = None,
        load_transfer: LoadTransferModel | None = None,
        cornering_solver: CorneringSolver | None = None,
    ) -> None:
        self.vehicle = vehicle
        self.tire_model = tire_model
        self.load_transfer = load_transfer
        self.cornering_solver = cornering_solver

    # ------------------------------------------------------------------
    # Resistance forces  (all return positive magnitudes)
    # ------------------------------------------------------------------

    def drag_force(self, speed_ms: float) -> float:
        """Aerodynamic drag (N).  F = 0.5 * rho * Cd * A * v^2."""
        v = abs(speed_ms)
        return (
            0.5
            * self.AIR_DENSITY_KG_M3
            * self.vehicle.drag_coefficient
            * self.vehicle.frontal_area_m2
            * v * v
        )

    def downforce(self, speed_ms: float) -> float:
        """Aerodynamic downforce (N). F = 0.5 * rho * ClA * v^2."""
        v = abs(speed_ms)
        return (
            0.5
            * self.AIR_DENSITY_KG_M3
            * self.vehicle.downforce_coefficient
            * v * v
        )

    def rolling_resistance_force(self, speed_ms: float = 0.0) -> float:
        """Rolling resistance (N).  Increases with downforce."""
        normal_force = (
            self.vehicle.mass_kg * self.GRAVITY_M_S2
            + self.downforce(speed_ms)
        )
        return normal_force * self.vehicle.rolling_resistance

    def grade_force(self, grade: float) -> float:
        """Grade resistance (N).  Positive grade = uphill = positive force opposing motion.

        ``grade`` is rise/run (dimensionless).
        """
        # sin(atan(grade)) for small grades ≈ grade, but exact is better
        angle = math.atan(grade)
        return self.vehicle.mass_kg * self.GRAVITY_M_S2 * math.sin(angle)

    def total_resistance(self, speed_ms: float, grade: float = 0.0) -> float:
        """Sum of all resistance forces (N) at given speed and grade."""
        return (
            self.drag_force(speed_ms)
            + self.rolling_resistance_force(speed_ms)
            + self.grade_force(grade)
        )

    # ------------------------------------------------------------------
    # Cornering speed limit
    # ------------------------------------------------------------------

    def max_cornering_speed(
        self, curvature: float, grip_factor: float = 1.0,
    ) -> float:
        """Maximum speed (m/s) through a corner of given curvature.

        When a ``CorneringSolver`` is available, delegates to it for a
        physics-based result that accounts for load transfer, tire model,
        and roll-induced camber.

        Otherwise falls back to the legacy analytical formula that uses
        ``_LEGACY_MAX_LATERAL_G`` with a downforce correction.

        For a straight segment (curvature ~ 0), returns infinity.
        """
        kappa = abs(curvature)
        if kappa < 1e-6:
            return float("inf")

        # Delegate to physics-based solver when available
        if self.cornering_solver is not None:
            return self.cornering_solver.max_cornering_speed(
                kappa, mu_scale=grip_factor,
            )

        # Legacy analytical formula
        mu = self._LEGACY_MAX_LATERAL_G * grip_factor
        m = self.vehicle.mass_kg
        g = self.GRAVITY_M_S2
        cl_a = self.vehicle.downforce_coefficient

        if cl_a < 1e-6:
            # No downforce: simple formula
            return math.sqrt(mu * g / kappa)

        # With downforce: (m*g + 0.5*rho*ClA*v^2)*mu = m*v^2*kappa
        # v^2 * (m*kappa - 0.5*rho*ClA*mu) = m*g*mu
        rho = self.AIR_DENSITY_KG_M3
        denom = m * kappa - 0.5 * rho * cl_a * mu
        if denom <= 0:
            # Downforce dominates: effectively unlimited speed for this curvature
            return float("inf")
        v_sq = m * g * mu / denom
        return math.sqrt(v_sq)

    # ------------------------------------------------------------------
    # Tire-limited traction / braking
    # ------------------------------------------------------------------

    def max_traction_force(self, speed_ms: float) -> float:
        """Maximum drive force (N) from rear tires.

        When tire and load-transfer models are available, returns the
        sum of peak longitudinal force from the two rear tires under
        mild acceleration load transfer (0.3 g forward).

        In legacy mode (no tire/load-transfer models), returns infinity
        so that the powertrain limit is the only constraint.
        """
        if self.tire_model is None or self.load_transfer is None:
            return float("inf")
        _, _, rl, rr = self.load_transfer.tire_loads(speed_ms, 0.0, 0.3)
        return (
            self.tire_model.peak_longitudinal_force(rl)
            + self.tire_model.peak_longitudinal_force(rr)
        )

    def max_braking_force(self, speed_ms: float) -> float:
        """Maximum braking force (N) from all four tires.

        When tire and load-transfer models are available, returns the
        sum of peak longitudinal force from all four tires under hard
        braking load transfer (-1.0 g).

        In legacy mode (no tire/load-transfer models), returns infinity
        so that there is no tire-limited braking constraint.
        """
        if self.tire_model is None or self.load_transfer is None:
            return float("inf")
        fl, fr, rl, rr = self.load_transfer.tire_loads(speed_ms, 0.0, -1.0)
        return sum(
            self.tire_model.peak_longitudinal_force(f)
            for f in [fl, fr, rl, rr]
        )

    # ------------------------------------------------------------------
    # Longitudinal acceleration
    # ------------------------------------------------------------------

    def acceleration(
        self, net_force_n: float,
    ) -> float:
        """Longitudinal acceleration (m/s^2) from net force.

        ``net_force_n`` is drive_force - total_resistance (positive = accelerating).
        """
        return net_force_n / self.vehicle.mass_kg

    def resolve_exit_speed(
        self,
        entry_speed_ms: float,
        segment_length_m: float,
        net_force_n: float,
        corner_speed_limit_ms: float,
    ) -> tuple[float, float]:
        """Compute segment exit speed and time from entry conditions.

        Uses constant-acceleration kinematics over the segment:
            v_exit^2 = v_entry^2 + 2 * a * d

        The exit speed is clamped to the corner speed limit.

        Returns:
            (exit_speed_ms, segment_time_s)
        """
        a = self.acceleration(net_force_n)

        # v^2 = v0^2 + 2*a*d
        v_sq = entry_speed_ms ** 2 + 2.0 * a * segment_length_m
        if v_sq < 0:
            # Car cannot make it through the segment (stalls)
            v_sq = 0.0

        exit_speed = math.sqrt(v_sq)

        # Clamp to cornering limit
        exit_speed = min(exit_speed, corner_speed_limit_ms)

        # Segment time: use average speed for the segment
        avg_speed = (entry_speed_ms + exit_speed) / 2.0
        if avg_speed < 0.1:
            # Near-zero speed: avoid division by zero, use small speed
            avg_speed = 0.1
        segment_time = segment_length_m / avg_speed

        return exit_speed, segment_time
