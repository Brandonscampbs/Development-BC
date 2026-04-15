"""Powertrain configuration."""

from dataclasses import dataclass


@dataclass(frozen=True)
class PowertrainConfig:
    """Motor, inverter, and drivetrain parameters."""
    motor_speed_max_rpm: float
    brake_speed_rpm: float
    torque_limit_inverter_nm: float
    torque_limit_lvcu_nm: float
    iq_limit_a: float
    id_limit_a: float
    gear_ratio: float
    drivetrain_efficiency: float
    # LVCU torque command parameters (from real LVCU Code.txt)
    lvcu_power_constant: float = 420.0        # 4200 in 0.1Nm CAN units / 10
    lvcu_rpm_scale: float = 0.1076            # RPM to angular velocity scale
    lvcu_omega_floor: float = 23.04           # 230.4 in CAN units / 10
    lvcu_pedal_deadzone_low: float = 0.1      # tmap_lut V_MIN
    lvcu_pedal_deadzone_high: float = 0.9     # tmap_lut V_MAX
    lvcu_overspeed_rpm: float = 6000.0        # hard torque override threshold
    lvcu_overspeed_torque_nm: float = 30.0    # torque at overspeed (300/10)
