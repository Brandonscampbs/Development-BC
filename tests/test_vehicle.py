"""Tests for vehicle configuration loading."""

import pytest
from fsae_sim.vehicle import VehicleConfig
from fsae_sim.vehicle.vehicle import TireConfig, SuspensionConfig


class TestVehicleConfigLoading:
    """Test YAML config loading into dataclasses."""

    def test_load_ct16ev(self, ct16ev_config_path):
        config = VehicleConfig.from_yaml(ct16ev_config_path)
        assert config.name == "CT-16EV"
        assert config.year == 2025

    def test_vehicle_params(self, ct16ev_config_path):
        config = VehicleConfig.from_yaml(ct16ev_config_path)
        assert config.vehicle.mass_kg == 278.0  # DSS: 210 car + 68 driver
        assert config.vehicle.drag_coefficient == 1.502  # DSS: CdA from drag data
        assert config.vehicle.rolling_resistance == 0.015

    def test_powertrain_params(self, ct16ev_config_path):
        config = VehicleConfig.from_yaml(ct16ev_config_path)
        assert config.powertrain.motor_speed_max_rpm == 2900
        assert config.powertrain.torque_limit_inverter_nm == 85.0
        assert config.powertrain.iq_limit_a == 170.0
        assert config.powertrain.gear_ratio == 3.818  # DSS: final drive ratio

    def test_battery_params(self, ct16ev_config_path):
        config = VehicleConfig.from_yaml(ct16ev_config_path)
        assert config.battery.cell_type == "P45B"
        assert config.battery.series == 110
        assert config.battery.parallel == 4
        assert config.battery.cell_voltage_min_v == 2.55
        assert config.battery.cell_voltage_max_v == 4.20  # DSS: 4.2V

    def test_discharge_limits(self, ct16ev_config_path):
        config = VehicleConfig.from_yaml(ct16ev_config_path)
        limits = config.battery.discharge_limits
        assert len(limits) == 8
        assert limits[0].temp_c == 30.0
        assert limits[0].max_current_a == 100.0
        assert limits[-1].temp_c == 65.0
        assert limits[-1].max_current_a == 0.0

    def test_soc_taper(self, ct16ev_config_path):
        config = VehicleConfig.from_yaml(ct16ev_config_path)
        assert config.battery.soc_taper_threshold_pct == 85.0
        assert config.battery.soc_taper_rate_a_per_pct == 1.0

    def test_load_ct17ev(self, ct17ev_config_path):
        config = VehicleConfig.from_yaml(ct17ev_config_path)
        assert config.name == "CT-17EV"
        assert config.year == 2026
        assert config.vehicle.mass_kg == 261.0
        assert config.battery.cell_type == "P50B"
        assert config.battery.series == 100


class TestTireConfig:
    def test_tire_config_construction(self):
        tc = TireConfig(tir_file="path/to/file.tir", static_camber_front_deg=-1.25, static_camber_rear_deg=-1.25)
        assert tc.tir_file == "path/to/file.tir"
        assert tc.static_camber_front_deg == -1.25
        assert tc.static_camber_rear_deg == -1.25

    def test_tire_config_is_frozen(self):
        tc = TireConfig(tir_file="path/to/file.tir", static_camber_front_deg=-1.25, static_camber_rear_deg=-1.25)
        with pytest.raises(AttributeError):
            tc.tir_file = "other.tir"

    def test_tire_config_requires_all_fields(self):
        with pytest.raises(TypeError):
            TireConfig(tir_file="path/to/file.tir")


class TestSuspensionConfig:
    def test_suspension_config_construction(self):
        sc = SuspensionConfig(
            roll_stiffness_front_nm_per_deg=238.0, roll_stiffness_rear_nm_per_deg=258.0,
            roll_center_height_front_mm=88.9, roll_center_height_rear_mm=63.5,
            roll_camber_front_deg_per_deg=-0.5, roll_camber_rear_deg_per_deg=-0.554,
            front_track_mm=1194.0, rear_track_mm=1168.0,
        )
        assert sc.roll_stiffness_front_nm_per_deg == 238.0
        assert sc.rear_track_mm == 1168.0

    def test_suspension_config_is_frozen(self):
        sc = SuspensionConfig(
            roll_stiffness_front_nm_per_deg=238.0, roll_stiffness_rear_nm_per_deg=258.0,
            roll_center_height_front_mm=88.9, roll_center_height_rear_mm=63.5,
            roll_camber_front_deg_per_deg=-0.5, roll_camber_rear_deg_per_deg=-0.554,
            front_track_mm=1194.0, rear_track_mm=1168.0,
        )
        with pytest.raises(AttributeError):
            sc.front_track_mm = 1200.0

    def test_suspension_config_requires_all_fields(self):
        with pytest.raises(TypeError):
            SuspensionConfig(roll_stiffness_front_nm_per_deg=238.0, roll_stiffness_rear_nm_per_deg=258.0)
