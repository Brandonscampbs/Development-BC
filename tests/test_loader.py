"""Tests for telemetry and simulation data loaders."""

import pandas as pd
from tests.conftest import requires_data
from fsae_sim.data import load_aim_csv, load_voltt_csv


@requires_data
class TestAiMLoader:
    """Test AiM Race Studio CSV loader."""

    def test_returns_metadata_and_dataframe(self, aim_csv_path):
        metadata, df = load_aim_csv(aim_csv_path)
        assert isinstance(metadata, dict)
        assert isinstance(df, pd.DataFrame)

    def test_metadata_fields(self, aim_csv_path):
        metadata, _ = load_aim_csv(aim_csv_path)
        assert metadata["Vehicle"] == "CT-16EV"
        assert metadata["Format"] == "AiM CSV File"
        assert metadata["Sample Rate"] == "20"

    def test_dataframe_has_key_columns(self, aim_csv_path):
        _, df = load_aim_csv(aim_csv_path)
        expected_cols = [
            "Time", "GPS Speed", "RPM", "Pack Voltage",
            "Pack Current", "State of Charge", "Throttle Pos",
        ]
        for col in expected_cols:
            assert col in df.columns, f"Missing column: {col}"

    def test_dataframe_is_numeric(self, aim_csv_path):
        _, df = load_aim_csv(aim_csv_path)
        assert df["Time"].dtype in ("float64", "float32")
        assert df["GPS Speed"].dtype in ("float64", "float32")

    def test_dataframe_shape(self, aim_csv_path):
        _, df = load_aim_csv(aim_csv_path)
        assert len(df) > 30000  # ~37k rows at 20Hz over 31 min
        assert len(df.columns) > 90  # ~100+ channels

    def test_units_stored(self, aim_csv_path):
        _, df = load_aim_csv(aim_csv_path)
        assert "units" in df.attrs
        assert df.attrs["units"]["GPS Speed"] == "km/h"
        assert df.attrs["units"]["Time"] == "s"


@requires_data
class TestVolttLoader:
    """Test Voltt battery simulation CSV loader."""

    def test_loads_cell_csv(self, voltt_cell_path):
        df = load_voltt_csv(voltt_cell_path)
        assert isinstance(df, pd.DataFrame)
        assert len(df) > 10000

    def test_loads_pack_csv(self, voltt_pack_path):
        df = load_voltt_csv(voltt_pack_path)
        assert isinstance(df, pd.DataFrame)

    def test_expected_columns(self, voltt_cell_path):
        df = load_voltt_csv(voltt_cell_path)
        expected = ["Time [s]", "Voltage [V]", "SOC [%]", "Current [A]", "Temperature [°C]"]
        for col in expected:
            assert col in df.columns, f"Missing column: {col}"

    def test_numeric_data(self, voltt_cell_path):
        df = load_voltt_csv(voltt_cell_path)
        assert df["Time [s]"].dtype in ("float64", "float32")
        assert df["SOC [%]"].iloc[0] == 100.0  # starts at full charge
