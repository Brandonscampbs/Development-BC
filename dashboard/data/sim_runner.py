"""Baseline simulation runner with caching for the dashboard.

Runs the Michigan 2025 baseline simulation (ReplayStrategy, CT-16EV,
22 laps) on first access and caches the result in memory. All dashboard
pages that need simulation data import from here.
"""

from __future__ import annotations

import logging
from pathlib import Path

from dashboard.data.telemetry_loader import get_telemetry

logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).parent.parent.parent
_CONFIG_PATH = _PROJECT_ROOT / "configs" / "ct16ev.yaml"
_TELEMETRY_PATH = _PROJECT_ROOT / "Real-Car-Data-And-Stats" / "2025 Endurance Data.csv"
_VOLTT_CELL_PATH = (
    _PROJECT_ROOT
    / "Real-Car-Data-And-Stats"
    / "About-Energy-Volt-Simulations-2025-Pack"
    / "2025_Pack_cell.csv"
)

_cache: dict = {}


def get_baseline_result():
    """Run or return cached baseline simulation result.

    Returns:
        SimResult from a 22-lap ReplayStrategy simulation of Michigan 2025.
        Returns None if the simulation fails.
    """
    if "result" in _cache:
        return _cache["result"]

    try:
        from fsae_sim.vehicle import VehicleConfig
        from fsae_sim.track.track import Track
        from fsae_sim.vehicle.battery_model import BatteryModel
        from fsae_sim.driver.strategies import ReplayStrategy
        from fsae_sim.sim.engine import SimulationEngine

        logger.info("Running baseline simulation (first load)...")

        vehicle = VehicleConfig.from_yaml(_CONFIG_PATH)
        track = Track.from_telemetry(str(_TELEMETRY_PATH))
        battery = BatteryModel.from_config_and_data(
            vehicle.battery, str(_VOLTT_CELL_PATH),
        )

        aim_df = get_telemetry()
        battery.calibrate_pack_from_telemetry(aim_df)

        strategy = ReplayStrategy.from_full_endurance(
            aim_df, track.total_distance_m,
        )

        engine = SimulationEngine(vehicle, track, strategy, battery)
        result = engine.run(
            num_laps=22,
            initial_soc_pct=95.0,
            initial_temp_c=29.0,
        )

        _cache["result"] = result
        _cache["track"] = track
        logger.info(
            "Baseline sim complete: %.1fs, %.2f kWh, %.1f%% SOC",
            result.total_time_s, result.total_energy_kwh, result.final_soc,
        )
        return result

    except Exception:
        logger.exception("Failed to run baseline simulation")
        _cache["result"] = None
        return None


def get_track():
    """Return the Track object (built during baseline sim)."""
    if "track" not in _cache:
        get_baseline_result()
    return _cache.get("track")


def get_baseline_score():
    """Compute FSAE scoring for the baseline simulation result.

    Returns:
        FSAEScoreResult, or None if sim failed.
    """
    if "score" in _cache:
        return _cache["score"]

    result = get_baseline_result()
    if result is None:
        return None

    try:
        from fsae_sim.analysis.scoring import FSAEScoring

        track = _cache["track"]
        scorer = FSAEScoring.michigan_2025_field()
        score = scorer.score_sim_result(
            result, track.total_distance_m / 1000,
        )
        _cache["score"] = score
        return score

    except Exception:
        logger.exception("Failed to compute FSAE scoring")
        return None


def get_validation_report():
    """Validate baseline sim against real telemetry.

    Returns:
        ValidationReport, or None if validation fails.
    """
    if "validation" in _cache:
        return _cache["validation"]

    result = get_baseline_result()
    if result is None:
        return None

    try:
        from fsae_sim.analysis.validation import validate_full_endurance

        aim_df = get_telemetry()
        report = validate_full_endurance(
            result.states,
            aim_df,
            result.total_time_s,
            result.final_soc,
            result.total_energy_kwh,
            result.laps_completed,
        )
        _cache["validation"] = report
        return report

    except Exception:
        logger.exception("Failed to run validation")
        return None
