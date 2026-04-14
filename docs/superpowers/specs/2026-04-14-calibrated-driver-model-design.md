# Calibrated Driver Model + FSAE Scoring Function

**Date**: 2026-04-14
**Status**: Approved
**Goal**: Build a per-segment driver model calibrated from Michigan 2025 telemetry that reproduces real driver behavior, collapsed into coachable zones, with FSAE Endurance + Efficiency scoring for strategy evaluation.

## Context

The Tier 3 simulation (4-wheel Pacejka model) is validated to ~2% energy error against Michigan 2025 telemetry. The tire model shows peak mu of ~2.66, but the real driver used only ~0.89g mean lateral acceleration — the car has far more grip than needed. FSAE endurance is an energy management game: the winning strategy balances lap time against energy consumption to maximize combined Endurance (275 pts) + Efficiency (100 pts) scoring.

The team's 2025 Michigan strategy was conservative: low power limits (85 Nm), low max RPM (2900), no braking, coast-only into corners. Result: 8th in endurance (152.9 pts), 1st in efficiency (100 pts), 252.9 combined. The simulation needs to find the strategy that maximizes that combined score.

This is Phase 2 of the project roadmap. Phase 3 (parameter sweeps and automated optimization) builds directly on the driver model and scoring function created here.

## Decision Log

| Decision | Choice | Rationale |
|---|---|---|
| Driver model type | Per-segment action table with zone collapsing | Michigan track is stable year-to-year; per-segment gives exact coaching output |
| Zone collapsing | Merge adjacent same-action segments | Reduces ~200 fine segments to ~30-40 coachable zones |
| Calibration source | AiM telemetry (Throttle Pos, FBrakePressure, RBrakePressure) | Direct measurement of what the driver did at each point |
| Scoring function scope | Full FSAE formulas with configurable competition field | Needed as foundation for Phase 3 optimization; competition field params (Tmin, CO2min) let you test robustness |
| Action classification | Throttle/Coast/Brake with intensity 0-1 | Matches existing ControlCommand interface exactly |
| Engine integration | New strategy class implementing DriverStrategy.decide() | Plugs into existing sim engine without engine changes |
| Optimizer | Out of scope (Phase 3) | Focus on high-quality foundation; sweep machinery comes next |

## Component 1: Zone-Based Driver Model

**File**: `src/fsae_sim/driver/strategies.py` (add `CalibratedStrategy` class)

### Data Structure: DriverZone

```python
@dataclass(frozen=True)
class DriverZone:
    zone_id: int                    # sequential zone number
    segment_start: int              # first segment index (inclusive)
    segment_end: int                # last segment index (inclusive)
    action: ControlAction           # THROTTLE, COAST, or BRAKE
    intensity: float                # 0.0 to 1.0 (throttle_pct or brake_pct)
    distance_start_m: float         # track distance where zone begins
    distance_end_m: float           # track distance where zone ends
    label: str                      # human-readable: "Turn 1 entry", "Back straight"
```

### Class: CalibratedStrategy

**Implements**: `DriverStrategy`

**Construction**:
```python
CalibratedStrategy(
    zones: list[DriverZone],
    num_segments: int,              # total track segments (for index mapping)
    name: str = "calibrated",
)
```

Internally builds a flat lookup array: `segment_idx -> (action, intensity)` from the zone list. The `decide()` method indexes into this array using `state.segment_idx % num_segments` for multi-lap wrapping.

**decide() method**:
```python
def decide(self, state: SimState, upcoming: list[Segment]) -> ControlCommand:
```
1. Look up action and intensity for `state.segment_idx % num_segments`
2. Return `ControlCommand(action, throttle_pct=intensity if THROTTLE else 0, brake_pct=intensity if BRAKE else 0)`

No speed targets, no force models. The strategy just says what the driver does. The sim engine's synthetic-mode processing handles the physics (force balance, corner speed clamping, traction limits).

**Helper methods**:
- `zone_for_segment(segment_idx: int) -> DriverZone` — which zone a segment belongs to
- `to_dataframe() -> pd.DataFrame` — export zone table for display/analysis
- `to_driver_brief() -> str` — formatted text output for driver coaching
- `with_zone_override(zone_id: int, action: ControlAction, intensity: float) -> CalibratedStrategy` — return a new strategy with one zone modified (foundation for Phase 3 perturbation)

### Class methods for construction

**From telemetry (primary path)**:
```python
@classmethod
def from_telemetry(
    cls,
    aim_df: pd.DataFrame,
    track: Track,
    *,
    laps: list[int] | None = None,         # which laps to average (None = all non-change laps)
    throttle_col: str = "Throttle Pos",
    front_brake_col: str = "FBrakePressure",
    rear_brake_col: str = "RBrakePressure",
    speed_col: str = "GPS Speed",
    distance_col: str = "Distance on GPS Speed",
    throttle_threshold: float = 5.0,       # % above which = throttle action
    brake_threshold: float = 2.0,          # bar above which = brake action
    merge_tolerance: float = 0.05,         # intensity diff within which zones merge
) -> CalibratedStrategy:
```

**From manual zone list (for testing and manual strategy creation)**:
```python
@classmethod
def from_zone_list(
    cls,
    zones: list[dict],    # [{"segments": (0, 18), "action": "throttle", "intensity": 1.0, "label": "..."}]
    track: Track,
) -> CalibratedStrategy:
```

### Calibration Algorithm (inside from_telemetry)

**Step 1: Sample telemetry at each segment midpoint**

For each of the track's N segments:
1. Compute segment midpoint distance: `segment.distance_start_m + segment.length_m / 2`
2. Find the telemetry rows within one lap that are closest to this distance
3. Average the telemetry values for that segment across non-driver-change laps (laps 2-10 for driver 1, laps 13-21 for driver 2 — skip lap 1 and the driver change lap as unrepresentative)
4. Classify action:
   - If `mean(max(FBrakePressure, RBrakePressure)) > brake_threshold`: **BRAKE**
     - Intensity: normalize brake pressure to 0-1 using 99th percentile of all nonzero brake samples (same approach as ReplayStrategy)
   - Elif `mean(Throttle Pos) > throttle_threshold`: **THROTTLE**
     - Intensity: `mean(Throttle Pos) / 100.0`, clipped to [0, 1]
   - Else: **COAST**
     - Intensity: 0.0

This produces a raw per-segment action array: `[(THROTTLE, 1.0), (THROTTLE, 0.95), ..., (COAST, 0.0), (BRAKE, 0.3), ...]`

**Step 2: Collapse into zones**

Walk the per-segment array and merge adjacent segments into zones:
1. Start a new zone at segment 0
2. For each subsequent segment:
   - If action type matches AND intensity difference < `merge_tolerance`: extend current zone
   - Else: close current zone, start new one
3. After merging, assign labels based on track geometry:
   - If all segments in zone have `|curvature| < 0.005`: label = "Straight"
   - If entering a curved section from a straight: label = "Turn N entry"
   - If inside a curved section: label = "Turn N apex"
   - If exiting a curved section to a straight: label = "Turn N exit"
   - Turn number increments for each distinct curved section on the track

**Step 3: Per-driver handling**

The Michigan endurance has a driver change at ~lap 11. The calibration should handle this:
- Extract zones for Driver 1 (laps 2-10) and Driver 2 (laps 13-21) separately
- `from_telemetry` takes an optional `laps` parameter to specify which laps to average
- Default: average all non-outlier laps for a "combined" driver profile
- Can create separate `CalibratedStrategy` per driver for comparison

## Component 2: Telemetry Extraction Utilities

**File**: `src/fsae_sim/analysis/telemetry_analysis.py` (new file)

Utility functions that support the calibration pipeline and produce analysis the team can use directly.

### Functions

**extract_per_segment_actions**:
```python
def extract_per_segment_actions(
    aim_df: pd.DataFrame,
    track: Track,
    *,
    laps: list[int] | None = None,          # which laps to average (None = all non-change laps)
    throttle_threshold: float = 5.0,
    brake_threshold: float = 2.0,
) -> pd.DataFrame:
```

Returns a DataFrame with one row per segment:
| Column | Type | Description |
|---|---|---|
| `segment_idx` | int | Segment index |
| `distance_m` | float | Segment midpoint distance |
| `curvature` | float | From track segment |
| `mean_throttle_pct` | float | Average throttle position (0-100) |
| `mean_brake_bar` | float | Average max(front, rear) brake pressure |
| `mean_speed_kmh` | float | Average GPS speed |
| `action` | ControlAction | Classified action |
| `intensity` | float | Normalized intensity (0-1) |

**collapse_to_zones**:
```python
def collapse_to_zones(
    segment_actions: pd.DataFrame,
    track: Track,
    *,
    merge_tolerance: float = 0.05,
) -> list[DriverZone]:
```

Takes the per-segment DataFrame and produces the collapsed zone list with auto-generated labels.

**detect_laps**:
```python
def detect_laps(
    aim_df: pd.DataFrame,
    lap_distance_m: float,
) -> list[tuple[int, int, float]]:
```

Returns `(start_idx, end_idx, lap_time)` for each lap. Wraps the existing `detect_lap_boundaries` from `validation.py` (which already implements lap detection from distance rollover). Used to select which laps to average during calibration.

**compare_driver_stints**:
```python
def compare_driver_stints(
    aim_df: pd.DataFrame,
    track: Track,
) -> pd.DataFrame:
```

Returns per-zone comparison of Driver 1 vs Driver 2 behavior: where they differ in throttle intensity, coast points, brake points. Useful for the team to understand driver-to-driver variation.

## Component 3: FSAE Scoring Function

**File**: `src/fsae_sim/analysis/scoring.py` (new file)

### Class: FSAEScoring

```python
@dataclass
class CompetitionField:
    """Assumptions about the competition field for scoring."""
    endurance_tmin_s: float              # fastest team's corrected endurance time
    efficiency_co2min_kg_per_lap: float  # most efficient team's CO2 per lap
    efficiency_efmax: float              # highest efficiency factor in the field
    efficiency_tmin_laps: int = 22       # laps completed by fastest team
    efficiency_co2min_laps: int = 22     # laps completed by most efficient team
```

```python
@dataclass
class FSAEScoreResult:
    endurance_time_score: float      # 0-250
    endurance_laps_score: float      # 0-25
    endurance_total: float           # 0-275
    efficiency_factor: float         # your efficiency factor
    efficiency_score: float          # 0-100
    combined_score: float            # endurance_total + efficiency_score (0-375)
    your_time_s: float               # your corrected endurance time
    your_energy_kwh: float           # your total energy consumed
    your_co2_kg: float               # your CO2 equivalent
    your_avg_lap_s: float            # your average lap time
    your_co2_per_lap: float          # your CO2 per lap
```

```python
class FSAEScoring:
    CO2_PER_KWH_EV = 0.65  # kg CO2 per kWh (D.13.4.1c)
    ENDURANCE_TIME_MAX_FACTOR = 1.45  # Tmax = 1.45 * Tmin (D.12.13.1)
    EV_CO2_MAX_PER_100KM = 20.02  # kg CO2/100km eligibility cap (D.13.4.5)
    
    def __init__(self, field: CompetitionField):
        self.field = field
    
    def score(
        self,
        total_time_s: float,
        total_energy_kwh: float,
        laps_completed: int,
        cone_penalties: int = 0,
        off_course_penalties: int = 0,
    ) -> FSAEScoreResult:
        """Compute combined Endurance + Efficiency score."""
    
    @classmethod
    def michigan_2025_field(cls) -> FSAEScoring:
        """Pre-configured with 2025 Michigan competition field data."""
        return cls(CompetitionField(
            endurance_tmin_s=1369.936,              # San Jose State
            efficiency_co2min_kg_per_lap=0.0967,    # UConn
            efficiency_efmax=0.848,                 # UConn (won efficiency)
            efficiency_tmin_laps=22,
            efficiency_co2min_laps=22,
        ))
```

### Scoring formulas (from 2026 FSAE Rules D.12.13 and D.13.4)

**Endurance Time Score** (D.12.13.2):
```
Tmax = 1.45 * Tmin
If Tyour < Tmax:
    Time Score = 250 * ((Tmax / Tyour) - 1) / ((Tmax / Tmin) - 1)
Else:
    Time Score = 0
```

**Endurance Laps Score** (D.12.13.3):
- 1 point per lap for laps 1-11
- 3 points for completing driver change (lap 12)
- 1 point per lap for laps 13-22
- Max 25 points

**Efficiency Factor** (D.13.4.4):
```
EF = (Tmin_avg / Tyour_avg) * (CO2min_per_lap / CO2your_per_lap)
where:
    Tmin_avg = Tmin / LapTotal_Tmin
    Tyour_avg = Tyour / Lapyours
    CO2min_per_lap = CO2min / LapTotal_CO2min
    CO2your_per_lap = CO2your / Lapyours
    CO2your = energy_kwh * 0.65
```

**Efficiency Score** (D.13.4.6):
```
EFmin = computed using CO2_max (20.02 kg/100km for EV) and Tyour = 1.45 * Tmin
Efficiency Score = 100 * (EFyour - EFmin) / (EFmax - EFmin)
```

EFmax is the highest efficiency factor among all eligible teams. In scoring mode, this is computed from the competition field data. When using `michigan_2025_field()`, EFmax = UConn's 2025 efficiency factor (0.848), since UConn won efficiency. When scoring a hypothetical strategy, if your EF exceeds the field's EFmax, your EF becomes the new EFmax (you'd win efficiency with that strategy).

**Eligibility checks** (D.13.3):
- Average lap time must be < 1.45 * fastest team's average lap time
- Average energy per lap must be < 20.02 kg CO2 / 100km equivalent
- Must have crossed start line after driver change (completed > 11 laps)

### Convenience method

```python
def score_sim_result(
    self,
    result: SimResult,
    track_distance_km: float,
    cone_penalties: int = 0,
    off_course_penalties: int = 0,
) -> FSAEScoreResult:
    """Score a SimResult directly."""
```

Takes a `SimResult` from the sim engine and computes the full FSAE score.

## Component 4: Validation

### Calibration accuracy validation

After building the `CalibratedStrategy` from telemetry, run a full 22-lap simulation and compare:

| Metric | Target | Method |
|---|---|---|
| Total driving time | < 3% error vs telemetry | SimResult.total_time_s vs AiM total |
| Total energy consumed | < 3% error | SimResult.total_energy_kwh vs AiM |
| Final SOC | < 3% error | SimResult.final_soc vs AiM |
| Per-zone speed profile | Mean speed per zone within 5 km/h of telemetry | Compare sim avg speed per zone to telemetry avg speed per zone |
| FSAE score | Reproduce 2025 score (152.9 endurance + 100 efficiency = 252.9) within 5% | FSAEScoring.michigan_2025_field().score() |

The key test: the CalibratedStrategy, running through the physics-based sim engine (not replay), should produce results close to the actual telemetry. This validates that the zone-based action table captures the driver's behavior well enough to be a reliable foundation for Phase 3 optimization.

### Comparison to ReplayStrategy

The existing ReplayStrategy achieves ~2% error by replaying exact speeds and torques. The CalibratedStrategy will have higher error because it uses the physics model to resolve speeds from force balance rather than replaying them directly. Expected: 3-5% error on time and energy. This is acceptable because the physics model is what gets exercised during optimization — we need to validate its behavior, not bypass it.

### Zone quality checks

- No zone should span more than 200m of track (indicates under-segmentation)
- No zone should span less than 5m of track (indicates over-segmentation / noise)
- Total zone count should be 25-50 (sanity check for Michigan-type track)
- Every corner section should have at least a coast or brake zone on entry

## Component 5: Documentation Updates

### CLAUDE.md updates

Update the Project Roadmap to reflect completed phases and current state:
- Phase 1: Mark as DONE with accuracy summary
- Phase 2: Mark as CURRENT with component list
- Phase 3: Reference the scoring function and driver model as foundation

### Architecture documentation

Update `docs/architecture.md` (or create if needed) with:
- Driver model architecture (zone-based, calibrated from telemetry)
- FSAE scoring function (formulas, competition field configuration)
- Data flow: telemetry -> calibration -> zones -> sim engine -> scoring
- How Phase 3 optimization will build on this (zone perturbation, car tune sweep)

### Inline documentation

- `CalibratedStrategy` docstring: explains the zone concept, calibration process, and how to use for manual strategy testing
- `FSAEScoring` docstring: references exact FSAE rule sections (D.12.13, D.13.4) and explains competition field assumptions
- `telemetry_analysis.py` module docstring: explains the calibration pipeline and what each function does

## File Summary

| File | Action | Description |
|---|---|---|
| `src/fsae_sim/driver/strategies.py` | Modify | Add `DriverZone` dataclass and `CalibratedStrategy` class |
| `src/fsae_sim/analysis/telemetry_analysis.py` | Create | Calibration pipeline: per-segment extraction, zone collapsing, driver comparison |
| `src/fsae_sim/analysis/scoring.py` | Create | `FSAEScoring`, `CompetitionField`, `FSAEScoreResult`, scoring formulas |
| `tests/test_calibrated_strategy.py` | Create | Tests for zone model, decide(), overrides, edge cases |
| `tests/test_telemetry_analysis.py` | Create | Tests for segment extraction, zone collapsing, lap detection |
| `tests/test_scoring.py` | Create | Tests for scoring formulas against known 2025 results |
| `scripts/validate_driver_model.py` | Create | End-to-end: calibrate from telemetry, run sim, compare, score |
| `CLAUDE.md` | Modify | Update roadmap and architecture guidance |

## Build Order

1. **FSAE Scoring** — independent, testable against 2025 results immediately
2. **Telemetry extraction utilities** — depends on existing Track and data loader
3. **CalibratedStrategy** — depends on extraction utilities and DriverZone
4. **Validation script** — depends on all above
5. **Documentation updates** — after everything works
