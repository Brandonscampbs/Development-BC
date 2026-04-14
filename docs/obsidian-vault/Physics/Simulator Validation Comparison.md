---
title: Simulator Validation Comparison
tags: [physics, validation, comparison]
---

# Physics Validation: Our Sim vs. Professional Open-Source Simulators

> [!abstract] Summary
> A rigorous comparison of our FSAE EV simulation physics against 7+ professional and open-source lap simulators, including OpenLAP, OptimumLap, TORCS, fastest-lap, TUM, and Purdue Electric Racing. **Verdict: our simulation is in the top tier for FSAE tools, with a few addressable gaps.**

---

## Simulators Compared

| Simulator | Type | Language | Primary Use |
|-----------|------|----------|-------------|
| **OpenLAP** | Point-mass, quasi-static | MATLAB | Most popular open-source FSAE sim |
| **OptimumLap** | Point-mass, quasi-static | Commercial | Industry-standard FSAE tool |
| **TORCS / Speed Dreams** | Full multi-body, transient | C++ | Real-time racing game engine |
| **fastest-lap** | 3DOF/6DOF optimal control | C++ / Python | Academic, finds theoretical fastest lap |
| **TUM FTFTM** | Quasi-static, energy mgmt | Python | Formula E / F1 hybrid focus |
| **Purdue Electric Racing** | Segment-based, quasi-static | MATLAB | FSAE Electric specific |
| **RFR-lapsim / Berkeley / RoseLap** | Point-mass variants | Python / MATLAB | Various FSAE teams |

---

## Feature Comparison Matrix

| Feature | OpenLAP | OptimumLap | TORCS | fastest-lap | TUM | Purdue EV | **Our Sim** |
|---------|---------|------------|-------|-------------|-----|-----------|-------------|
| **Vehicle dynamics** | Point mass | Point mass | Full multi-body | 3DOF / 6DOF | Quasi-static | Segment-based | **Quasi-static + 4-wheel** |
| **Tire model** | Friction ellipse + load sensitivity | Friction coeff + load sensitivity | Magic Formula (B,C,E) | Pacejka (XML) | Force potential | Magic Formula | **PAC2002 Pacejka** |
| **Aero** | CdA, ClA | CdA, ClA | Wings + ground effect + drafting | Parameterized | With DRS | Minimal | **CdA, ClA** |
| **Powertrain** | ICE torque + gearing | Power curve + efficiency | Full drivetrain chain | Parameterized | Hybrid + Electric | Motor RPM-torque | **Motor + inverter + BMS limits** |
| **Battery/Energy** | None | Energy tracking only | Fuel only | None | Energy mgmt strategies | OCV-SOC interpolation | **OCV-SOC + R_internal + SOC taper** |
| **Driver model** | Reactive (fwd/bwd) | Optimal (at limit) | AI robots (behavioral) | Optimal control (Ipopt) | Configurable strategies | At limit | **Calibrated zones from telemetry** |
| **Track** | Distance + curvature | Sectors (radius + length) | 3D mesh | Mathematical circuit | Raceline CSV | Straight/corner table | **GPS-derived curvature** |
| **Sim method** | Quasi-static distance-step | Quasi-static distance-step | Transient 500Hz Euler | Transient optimal control | Quasi-static iterative | Quasi-static segment | **Quasi-static distance-step** |
| **Weight transfer** | No | No | Yes (suspension) | Yes (dynamic eqns) | Unknown | Basic longitudinal | **Geometric + elastic, 4-wheel** |
| **Rolling resistance** | Cr × Fz_total | Constant Cr | In tire model | In vehicle params | In vehicle model | Not detailed | **Cr × (mg + F_downforce)** |
| **Thermal** | None | None | Tire temp (simuv3) | None | None | None | **Battery lumped thermal** |

---

## Where We're Ahead

### 1. 4-Wheel PAC2002 Tire Model

OpenLAP and OptimumLap — the two most common FSAE tools — use simple friction ellipse / friction coefficient with load sensitivity. They have **no Pacejka**, no slip angles, no camber effects. Our full PAC2002 implementation with load-dependent friction, cornering stiffness, and camber sensitivity is a tier above.

Only Purdue EV and TORCS use Magic Formula at all, and TORCS is a real-time game engine with different goals.

### 2. Cornering Solver with Load Transfer

Our cornering solver computes per-tire normal loads under lateral acceleration (geometric + elastic decomposition), estimates roll angle, applies roll-induced camber changes, then sums peak lateral capacity from all four tires via bisection search.

OpenLAP computes corner speed as `v = sqrt(mu * g / kappa)`. OptimumLap does the same. No load transfer, no camber, no tire model in the loop.

### 3. EV-Specific Battery Model

Our equivalent-circuit model (OCV-SOC + internal resistance + SOC taper + BMS current limits + lumped thermal) is the most complete of any open-source FSAE simulator:

| Feature | Purdue EV | TUM | **Our Sim** |
|---------|-----------|-----|-------------|
| OCV-SOC curve | Yes | Unknown | Yes (Voltt-calibrated) |
| Internal resistance | No | Unknown | Yes (SOC-dependent) |
| SOC taper current limit | No | Unknown | Yes |
| BMS temp-based limits | No | Unknown | Yes |
| Regen energy recovery | No | Yes | Yes |
| Lumped thermal model | No | No | Yes |
| Pack-level calibration from telemetry | No | No | Yes |

### 4. Calibrated Zone-Based Driver Model

Every other simulator either assumes the driver is **optimal** (always at the grip limit) or uses simple reactive logic. Our `CalibratedStrategy` is unique:
- Extracts actual driver behavior from AiM telemetry
- Collapses ~200 segments into ~30-40 coachable zones
- Each zone: action (throttle/coast/brake) + intensity (0-1)
- Supports `with_zone_override()` for parameter sweeps
- Produces `to_driver_brief()` for coaching output

This is critical for realistic endurance simulation — FSAE drivers don't drive at the limit.

### 5. FSAE Scoring Integration

No other open-source simulator directly computes FSAE competition points. Our `FSAEScoring` implements Endurance (D.12.13) + Efficiency (D.13.4) scoring with field data context.

---

## Actual Physics Gaps

### Gap 1: Forward-Backward Speed Solver — Medium-High Impact

> [!warning] Most impactful gap

**What we do:** Single forward pass. Compute `v_exit = sqrt(v_entry² + 2·a·dx)`, clamp to corner speed limit. If the car arrives too fast, it snaps to the limit.

**What OpenLAP/OptimumLap do:** Forward-backward integration:
1. Compute max cornering speed at every apex
2. **Backward pass** from each apex: "given max braking, what's the latest I can start decelerating?"
3. **Forward pass**: "given max acceleration, how fast can I go?"
4. Take the minimum envelope of all three

**Why it matters:** Our approach allows the car to carry impossible speed into a segment and instantaneously decelerate — violating F=ma. This can produce optimistic lap times and underestimate energy use in braking zones.

**Mitigation:** The `CalibratedStrategy` already encodes when to brake from telemetry, and `CoastOnlyStrategy` never brakes. This mainly affects `ThresholdBrakingStrategy` and Phase 3 sweeps where zone parameters vary braking behavior.

**Fix complexity:** ~100 lines of code. Well-documented in OpenLAP source.

### Gap 2: Longitudinal Tire Data — Medium Impact

**What we do:** Mirror the lateral Pacejka coefficients to create a longitudinal model (`.tir` file has zeroed longitudinal coefficients — lateral-only TTC data).

**Reality:** Longitudinal and lateral tire characteristics differ — different peak slip values, stiffness, and load sensitivity. Traction-limited acceleration and braking forces may be off by 10-20%.

**Mitigation:** FSAE EV acceleration is mostly powertrain-limited (85 Nm inverter limit), not tire-limited. Matters more for braking if we add aggressive braking strategies.

**Fix:** Obtain longitudinal tire data from TTC (Tire Testing Consortium) drive/brake sweeps if UConn has access. Otherwise, document the assumption.

### Gap 3: No Combined Slip in Cornering Solver — Low-Medium Impact

**What we do:** The cornering solver checks pure lateral capacity (peak Fy per tire). The friction circle coupling in `tire_model.py` exists but isn't used during max corner speed computation.

**What this means:** When computing max corner speed, we find where pure lateral grip = centripetal force. If the driver is also on throttle or brake mid-corner, available lateral grip decreases — but the solver doesn't account for this.

**Mitigation:** FSAE endurance driving is mostly sequential (corner → throttle), not combined. Trail-braking and throttle-on-exit scenarios would see the error.

### Gap 4: No Motor/Drivetrain Rotational Inertia — Low Impact

**What we do:** `F_wheel = T_motor × G × η / r`. No rotational inertia term.

**More complete:** `F_net = (T_motor × G × η - T_resist) / (m + J_eff/r²)` where `J_eff` accounts for motor rotor, gears, wheels. Effective mass increase is typically 3-8% for EVs.

**Why it's low impact:** Single-speed EV with no gear changes — the effect is a constant ~5% slower acceleration that could be absorbed into the drivetrain efficiency factor. Doesn't change the shape of results.

### Gap 5: No Tire Thermal Model — Low Impact (Currently)

No open-source FSAE sim models tire temperature either (only TORCS simuv3, which is a game). Hoosier LC0s are fairly thermally stable. Would matter more for tire compound comparison studies or extreme ambient conditions.

---

## Things That Are Correct As-Is

| Feature | Our Approach | Assessment |
|---------|-------------|------------|
| **Quasi-static simulation** | Distance-stepping, constant-a kinematics | Industry standard for FSAE. Academic consensus: within 2-5% of transient, 10x+ faster. Perfect for sweeps. |
| **Rolling resistance** | `F_roll = Crr × (mg + F_downforce)` | Correct — includes aero load on tires. Same as OpenLAP. |
| **Aero model** | CdA/ClA, `0.5 × ρ × C × A × v²` | Standard. Only TORCS models ground effect — overkill for FSAE. |
| **Battery equivalent circuit** | OCV(SOC) - I×R(SOC) with Voltt calibration | More than adequate. Only industrial BMS software does better. |
| **Powertrain torque envelope** | Constant torque → field weakening → zero | Correct piecewise model. Linear field weakening is standard. |
| **Load transfer model** | Geometric + elastic, 4-wheel | More detailed than any other quasi-static FSAE sim. |
| **Track from GPS telemetry** | Curvature from lateral accel, 5m bins, median filter | Standard approach. Matches OpenLAP's processing. |
| **Grade resistance** | `F_grade = m × g × sin(atan(grade))` | Exact trig for arbitrary grades. Correct. |
| **Regen modeling** | Efficiency-scaled, RPM-limited, SOC-aware | More detailed than any FSAE peer. |

---

## Recommendations (Priority Order)

### Priority 1: Add Backward Braking Pass
The one physics gap that could meaningfully affect Phase 3 sweep results. Algorithm is well-documented in [OpenLAP source](https://github.com/mc12027/OpenLAP-Lap-Time-Simulator). ~100 lines of code.

### Priority 2: Obtain Longitudinal Tire Data
If UConn has TTC access, get drive/brake sweep data for Hoosier LC0. If not, the mirrored-lateral approach is a documented, reasonable fallback.

### Priority 3: Add Effective Rotational Inertia
Simple `m_effective = m + J_rot/r²` term. Easy to add, small but systematic improvement (~3-8% acceleration correction).

### Not Worth Pursuing (for our use case)
- Tire thermal model — no FSAE sim does this, Hoosier LC0s are stable
- Transient dynamics — quasi-static is within 2-5%, 10x faster for sweeps
- Ground effect aero — only matters for underbody diffusers at high speed
- Full 3DOF/6DOF dynamics — overkill for strategy optimization

---

## Validation Context

Our current validation results:
- **~2% energy error** vs. Michigan 2025 telemetry
- **8/8 validation metrics passing**

This already exceeds what most FSAE teams achieve with OptimumLap. The academic consensus (see Duke FSAE's "[All Models Are Wrong But...](https://www.dukefsae.com/single-post/all-models-are-wrong-but)") is that for FSAE, the goal is **relative sensitivity analysis** (which design change gives more points), not absolute lap time prediction.

---

## Sources

- [OpenLAP GitHub](https://github.com/mc12027/OpenLAP-Lap-Time-Simulator)
- [OptimumG OptimumLap](https://optimumg.com/product/optimumlap/)
- [TORCS Source (GitHub)](https://github.com/jzbontar/torcs)
- [fastest-lap GitHub](https://github.com/juanmanzanero/fastest-lap)
- [TUM laptime-simulation GitHub](https://github.com/TUMFTM/laptime-simulation)
- [Purdue Electric Racing LapSim](https://github.com/PurdueElectricRacing/LapSim)
- [RFR-lapsim](https://github.com/rpatel3001/RFR-lapsim)
- [Berkeley FSAE-Lapsim](https://github.com/sean132/FSAE-Lapsim)
- [RoseLapWeb](https://github.com/RoseGPE/RoseLapWeb)

---

> [!tip] Related Pages
> - [[Quasi-Static Simulation]] — Our simulation methodology explained
> - [[Vehicle Dynamics]] — Force-balance and tire model details
> - [[Powertrain Model]] — Motor torque envelope and drivetrain
> - [[Battery Physics]] — Equivalent-circuit model
> - [[Driver Strategies]] — CalibratedStrategy and zone-based model
