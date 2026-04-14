# 3D Vehicle Dynamics Visualization

**Status:** Idea / Future work
**Date:** 2026-04-14
**Priority:** Low (nice-to-have, after Phase 3 sweeps)

## Concept

A browser-based 3D visualization showing the car completing 1 lap of endurance with real-time telemetry overlays. Wireframe car model moves along the track with visible:

- **Tire traction forces** — colored arrows at each wheel (lateral + longitudinal)
- **Weight transfer** — chassis tilts to reflect load distribution
- **Driver inputs** — throttle/brake HUD overlay
- **Speed, SOC, energy** — telemetry gauges

Think: a simplified version of professional vehicle dynamics viewers, but running in the browser off our simulation output.

## Recommended Approach: Three.js

Three.js is the best fit — lightweight, performant, and purpose-built for this kind of browser 3D.

### Why Three.js over alternatives

| Option | Pros | Cons |
|---|---|---|
| **Three.js** | Fast, mature, perfect for wireframes + arrows, smooth animation | New dependency, separate from Dash |
| **Plotly 3D** (in Dash) | No new deps, stays in current stack | Poor animation performance, not built for playback |
| **dash-vtk / PyVista** | Scientific viz, good for force vectors | Heavy, niche, overkill for wireframe |
| **Rerun.io** | Built for exactly this (robotics/vehicle viz) | Separate app, not embedded in Dash |

### Architecture

```
Simulation output (per-timestep JSON)
    ↓
Three.js page (served alongside Dash app)
    ├── Wireframe chassis (box geometry)
    ├── 4 wheels (cylinder geometry)
    ├── Track spline (from GPS coordinates)
    ├── Force arrows (per-wheel, color-coded)
    ├── Chassis tilt (weight transfer)
    └── HUD overlay (throttle, brake, speed, SOC)
```

### Data pipeline

The simulation already outputs everything needed per timestep:
- Position on track (from GPS or distance-along-track)
- Speed, throttle, brake
- Per-wheel tire forces (Fx, Fy, Fz) from Pacejka model
- Weight transfer (already computed in vehicle dynamics)
- SOC, pack current, energy used

**Export format:** Single JSON file for 1 lap. At 20Hz over ~60s = ~1200 frames. Each frame:
```json
{
  "t": 0.05,
  "x": 12.3, "y": 45.6,
  "speed": 14.2,
  "heading": 1.23,
  "throttle": 0.8,
  "brake": 0.0,
  "wheels": [
    {"fx": 120, "fy": -45, "fz": 980},
    {"fx": 115, "fy": -42, "fz": 920},
    {"fx": 80, "fy": 30, "fz": 750},
    {"fx": 75, "fy": 28, "fz": 710}
  ],
  "roll": 0.02,
  "pitch": -0.01,
  "soc": 0.94,
  "energy_used": 0.42
}
```

### Wireframe car model

No need for a real CAD model. A simple parametric wireframe:
- Rectangular box for chassis (wheelbase 1549mm x ~1200mm track width x ~300mm height)
- 4 cylinders for wheels (10" diameter Hoosiers)
- Optional: lines for suspension links, aero surfaces
- Force vectors as cone-tipped lines at each wheel contact patch
- Color coding: green = grip available, yellow = moderate, red = near limit

### UI controls

- **Play/pause/scrub** — timeline slider to step through the lap
- **Playback speed** — 0.5x, 1x, 2x, 5x
- **Camera modes** — chase cam, bird's eye, fixed trackside, free orbit
- **Toggle layers** — show/hide force arrows, weight transfer tilt, HUD
- **Telemetry strip** — small 2D chart below showing speed/throttle/brake trace with playhead

## Effort Breakdown

| Task | Scope |
|---|---|
| Data export (sim → JSON) | Small — wire up existing per-step data |
| Track spline from GPS | Small — already have GPS coords |
| Three.js scene + wireframe car | Medium — box/cylinder geometry, materials |
| Animation along track + chassis tilt | Medium — interpolation, heading, roll/pitch |
| Force vector arrows | Small — Arrow helpers, color mapping |
| HUD overlay (throttle/brake/speed) | Small — HTML/CSS overlay |
| Timeline scrubber + playback | Medium — transport controls, frame stepping |
| Camera modes | Small-Medium — Three.js OrbitControls + follow logic |
| Integration with Dash app | Small — link/iframe, or separate route |

## Open Questions

- Serve as a separate page linked from Dash, or embed via iframe?
- Do we want to compare two laps side-by-side (e.g., optimal vs. actual)?
- Should this work with real telemetry too, not just simulation output?
- Worth adding track surface/curbing for spatial reference?

## References

- [Three.js](https://threejs.org/) — main library
- [Rerun.io](https://rerun.io/) — alternative worth revisiting if needs grow
- Existing sim outputs tire forces via Pacejka model in `vehicle/tire_model.py`
- Track representation in `track/track_model.py`
