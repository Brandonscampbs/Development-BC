"""Sweep Results — parameter sweep visualization (Phase 3).

Scans the results/ directory for sweep output. When sweep data exists,
displays heatmaps, sensitivity plots, and top configurations. When no
data exists, shows the current Phase 3 status.
"""

import dash
from dash import html
import dash_bootstrap_components as dbc
from pathlib import Path

dash.register_page(__name__, path="/sweeps", name="Sweep Results")

_PROJECT_ROOT = Path(__file__).parent.parent.parent
_RESULTS_DIR = _PROJECT_ROOT / "results"


def _find_sweep_dirs() -> list[Path]:
    """Find sweep result directories in results/."""
    if not _RESULTS_DIR.exists():
        return []
    return sorted(
        [d for d in _RESULTS_DIR.iterdir() if d.is_dir() and d.name.startswith("sweep_")],
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )


def _build_layout():
    sweep_dirs = _find_sweep_dirs()

    if not sweep_dirs:
        return dbc.Container([
            html.H2("Parameter Sweep Results", className="mb-4"),
            dbc.Alert([
                html.H5("No sweep results yet", className="alert-heading"),
                html.P(
                    "This page visualizes parameter sweep results from Phase 3. "
                    "Sweeps vary driver parameters (zone overrides, throttle intensity) "
                    "and car tune (max RPM, torque limit, regen) to find the combination "
                    "that maximizes combined FSAE Endurance + Efficiency score.",
                ),
                html.Hr(),
                html.P([
                    html.Strong("Phase 3 status: "),
                    "Sweep infrastructure is next on the roadmap. "
                    "Once implemented, results will appear here automatically.",
                ], className="mb-0"),
            ], color="info"),

            html.H5("What sweeps will explore", className="mt-4 mb-3"),
            dbc.Table([
                html.Thead(html.Tr([
                    html.Th("Parameter"),
                    html.Th("Range"),
                    html.Th("Why it matters"),
                ])),
                html.Tbody([
                    html.Tr([
                        html.Td("Max RPM"),
                        html.Td("2000 - 3500 RPM"),
                        html.Td("Top speed vs efficiency tradeoff"),
                    ]),
                    html.Tr([
                        html.Td("Torque limit"),
                        html.Td("50 - 85 Nm"),
                        html.Td("Acceleration vs energy consumption"),
                    ]),
                    html.Tr([
                        html.Td("Regen intensity"),
                        html.Td("0 - 100%"),
                        html.Td("Energy recovery vs driver feel"),
                    ]),
                    html.Tr([
                        html.Td("Zone throttle overrides"),
                        html.Td("Per-zone 0-100%"),
                        html.Td("Fine-tune acceleration in each track section"),
                    ]),
                    html.Tr([
                        html.Td("Coast distance"),
                        html.Td("Per-corner adjustment"),
                        html.Td("When to lift before corners"),
                    ]),
                ]),
            ], bordered=True, color="dark", hover=True, size="sm"),

            html.H5("Expected results format", className="mt-4 mb-3"),
            html.Pre(
                "results/\n"
                "  sweep_{param}_{car}_{timestamp}/\n"
                "    manifest.json         # sweep config and parameter ranges\n"
                "    summary.parquet       # one row per run: params + metrics + score\n"
                "    run_{NNN}.parquet     # full time-series per run (optional)\n",
                className="bg-dark p-3 rounded small",
            ),
        ], fluid=True)

    # When sweep data exists, load and display it
    # (This section will be populated when Phase 3 sweep runner is built)
    return dbc.Container([
        html.H2("Parameter Sweep Results", className="mb-4"),
        html.P(
            f"Found {len(sweep_dirs)} sweep result(s) in results/.",
            className="text-muted mb-3",
        ),
        dbc.Alert(
            "Sweep visualization will be built alongside the Phase 3 sweep runner.",
            color="info",
        ),
    ], fluid=True)


layout = _build_layout()
