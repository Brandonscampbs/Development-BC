"""Validation — simulation vs real telemetry comparison."""

import dash
from dash import html, dcc
import dash_bootstrap_components as dbc
import plotly.graph_objects as go
import numpy as np

from dashboard.data.sim_runner import (
    get_baseline_result,
    get_validation_report,
)
from dashboard.data.telemetry_loader import get_telemetry

dash.register_page(__name__, path="/validation", name="Validation")


def _build_error_layout(msg: str):
    return dbc.Container([
        html.H2("Simulation Validation", className="mb-4"),
        dbc.Alert(msg, color="danger"),
    ], fluid=True)


def _build_layout():
    result = get_baseline_result()
    if result is None:
        return _build_error_layout(
            "Baseline simulation failed. Cannot validate without sim results."
        )

    report = get_validation_report()
    if report is None:
        return _build_error_layout(
            "Validation failed to run. Check console for errors."
        )

    aim_df = get_telemetry()
    states = result.states

    # --- Summary badge ---
    passed = report.num_passed
    total = report.num_total
    badge_color = "success" if report.all_passed else "warning"
    badge_text = f"{passed}/{total} metrics pass"

    # --- Speed overlay: sim vs telemetry ---
    # Telemetry: filter to moving samples, plot speed vs cumulative distance
    telem_speed = aim_df["GPS Speed"].values
    telem_dist = aim_df["Distance on GPS Speed"].values / 1000  # km
    moving = telem_speed > 5.0

    # Downsample telemetry for plotting (every 10th moving sample)
    moving_idx = np.where(moving)[0]
    step_t = max(1, len(moving_idx) // 2000)
    t_idx = moving_idx[::step_t]

    # Sim speed vs distance
    step_s = max(1, len(states) // 2000)

    speed_fig = go.Figure()
    speed_fig.add_trace(go.Scatter(
        x=telem_dist[t_idx],
        y=telem_speed[t_idx],
        mode="lines",
        name="Telemetry",
        line=dict(color="#3498db", width=1),
        opacity=0.7,
    ))
    speed_fig.add_trace(go.Scatter(
        x=states["distance_m"].iloc[::step_s] / 1000,
        y=states["speed_kmh"].iloc[::step_s],
        mode="lines",
        name="Simulation",
        line=dict(color="#e74c3c", width=1),
        opacity=0.7,
    ))
    speed_fig.update_layout(
        template="plotly_dark",
        height=350,
        margin=dict(l=50, r=20, t=40, b=40),
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        title="Speed: Simulation vs Telemetry",
        xaxis_title="Distance (km)",
        yaxis_title="Speed (km/h)",
        legend=dict(x=0.01, y=0.99),
    )

    # --- SOC overlay: sim vs telemetry ---
    telem_soc = aim_df["State of Charge"].values

    soc_fig = go.Figure()
    soc_fig.add_trace(go.Scatter(
        x=telem_dist[t_idx],
        y=telem_soc[t_idx],
        mode="lines",
        name="Telemetry",
        line=dict(color="#3498db", width=2),
    ))
    soc_fig.add_trace(go.Scatter(
        x=states["distance_m"].iloc[::step_s] / 1000,
        y=states["soc_pct"].iloc[::step_s],
        mode="lines",
        name="Simulation",
        line=dict(color="#e74c3c", width=2),
    ))
    soc_fig.update_layout(
        template="plotly_dark",
        height=350,
        margin=dict(l=50, r=20, t=40, b=40),
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        title="SOC: Simulation vs Telemetry",
        xaxis_title="Distance (km)",
        yaxis_title="SOC (%)",
        legend=dict(x=0.01, y=0.99),
    )

    # --- Validation metrics table ---
    metric_rows = []
    for m in report.metrics:
        status_badge = dbc.Badge(
            "PASS" if m.passed else "FAIL",
            color="success" if m.passed else "danger",
            className="me-1",
        )
        metric_rows.append(html.Tr([
            html.Td(m.name),
            html.Td(f"{m.telemetry_value:.2f} {m.unit}"),
            html.Td(f"{m.simulation_value:.2f} {m.unit}"),
            html.Td(f"{m.relative_error_pct:.1f}%"),
            html.Td(f"< {m.target_pct:.0f}%"),
            html.Td(status_badge),
        ]))

    return dbc.Container([
        html.H2("Simulation Validation", className="mb-3"),
        html.P(
            "Comparing baseline simulation against Michigan 2025 AiM telemetry.",
            className="text-muted mb-3",
        ),

        # Summary badge
        dbc.Row([
            dbc.Col(
                dbc.Alert([
                    html.H5([
                        dbc.Badge(badge_text, color=badge_color, className="me-2"),
                        "Validation Status",
                    ]),
                    html.P(
                        f"Baseline simulation validated with "
                        f"{result.total_energy_kwh:.2f} kWh energy "
                        f"({passed}/{total} metrics within target).",
                        className="mb-0",
                    ),
                ], color=badge_color, className="mb-3"),
                md=12,
            ),
        ]),

        # Overlay charts
        dbc.Row([
            dbc.Col(dcc.Graph(figure=speed_fig), md=6),
            dbc.Col(dcc.Graph(figure=soc_fig), md=6),
        ]),

        # Metrics table
        dbc.Row([
            dbc.Col([
                html.H5("Validation Metrics", className="mt-3 mb-2"),
                dbc.Table([
                    html.Thead(html.Tr([
                        html.Th("Metric"),
                        html.Th("Telemetry"),
                        html.Th("Simulation"),
                        html.Th("Error"),
                        html.Th("Target"),
                        html.Th("Status"),
                    ])),
                    html.Tbody(metric_rows),
                ], bordered=True, color="dark", hover=True, striped=True, size="sm"),
            ], md=12),
        ]),
    ], fluid=True)


layout = _build_layout()
