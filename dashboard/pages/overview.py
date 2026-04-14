"""Simulation Overview — baseline sim results and FSAE scoring."""

import dash
from dash import html, dcc
import dash_bootstrap_components as dbc
import plotly.graph_objects as go

from dashboard.data.sim_runner import get_baseline_result, get_baseline_score

dash.register_page(__name__, path="/", name="Overview")


def metric_card(title: str, value: str, subtitle: str) -> dbc.Card:
    return dbc.Card(
        dbc.CardBody([
            html.P(title, className="text-muted mb-1 small"),
            html.H3(value, className="mb-1"),
            html.Small(subtitle, className="text-muted"),
        ]),
        className="mb-3",
    )


def _build_error_layout(msg: str):
    return dbc.Container([
        html.H2("Simulation Overview", className="mb-4"),
        dbc.Alert(msg, color="danger"),
    ], fluid=True)


def _build_layout():
    result = get_baseline_result()
    if result is None:
        return _build_error_layout(
            "Baseline simulation failed to run. Check console for errors."
        )

    score = get_baseline_score()
    states = result.states

    # Format metric values
    minutes = int(result.total_time_s // 60)
    seconds = result.total_time_s % 60
    time_str = f"{minutes}:{seconds:05.2f}"
    energy_str = f"{result.total_energy_kwh:.2f}"
    soc_str = f"{result.final_soc:.1f}%"
    score_str = f"{score.combined_score:.1f}" if score else "--"

    # Subtitles
    time_sub = f"{result.laps_completed} laps completed"
    energy_sub = f"{result.total_energy_kwh / result.laps_completed:.3f} kWh/lap" if result.laps_completed > 0 else ""
    soc_sub = f"Started at {states['soc_pct'].iloc[0]:.1f}%"
    score_sub = "out of 375 max" if score else "scoring unavailable"

    # --- SOC vs Distance chart ---
    soc_fig = go.Figure()
    soc_fig.add_trace(go.Scatter(
        x=states["distance_m"] / 1000,
        y=states["soc_pct"],
        mode="lines",
        name="SOC",
        line=dict(color="#3498db", width=2),
    ))
    soc_fig.update_layout(
        template="plotly_dark",
        height=300,
        margin=dict(l=50, r=20, t=40, b=40),
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        title="SOC Depletion Over Endurance",
        xaxis_title="Distance (km)",
        yaxis_title="SOC (%)",
    )

    # --- Speed vs Distance chart ---
    # Downsample for performance (every 5th point)
    step = max(1, len(states) // 1000)
    speed_fig = go.Figure()
    speed_fig.add_trace(go.Scatter(
        x=states["distance_m"].iloc[::step] / 1000,
        y=states["speed_kmh"].iloc[::step],
        mode="lines",
        name="Speed",
        line=dict(color="#2ecc71", width=1),
    ))
    speed_fig.update_layout(
        template="plotly_dark",
        height=300,
        margin=dict(l=50, r=20, t=40, b=40),
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        title="Speed Profile",
        xaxis_title="Distance (km)",
        yaxis_title="Speed (km/h)",
    )

    # --- Scoring breakdown table ---
    scoring_rows = []
    if score:
        scoring_rows = [
            html.Tr([html.Td("Endurance Time Score"), html.Td(f"{score.endurance_time_score:.1f} / 250")]),
            html.Tr([html.Td("Endurance Laps Score"), html.Td(f"{score.endurance_laps_score:.1f} / 25")]),
            html.Tr([html.Td("Endurance Total"), html.Td(f"{score.endurance_total:.1f} / 275")]),
            html.Tr([html.Td("Efficiency Factor"), html.Td(f"{score.efficiency_factor:.4f}")]),
            html.Tr([html.Td("Efficiency Score"), html.Td(f"{score.efficiency_score:.1f} / 100")]),
            html.Tr([
                html.Td(html.Strong("Combined Score")),
                html.Td(html.Strong(f"{score.combined_score:.1f} / 375")),
            ]),
        ]

    return dbc.Container([
        html.H2("Simulation Overview", className="mb-4"),
        html.P(
            f"Baseline: {result.config_name} — {result.strategy_name} strategy — "
            f"{result.track_name}",
            className="text-muted mb-3",
        ),

        # Metric cards
        dbc.Row([
            dbc.Col(metric_card("Total Time", time_str, time_sub), md=3),
            dbc.Col(metric_card("Energy Used", f"{energy_str} kWh", energy_sub), md=3),
            dbc.Col(metric_card("Final SOC", soc_str, soc_sub), md=3),
            dbc.Col(metric_card("FSAE Score", score_str, score_sub), md=3),
        ]),

        # Charts
        dbc.Row([
            dbc.Col(dcc.Graph(figure=soc_fig), md=6),
            dbc.Col(dcc.Graph(figure=speed_fig), md=6),
        ]),

        # Scoring breakdown
        dbc.Row([
            dbc.Col([
                html.H5("FSAE Scoring Breakdown", className="mt-3 mb-2"),
                dbc.Table(
                    [html.Tbody(scoring_rows)] if scoring_rows else [],
                    bordered=True,
                    color="dark",
                    hover=True,
                    size="sm",
                ),
            ], md=6),
            dbc.Col([
                html.H5("Run Summary", className="mt-3 mb-2"),
                dbc.Table([html.Tbody([
                    html.Tr([html.Td("Config"), html.Td(result.config_name)]),
                    html.Tr([html.Td("Strategy"), html.Td(result.strategy_name)]),
                    html.Tr([html.Td("Track"), html.Td(result.track_name)]),
                    html.Tr([html.Td("Laps"), html.Td(str(result.laps_completed))]),
                    html.Tr([html.Td("CO2 Equivalent"), html.Td(f"{score.your_co2_kg:.2f} kg" if score else "--")]),
                    html.Tr([html.Td("Avg Lap Time"), html.Td(f"{score.your_avg_lap_s:.2f} s" if score else "--")]),
                ])], bordered=True, color="dark", hover=True, size="sm"),
            ], md=6),
        ]),
    ], fluid=True)


layout = _build_layout()
