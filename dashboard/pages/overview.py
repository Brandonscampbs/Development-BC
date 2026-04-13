"""Overview page — simulation summary dashboard."""

import dash
from dash import html, dcc
import dash_bootstrap_components as dbc
import plotly.graph_objects as go

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


# Placeholder chart
placeholder_fig = go.Figure()
placeholder_fig.add_annotation(
    text="Run a simulation to see results",
    xref="paper", yref="paper",
    x=0.5, y=0.5, showarrow=False,
    font=dict(size=18, color="gray"),
)
placeholder_fig.update_layout(
    template="plotly_dark",
    height=400,
    paper_bgcolor="rgba(0,0,0,0)",
    plot_bgcolor="rgba(0,0,0,0)",
    xaxis=dict(visible=False),
    yaxis=dict(visible=False),
    margin=dict(l=0, r=0, t=40, b=0),
    title="SOC Depletion Over Endurance Run",
)

layout = dbc.Container(
    [
        html.H2("Simulation Overview", className="mb-4"),
        dbc.Row(
            [
                dbc.Col(metric_card("Best Lap Time", "--", "No data"), md=3),
                dbc.Col(metric_card("Total Energy", "--", "No data"), md=3),
                dbc.Col(metric_card("Final SOC", "--", "No data"), md=3),
                dbc.Col(metric_card("Predicted Points", "--", "No data"), md=3),
            ]
        ),
        dbc.Row([dbc.Col(dcc.Graph(figure=placeholder_fig), md=12)]),
        dbc.Row(
            [
                dbc.Col(
                    dbc.Alert(
                        [
                            html.H5("Getting Started", className="alert-heading"),
                            html.P(
                                "Run a simulation to populate this dashboard:"
                            ),
                            html.Code(
                                "python -m fsae_sim.sim.engine --config configs/ct17ev.yaml",
                                className="d-block mb-2",
                            ),
                            html.P(
                                "Or run a parameter sweep:",
                                className="mb-1",
                            ),
                            html.Code(
                                "python -m fsae_sim.optimization.sweep "
                                "--config ct17ev.yaml --sweep max_rpm"
                            ),
                        ],
                        color="info",
                        className="mt-3",
                    ),
                    md=12,
                ),
            ]
        ),
    ],
    fluid=True,
)
