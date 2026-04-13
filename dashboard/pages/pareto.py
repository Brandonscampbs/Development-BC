"""Pareto Frontier page — time vs energy tradeoff."""

import dash
from dash import html
import dash_bootstrap_components as dbc

dash.register_page(__name__, path="/pareto", name="Pareto Frontier")

layout = dbc.Container([
    html.H2("Pareto Frontier", className="mb-4"),
    dbc.Alert(
        "Interactive Pareto frontier showing the lap-time vs energy tradeoff. "
        "Click points to inspect strategy and config details. "
        "Available after optimization infrastructure is built.",
        color="info",
    ),
], fluid=True)
