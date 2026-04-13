"""Strategy Comparison page — compare driver strategies side-by-side."""

import dash
from dash import html
import dash_bootstrap_components as dbc

dash.register_page(__name__, path="/strategy", name="Strategy Comparison")

layout = dbc.Container([
    html.H2("Strategy Comparison", className="mb-4"),
    dbc.Alert(
        "Compare driver strategies (coasting, threshold braking, etc.) "
        "on the same car and track. Available after simulation implementation.",
        color="info",
    ),
], fluid=True)
