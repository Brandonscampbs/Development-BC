"""Car Comparison page — CT-16EV vs CT-17EV."""

import dash
from dash import html
import dash_bootstrap_components as dbc

dash.register_page(__name__, path="/cars", name="Car Comparison")

layout = dbc.Container([
    html.H2("Car Comparison", className="mb-4"),
    dbc.Alert(
        "Compare CT-16EV (2025, 110S4P P45B) vs CT-17EV (2026, 100S4P P50B) "
        "on the same strategy and track. Available after simulation implementation.",
        color="info",
    ),
], fluid=True)
