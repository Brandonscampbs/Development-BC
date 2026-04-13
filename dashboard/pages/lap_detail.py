"""Lap Detail page — deep dive into a single simulation run."""

import dash
from dash import html
import dash_bootstrap_components as dbc

dash.register_page(__name__, path="/lap-detail", name="Lap Detail")

layout = dbc.Container([
    html.H2("Lap Detail", className="mb-4"),
    dbc.Alert(
        "Segment-by-segment view of a simulation run: speed, torque, current, "
        "SOC, and thermal state over distance. "
        "Available after simulation implementation.",
        color="info",
    ),
], fluid=True)
