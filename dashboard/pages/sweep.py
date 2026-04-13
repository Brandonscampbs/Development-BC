"""Parameter Sweep page — visualize sweep results."""

import dash
from dash import html
import dash_bootstrap_components as dbc

dash.register_page(__name__, path="/sweep", name="Parameter Sweep")

layout = dbc.Container([
    html.H2("Parameter Sweep", className="mb-4"),
    dbc.Alert(
        "Visualize parameter sweep results: heatmaps, sensitivity plots, and "
        "optimal configurations. Available after sweep infrastructure is built.",
        color="info",
    ),
], fluid=True)
