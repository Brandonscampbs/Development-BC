"""FSAE EV Simulation Dashboard."""

import dash
from dash import html, dcc
import dash_bootstrap_components as dbc

app = dash.Dash(
    __name__,
    use_pages=True,
    pages_folder="pages",
    external_stylesheets=[dbc.themes.DARKLY],
    suppress_callback_exceptions=True,
    title="FSAE EV Sim",
)

sidebar = html.Div(
    [
        html.H4("FSAE EV Sim", className="text-light mb-2"),
        html.Hr(),
        html.P("CT-16EV / CT-17EV", className="text-muted small"),
        dbc.Nav(
            [
                dbc.NavLink("Overview", href="/", active="exact"),
                dbc.NavLink("Strategy Comparison", href="/strategy", active="exact"),
                dbc.NavLink("Car Comparison", href="/cars", active="exact"),
                dbc.NavLink("Parameter Sweep", href="/sweep", active="exact"),
                dbc.NavLink("Pareto Frontier", href="/pareto", active="exact"),
                dbc.NavLink("Lap Detail", href="/lap-detail", active="exact"),
            ],
            vertical=True,
            pills=True,
        ),
    ],
    className="bg-dark p-3",
    style={"height": "100vh", "position": "fixed", "width": "220px"},
)

content = html.Div(
    dash.page_container,
    style={"marginLeft": "240px", "padding": "20px"},
)

app.layout = html.Div([sidebar, content])

if __name__ == "__main__":
    app.run(debug=True, port=3000, host="0.0.0.0")
