"""Entry point: python -m dashboard"""

from dashboard.app import app

if __name__ == "__main__":
    app.run(debug=True, port=3000, host="0.0.0.0")
