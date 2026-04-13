FROM python:3.12-slim

WORKDIR /app

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies first (layer caching)
COPY pyproject.toml .
RUN pip install --no-cache-dir -e ".[dev]"

# Copy project
COPY . .

# Re-install in editable mode with full source
RUN pip install --no-cache-dir -e ".[dev]"

EXPOSE 3000

CMD ["python", "-m", "dashboard"]
