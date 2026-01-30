FROM python:3.12-slim

COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

WORKDIR /app
COPY pyproject.toml .
COPY src/ src/

RUN uv pip install --system .

ENTRYPOINT ["python", "-m", "couch_ekf.runner"]
