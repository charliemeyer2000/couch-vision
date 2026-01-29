FROM ros:jazzy

# Install CycloneDDS RMW and uv
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-numpy \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install uv
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /usr/local/bin/

WORKDIR /bridge

# Install Python deps with uv
COPY pyproject.toml uv.lock ./
RUN uv venv --system-site-packages && uv sync

COPY ios_bridge.py .
COPY cyclonedds.xml .

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///bridge/cyclonedds.xml

EXPOSE 7447

ENTRYPOINT ["/ros_entrypoint.sh", "uv", "run", "python3", "ios_bridge.py"]
CMD ["--port", "7447"]
