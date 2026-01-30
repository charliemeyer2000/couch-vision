# For Mac/x86 testing: ros:jazzy
# For Jetson deployment: nvcr.io/nvidia/l4t-pytorch:r36.4.0-pth2.5-py3
#   then install ROS2 Jazzy on top, or use a multi-stage build.
FROM ros:jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-vision-msgs \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-numpy \
    python3-opencv \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install uv
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /usr/local/bin/

WORKDIR /perception

COPY pyproject.toml uv.lock ./
RUN uv venv --system-site-packages && uv sync

COPY src/ src/

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ENTRYPOINT ["/ros_entrypoint.sh", "uv", "run", "python3", "-m", "couch_perception.ros_node"]
CMD []
