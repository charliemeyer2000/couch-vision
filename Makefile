# CouchVision — iOS Sensor Streamer for ROS2

SHELL := /bin/bash
PROJECT_ROOT := $(shell pwd)

# === Configurable defaults ===
PORT ?= 7447
BAG ?= bags/walk.mcap
SLAM_BACKEND ?= rtabmap
JETSON_HOST ?= jetson-nano

# Derived values
SLAM_DOCKERFILE := $(if $(filter cuvslam,$(SLAM_BACKEND)),Dockerfile.cuvslam,Dockerfile.rtabmap)

# ROS2 setup — searches common install locations
ROS2_SETUP := export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
              export CYCLONEDDS_URI=file://$(realpath cyclonedds.xml); \
              source ~/ros2_jazzy/install/setup.bash 2>/dev/null || \
              source ~/ros2_ws/install/setup.bash 2>/dev/null || \
              source /opt/ros/jazzy/setup.bash 2>/dev/null || \
              source /opt/ros/humble/setup.bash 2>/dev/null || \
              (echo "Error: ROS2 not found" && exit 1)

# Platform detection
UNAME_M := $(shell uname -m)
PERC_PYTHON := $(if $(filter aarch64,$(UNAME_M)),python3.10,python3.12)

.PHONY: help setup xcode bridge topics hz echo bag verify \
        full-stack test benchmark lint deploy clean

# ═══════════════════════════════════════════════════════════════════════════════
# HELP
# ═══════════════════════════════════════════════════════════════════════════════

help:
	@echo "CouchVision — iOS Sensor Streamer for ROS2"
	@echo ""
	@echo "MAIN COMMANDS"
	@echo "  make full-stack BAG=<path>    Run perception + Nav2 + SLAM pipeline"
	@echo "  make bridge                   Run iOS→ROS2 TCP bridge"
	@echo "  make test                     Run perception tests"
	@echo ""
	@echo "  Add HELP=1 to any command for detailed options:"
	@echo "    make full-stack HELP=1"
	@echo "    make bridge HELP=1"
	@echo "    make test HELP=1"
	@echo ""
	@echo "SETUP"
	@echo "  make setup                    Install dependencies (Homebrew, ROS2, uv)"
	@echo "  make xcode                    Open iOS project in Xcode"
	@echo ""
	@echo "ROS2 DEBUGGING"
	@echo "  make topics                   List all ROS2 topics"
	@echo "  make hz T=/topic              Show publish rate of a topic"
	@echo "  make echo T=/topic            Print messages from a topic"
	@echo ""
	@echo "UTILITIES"
	@echo "  make bag                      Record all ROS2 topics to MCAP"
	@echo "  make verify BAG=<path>        Verify bag file contents"
	@echo "  make deploy                   Deploy code to Jetson"
	@echo "  make lint                     Run pre-commit linters"
	@echo "  make clean                    Remove build artifacts"

# ═══════════════════════════════════════════════════════════════════════════════
# FULL-STACK (main command)
# ═══════════════════════════════════════════════════════════════════════════════

ifeq ($(HELP),1)
full-stack:
	@echo "make full-stack — Perception + Nav2 + SLAM pipeline"
	@echo ""
	@echo "Runs YOLOv8 + YOLOP perception, EKF localization, Google Maps routing,"
	@echo "Nav2 path planning, and SLAM in Docker. Foxglove at ws://localhost:8765"
	@echo ""
	@echo "MODES"
	@echo "  make full-stack BAG=path.mcap    Replay recorded bag file"
	@echo "  make full-stack                  Live mode (subscribe to ROS2 topics)"
	@echo ""
	@echo "OPTIONS"
	@echo "  BAG=<path>              MCAP bag file path (omit for live mode)"
	@echo "  RATE=<float>            Playback speed multiplier (default: 1.0)"
	@echo "  PREFIX=<string>         Topic prefix for live mode (default: /iphone)"
	@echo "  SLAM_BACKEND=<type>     SLAM algorithm (default: rtabmap)"
	@echo "                            none    — No SLAM, static TF only"
	@echo "                            rtabmap — RTAB-Map (Mac + Jetson)"
	@echo "                            cuvslam — cuVSLAM + nvblox (Jetson only)"
	@echo "  DEST_LAT=<float>        Destination latitude (default: 38.036830)"
	@echo "  DEST_LON=<float>        Destination longitude (default: -78.503577)"
	@echo "  LOOKAHEAD=<float>       Path following lookahead in meters (default: 15.0)"
	@echo ""
	@echo "ENVIRONMENT"
	@echo "  GOOGLE_MAPS_API_KEY     Required for routing. Set in .env or export."
	@echo ""
	@echo "EXAMPLES"
	@echo "  make full-stack BAG=bags/walk.mcap"
	@echo "  make full-stack BAG=bags/walk.mcap RATE=2.0 SLAM_BACKEND=none"
	@echo "  make full-stack SLAM_BACKEND=cuvslam  # Jetson live mode"
else
full-stack:
	@[ -f .env ] && set -a && . ./.env && set +a; \
	ARCH=$$([ "$$(uname -m)" = "aarch64" ] && echo arm64 || echo amd64); \
	RUNTIME=$$(command -v nvidia-smi >/dev/null 2>&1 && echo nvidia || echo runc); \
	cd perception && \
	$(if $(BAG),BAG_FILE=$(patsubst bags/%,%,$(BAG)) PLAYBACK_RATE=$(or $(RATE),1.0),LIVE_MODE=1 TOPIC_PREFIX=$(or $(PREFIX),/iphone) NETWORK_MODE=host) \
	DEST_LAT=$(or $(DEST_LAT),38.036830) DEST_LON=$(or $(DEST_LON),-78.503577) LOOKAHEAD=$(or $(LOOKAHEAD),15.0) \
	SLAM_BACKEND=$(SLAM_BACKEND) SLAM_DOCKERFILE=$(SLAM_DOCKERFILE) \
	DOCKER_RUNTIME=$$RUNTIME DOCKER_ARCH=$$ARCH \
	docker compose -f docker-compose.nav2.yml build --build-arg DOCKER_ARCH=$$ARCH && \
	$(if $(BAG),BAG_FILE=$(patsubst bags/%,%,$(BAG)) PLAYBACK_RATE=$(or $(RATE),1.0),LIVE_MODE=1 TOPIC_PREFIX=$(or $(PREFIX),/iphone) NETWORK_MODE=host) \
	DEST_LAT=$(or $(DEST_LAT),38.036830) DEST_LON=$(or $(DEST_LON),-78.503577) LOOKAHEAD=$(or $(LOOKAHEAD),15.0) \
	SLAM_BACKEND=$(SLAM_BACKEND) SLAM_DOCKERFILE=$(SLAM_DOCKERFILE) \
	DOCKER_RUNTIME=$$RUNTIME DOCKER_ARCH=$$ARCH \
	docker compose -f docker-compose.nav2.yml up
endif

# ═══════════════════════════════════════════════════════════════════════════════
# BRIDGE
# ═══════════════════════════════════════════════════════════════════════════════

ifeq ($(HELP),1)
bridge:
	@echo "make bridge — iOS to ROS2 TCP bridge"
	@echo ""
	@echo "Receives sensor data from iPhone app over TCP and publishes to ROS2."
	@echo "Run this on your Mac, then connect iPhone to the displayed address."
	@echo ""
	@echo "OPTIONS"
	@echo "  PORT=<int>    TCP port to listen on (default: 7447)"
	@echo ""
	@echo "EXAMPLES"
	@echo "  make bridge"
	@echo "  make bridge PORT=9000"
else
bridge:
	@echo "Starting iOS bridge on port $(PORT)..."
	@echo "Connect iPhone to one of:"
	@ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print "  tcp://" $$2 ":$(PORT)"}'
	@echo ""
	@$(ROS2_SETUP) && cd bridge && \
	([ -f .venv/pyvenv.cfg ] && grep -q "include-system-site-packages = true" .venv/pyvenv.cfg || uv venv --system-site-packages) && \
	uv sync --quiet && uv run python ios_bridge.py --port $(PORT)
endif

# ═══════════════════════════════════════════════════════════════════════════════
# TEST
# ═══════════════════════════════════════════════════════════════════════════════

ifeq ($(HELP),1)
test:
	@echo "make test — Run perception unit tests"
	@echo ""
	@echo "Runs pytest on the perception module with verbose output."
	@echo ""
	@echo "OPTIONS"
	@echo "  ARGS=<string>    Additional pytest arguments"
	@echo ""
	@echo "EXAMPLES"
	@echo "  make test"
	@echo "  make test ARGS='-k test_ekf'           # Run only EKF tests"
	@echo "  make test ARGS='--tb=short'            # Shorter tracebacks"
	@echo "  make test ARGS='--benchmark-enable'    # Enable benchmarks"
else
test:
	cd perception && uv run --group dev pytest tests/ -v $(ARGS)
endif

benchmark:
	cd perception && uv run --group dev pytest tests/test_benchmark.py -v --benchmark-enable $(ARGS)

# ═══════════════════════════════════════════════════════════════════════════════
# SETUP & iOS
# ═══════════════════════════════════════════════════════════════════════════════

setup:
	@bash scripts/setup.sh

xcode:
	open CouchVision.xcodeproj

# ═══════════════════════════════════════════════════════════════════════════════
# ROS2 DEBUGGING
# ═══════════════════════════════════════════════════════════════════════════════

topics:
	@$(ROS2_SETUP) && ros2 topic list --no-daemon

hz:
ifndef T
	@echo "Usage: make hz T=/iphone/imu"
else
	@$(ROS2_SETUP) && ros2 topic hz $(T)
endif

echo:
ifndef T
	@echo "Usage: make echo T=/iphone/imu"
else
	@$(ROS2_SETUP) && ros2 topic echo $(T)
endif

# ═══════════════════════════════════════════════════════════════════════════════
# UTILITIES
# ═══════════════════════════════════════════════════════════════════════════════

bag:
	@mkdir -p bags
	@$(ROS2_SETUP) && ros2 bag record -a -o bags/$$(date +%Y-%m-%d_%H-%M-%S)

verify:
ifndef BAG
	@echo "Usage: make verify BAG=bags/walk.mcap"
else
	uv run --with mcap,numpy,matplotlib python verify_bag.py $(BAG)
endif

ifeq ($(HELP),1)
deploy:
	@echo "make deploy — Deploy code to Jetson"
	@echo ""
	@echo "Pulls latest git changes on the Jetson via SSH."
	@echo ""
	@echo "OPTIONS"
	@echo "  JETSON_HOST=<hostname>    SSH host (default: jetson-nano)"
	@echo ""
	@echo "EXAMPLES"
	@echo "  make deploy"
	@echo "  make deploy JETSON_HOST=jetson-orin"
else
deploy:
	ssh $(JETSON_HOST) 'cd ~/couch-vision && git pull'
endif

lint:
	pre-commit run --all-files

clean:
	rm -rf build/ DerivedData/
	cd bridge && rm -rf .venv __pycache__
	cd perception && rm -rf .venv __pycache__ .pytest_cache
