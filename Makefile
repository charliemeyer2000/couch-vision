# CouchVision — iOS Sensor Streamer for ROS2

SHELL := /bin/bash
PROJECT_ROOT := $(shell pwd)

# === Configurable defaults ===
PORT ?= 7447
BAG ?=
SLAM_BACKEND ?= none
JETSON_HOST ?= jetson-nano

# ROS2 setup — searches common install locations, uses CycloneDDS
ROS2_SETUP := export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
              export CYCLONEDDS_URI=file://$(realpath cyclonedds.xml); \
              source ~/ros2_jazzy/install/setup.bash 2>/dev/null || \
              source ~/ros2_ws/install/setup.bash 2>/dev/null || \
              source /opt/ros/jazzy/setup.bash 2>/dev/null || \
              (echo "Error: ROS2 Jazzy not found. See README.md for install instructions." && exit 1)

# Platform detection
UNAME_M := $(shell uname -m)
PERC_PYTHON := $(if $(filter aarch64,$(UNAME_M)),python3.10,python3.12)

.PHONY: help setup xcode bridge topics hz echo rviz bag bag-play verify \
        perception-node full-stack test benchmark lint deploy clean \
        build-extension install-extension lint-extension ip \
        logs logs-bridge logs-nav2 stop

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
	@echo "  make rviz                     Launch RViz2 with project config"
	@echo ""
	@echo "UTILITIES"
	@echo "  make bag                      Record all ROS2 topics to MCAP"
	@echo "  make bag-play F=path          Play back a bag file"
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
	@echo "Runs YOLOv8 + YOLOP perception, GPS localization + phone heading, Google Maps routing,"
	@echo "Nav2 path planning, and SLAM in Docker. Foxglove at ws://localhost:8765"
	@echo ""
	@echo "MODES"
	@echo "  make full-stack BAG=path.mcap    Replay recorded bag file"
	@echo "  make full-stack                  Live mode (subscribe to ROS2 topics)"
	@echo ""
	@echo "OPTIONS"
	@echo "  BAG=<path>              MCAP bag file path (omit for live mode)"
	@echo "  RATE=<float>            Playback speed multiplier (default: 1.0)"
	@echo "  PREFIX=<string>         Topic prefix for live mode (default: /iphone_charlie)"
	@echo "  SLAM_BACKEND=<type>     SLAM algorithm (default: rtabmap)"
	@echo "                            none    — No SLAM, static TF only"
	@echo "                            rtabmap — RTAB-Map visual SLAM"
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
	@echo "  make full-stack SLAM_BACKEND=rtabmap  # Jetson live mode"
else
full-stack:
	@for f in .env perception/.env; do [ -f "$$f" ] && set -a && . "./$$f" && set +a; done; \
	ARCH=$$([ "$$(uname -m)" = "aarch64" ] && echo arm64 || echo amd64); \
	RUNTIME=$$([ -e /dev/nvidia0 ] && echo nvidia || echo runc); \
	PROFILE="$(if $(BAG),,--profile live)"; \
	$(if $(BAG),,echo ""; echo "=== LIVE MODE ==="; echo "Bridge + Nav2 stack starting together."; echo "Connect iPhone CouchVision app to one of these addresses:"; (ifconfig 2>/dev/null || ip -4 addr show) | grep "inet " | grep -v "127.0.0.1" | awk '{gsub("/[0-9]+","",$$2); print "  tcp://" $$2 ":7447"}'; echo "";) \
	cd perception && \
	$(if $(BAG),BAG_FILE=$(patsubst bags/%,%,$(BAG)) PLAYBACK_RATE=$(or $(RATE),1.0),LIVE_MODE=1 TOPIC_PREFIX=$(or $(PREFIX),/iphone_charlie) NETWORK_MODE=host) \
	DEST_LAT=$(or $(DEST_LAT),38.036830) DEST_LON=$(or $(DEST_LON),-78.503577) LOOKAHEAD=$(or $(LOOKAHEAD),15.0) \
	SLAM_BACKEND=$(SLAM_BACKEND) \
	DOCKER_RUNTIME=$$RUNTIME DOCKER_ARCH=$$ARCH \
	docker compose -f docker-compose.nav2.yml $$PROFILE build --build-arg DOCKER_ARCH=$$ARCH && \
	$(if $(BAG),BAG_FILE=$(patsubst bags/%,%,$(BAG)) PLAYBACK_RATE=$(or $(RATE),1.0),LIVE_MODE=1 TOPIC_PREFIX=$(or $(PREFIX),/iphone_charlie) NETWORK_MODE=host) \
	DEST_LAT=$(or $(DEST_LAT),38.036830) DEST_LON=$(or $(DEST_LON),-78.503577) LOOKAHEAD=$(or $(LOOKAHEAD),15.0) \
	SLAM_BACKEND=$(SLAM_BACKEND) \
	DOCKER_RUNTIME=$$RUNTIME DOCKER_ARCH=$$ARCH \
	docker compose -f docker-compose.nav2.yml $$PROFILE up -d && \
	echo "" && \
	echo "=== Stack running in background ===" && \
	echo "View logs in separate terminals:" && \
	echo "  make logs-bridge    # iOS bridge only" && \
	echo "  make logs-nav2      # Nav2 planner only" && \
	echo "  make logs           # Both (interleaved)" && \
	echo "  make stop           # Stop everything" && \
	echo ""
endif

logs:
	cd perception && docker compose -f docker-compose.nav2.yml logs -f --tail 50

logs-bridge:
	cd perception && docker compose -f docker-compose.nav2.yml logs -f --tail 50 ios-bridge

logs-nav2:
	cd perception && docker compose -f docker-compose.nav2.yml logs -f --tail 50 nav2-planner

stop:
	cd perception && docker compose -f docker-compose.nav2.yml down

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
	@echo "Connect from iPhone using one of these addresses:"
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

rviz:
	@$(ROS2_SETUP) && rviz2 -d $(PROJECT_ROOT)/rviz/couchvision.rviz

# ═══════════════════════════════════════════════════════════════════════════════
# PERCEPTION (standalone)
# ═══════════════════════════════════════════════════════════════════════════════

perception-node:
	@$(ROS2_SETUP) && cd perception && \
	([ -f .venv/pyvenv.cfg ] && grep -q "include-system-site-packages = true" .venv/pyvenv.cfg || uv venv --python $(PERC_PYTHON) --system-site-packages) && \
	uv sync --quiet && uv run python -m couch_perception.ros_node $(ARGS)

# ═══════════════════════════════════════════════════════════════════════════════
# UTILITIES
# ═══════════════════════════════════════════════════════════════════════════════

bag:
	@mkdir -p bags
	@$(ROS2_SETUP) && ros2 bag record -a -o bags/$$(date +%Y-%m-%d_%H-%M-%S)

bag-play:
	@$(ROS2_SETUP) && ros2 bag play $(F)

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

ip:
	@ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print $$2}'

clean:
	rm -rf build/ DerivedData/
	cd bridge && rm -rf .venv __pycache__
	cd perception && rm -rf .venv __pycache__ .pytest_cache

# ═══════════════════════════════════════════════════════════════════════════════
# FOXGLOVE EXTENSION
# ═══════════════════════════════════════════════════════════════════════════════

build-extension:
	cd foxglove/nav-control-panel && pnpm install && pnpm build

install-extension:
	cd foxglove/nav-control-panel && pnpm local-install

lint-extension:
	cd foxglove/nav-control-panel && pnpm typecheck && pnpm lint && pnpm format:check
