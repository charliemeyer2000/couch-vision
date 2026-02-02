# CouchVision — iOS Sensor Streamer for ROS2

SHELL := /bin/bash
PROJECT_ROOT := $(shell pwd)
BRIDGE_PORT ?= 7447
BAG ?= bags/2026-01-29_12-10-44/walk_around_university_all_data.mcap
BAG_DIR ?= bags

# ROS2 setup — searches common install locations, uses CycloneDDS
ROS2_SETUP := export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
			  export CYCLONEDDS_URI=file://$(realpath cyclonedds.xml); \
              source ~/ros2_jazzy/install/setup.bash 2>/dev/null || \
              source ~/ros2_ws/install/setup.bash 2>/dev/null || \
              source /opt/ros/jazzy/setup.bash 2>/dev/null || \
              source /opt/ros/humble/setup.bash 2>/dev/null || \
              (echo "Error: ROS2 not found. See README.md for install instructions." && exit 1)

# Auto-detect platform for correct Python version (Jetson = 3.10)
UNAME_S := $(shell uname -s)
UNAME_M := $(shell uname -m)
ifeq ($(UNAME_S)-$(UNAME_M),Linux-aarch64)
  PERC_PYTHON := python3.10
else
  PERC_PYTHON := python3.12
endif

.PHONY: help setup xcode bridge \
        topics hz echo rviz bag bag-play verify \
        perception-node full-stack \
        test benchmark lint deploy-jetson ip clean

help:
	@echo "CouchVision — iOS Sensor Streamer for ROS2"
	@echo ""
	@echo "Setup & iOS:"
	@echo "  make setup                            Full dev environment setup"
	@echo "  make xcode                            Open Xcode project"
	@echo ""
	@echo "Bridge:"
	@echo "  make bridge                           Run iOS→ROS2 bridge (PORT=$(BRIDGE_PORT))"
	@echo ""
	@echo "ROS2 Tools:"
	@echo "  make topics                           List ROS2 topics"
	@echo "  make hz T=/topic                      Show topic frequency"
	@echo "  make echo T=/topic                    Echo topic messages"
	@echo "  make rviz                             Launch RViz2"
	@echo "  make bag                              Record all topics"
	@echo "  make bag-play F=path                  Play back a bag file"
	@echo "  make verify BAG=path                  Verify bag contents"
	@echo ""
	@echo "Perception:"
	@echo "  make full-stack BAG=path.mcap         Perception + Nav2 planning (Docker, bag replay)"
	@echo "  make full-stack                       Live mode — subscribes to ROS2 topics"
	@echo "  make perception-node                  Run live perception ROS2 node (no Nav2)"
	@echo ""
	@echo "Testing:"
	@echo "  make test                             Run perception tests"
	@echo "  make benchmark                        Run component benchmarks"
	@echo ""
	@echo "Other:"
	@echo "  make lint                             Run all linters (pre-commit)"
	@echo "  make deploy-jetson                    Pull latest code on Jetson"
	@echo "  make ip                               Show Mac IP addresses"
	@echo "  make clean                            Clean build artifacts"

# === Setup ===

setup:
	@bash scripts/setup.sh

# === iOS ===

xcode:
	open CouchVision.xcodeproj

# === Bridge ===

bridge:
	@echo "Starting iOS bridge on port $(BRIDGE_PORT)..."
	@echo "Connect from iPhone using one of these addresses:"
	@ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print "  tcp://" $$2 ":$(BRIDGE_PORT)"}'
	@echo ""
	@$(ROS2_SETUP) && cd bridge && ([ -f .venv/pyvenv.cfg ] && grep -q "include-system-site-packages = true" .venv/pyvenv.cfg || uv venv --system-site-packages) && uv sync --quiet && uv run python ios_bridge.py --port $(BRIDGE_PORT)

# === ROS2 Tools ===

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

bag:
	@mkdir -p $(BAG_DIR)
	@$(ROS2_SETUP) && ros2 bag record -a -o $(BAG_DIR)/$$(date +%Y-%m-%d_%H-%M-%S)

bag-play:
	@$(ROS2_SETUP) && ros2 bag play $(F)

verify:
	uv run --with mcap,numpy,matplotlib python verify_bag.py $(BAG)

# === Perception ===

CONFIG ?=

perception-node:
	@$(ROS2_SETUP) && cd perception && ([ -f .venv/pyvenv.cfg ] && grep -q "include-system-site-packages = true" .venv/pyvenv.cfg || uv venv --python $(PERC_PYTHON) --system-site-packages) && uv sync --quiet && uv run python -m couch_perception.ros_node $(ARGS)

full-stack:
	@[ -f .env ] && set -a && . ./.env && set +a; \
	ARCH=$$(if [ "$$(uname -m)" = "aarch64" ]; then echo arm64; else echo amd64; fi); \
	RUNTIME=$$(if command -v nvidia-smi >/dev/null 2>&1; then echo nvidia; else echo runc; fi); \
	cd perception && \
	$(if $(BAG),BAG_FILE=$(patsubst bags/%,%,$(BAG)) PLAYBACK_RATE=$(or $(RATE),1.0),LIVE_MODE=1 TOPIC_PREFIX=$(or $(PREFIX),/iphone) NETWORK_MODE=host) \
	DEST_LAT=$(or $(DEST_LAT),38.036830) DEST_LON=$(or $(DEST_LON),-78.503577) LOOKAHEAD=$(or $(LOOKAHEAD),15.0) \
	DOCKER_RUNTIME=$$RUNTIME DOCKER_ARCH=$$ARCH \
	docker compose -f docker-compose.nav2.yml build --build-arg DOCKER_ARCH=$$ARCH && \
	$(if $(BAG),BAG_FILE=$(patsubst bags/%,%,$(BAG)) PLAYBACK_RATE=$(or $(RATE),1.0),LIVE_MODE=1 TOPIC_PREFIX=$(or $(PREFIX),/iphone) NETWORK_MODE=host) \
	DEST_LAT=$(or $(DEST_LAT),38.036830) DEST_LON=$(or $(DEST_LON),-78.503577) LOOKAHEAD=$(or $(LOOKAHEAD),15.0) \
	DOCKER_RUNTIME=$$RUNTIME DOCKER_ARCH=$$ARCH \
	docker compose -f docker-compose.nav2.yml up

# === Testing ===

test:
	cd perception && uv run --group dev pytest tests/ -v $(ARGS)

benchmark:
	cd perception && uv run --group dev pytest tests/test_benchmark.py -v --benchmark-enable $(ARGS)

# === Linting ===

lint:
	pre-commit run --all-files

# === Deploy ===

deploy-jetson:
	ssh jetson-nano 'cd ~/couch-vision && git pull'

# === Foxglove Extension ===

build-extension:
	cd foxglove/nav-control-panel && pnpm install && pnpm build

install-extension:
	cd foxglove/nav-control-panel && pnpm local-install

lint-extension:
	cd foxglove/nav-control-panel && pnpm typecheck && pnpm lint && pnpm format:check

# === Utilities ===

ip:
	@ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print $$2}'

clean:
	rm -rf build/ DerivedData/
	cd bridge && rm -rf .venv __pycache__
