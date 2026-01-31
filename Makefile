# CouchVision - iOS Sensor Streamer for ROS2
# Run `make help` to see available commands

SHELL := /bin/bash
PROJECT_ROOT := $(shell pwd)
BRIDGE_PORT ?= 7447

# ROS2 setup - searches common install locations (Jazzy and Humble)
# Uses CycloneDDS as FastRTPS discovery hangs on macOS
ROS2_SETUP := export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
			  export CYCLONEDDS_URI=file://$(realpath cyclonedds.xml); \
              source ~/ros2_jazzy/install/setup.bash 2>/dev/null || \
              source ~/ros2_ws/install/setup.bash 2>/dev/null || \
              source /opt/ros/jazzy/setup.bash 2>/dev/null || \
              source /opt/ros/humble/setup.bash 2>/dev/null || \
              (echo "Error: ROS2 not found. See README.md for install instructions." && exit 1)

.PHONY: help setup setup-ros2 setup-bridge \
        build-ios build-sim xcode regen \
        bridge foxglove topics echo hz rviz rqt image bag bag-play \
        deploy-jetson ip check-ros2 clean \
        perception perception-node perception-docker costmap costmap-docker nav2-planner full-stack \
        lint lint-fix format

# === Help ===

help:
	@echo "CouchVision - iOS Sensor Streamer for ROS2"
	@echo ""
	@echo "Setup (run once):"
	@echo "  make setup          Full dev environment setup"
	@echo "  make setup-ros2     Install ROS2 Jazzy (macOS Apple Silicon)"
	@echo "  make setup-bridge   Install bridge Python dependencies"
	@echo ""
	@echo "iOS App:"
	@echo "  make xcode          Open Xcode project"
	@echo "  make regen          Regenerate Xcode project"
	@echo "  make build-ios      Build for device"
	@echo "  make build-sim      Build for simulator"
	@echo ""
	@echo "ROS2 Bridge:"
	@echo "  make bridge         Run iOS bridge (PORT=$(BRIDGE_PORT))"
	@echo "  make foxglove       Start Foxglove WebSocket bridge (Jetson only)"
	@echo ""
	@echo "Jetson Deployment:"
	@echo "  make deploy-jetson  Pull latest code on Jetson via SSH"
	@echo ""
	@echo "ROS2 Tools:"
	@echo "  make topics         List ROS2 topics"
	@echo "  make hz T=/topic    Show topic frequency"
	@echo "  make echo T=/topic  Echo topic messages"
	@echo "  make rviz           Launch RViz2"
	@echo "  make rqt            Launch rqt"
	@echo "  make image          View camera in rqt_image_view"
	@echo "  make bag            Record all topics to bags/"
	@echo "  make bag-play F=x   Play back a bag file"
	@echo ""
	@echo "Perception:"
	@echo "  make perception BAG=path/to/bag.mcap  Run YOLOv8+YOLOP on bag"
	@echo "  make perception-node                  Run live perception ROS2 node"
	@echo "  make perception-docker                Run perception in Docker"
	@echo "  make costmap BAG=path/to/bag.mcap     Generate costmap from bag"
	@echo "  make costmap-docker BAG=bag.mcap       Run costmap in Docker"
	@echo "  make full-stack BAG=bag.mcap           Run perception + Nav2 planning (Docker)"
	@echo ""
	@echo "Linting & Formatting:"
	@echo "  make lint           Run all linters (via pre-commit)"
	@echo "  make lint-fix       Run linters with auto-fix"
	@echo "  make format         Format all code"
	@echo ""
	@echo "Utilities:"
	@echo "  make ip             Show Mac IP addresses"
	@echo "  make check-ros2     Verify ROS2 installation"
	@echo "  make quickstart     Show quick start guide"
	@echo "  make clean          Clean build artifacts"

# === Setup ===

setup:
	@bash scripts/setup.sh

setup-ros2:
	@bash scripts/setup-ros2.sh

setup-bridge:
	cd bridge && uv venv --system-site-packages && uv sync

# === iOS App ===

xcode:
	open CouchVision.xcodeproj

regen:
	xcodegen generate

build-ios:
	xcodebuild -project CouchVision.xcodeproj -scheme CouchVision -sdk iphoneos build

build-sim:
	xcodebuild -project CouchVision.xcodeproj -scheme CouchVision \
		-sdk iphonesimulator -destination 'platform=iOS Simulator,name=iPhone 17 Pro' build

# === ROS2 Bridge ===

bridge:
	@echo "Starting iOS bridge on port $(BRIDGE_PORT)..."
	@echo "Connect from iPhone using one of these addresses:"
	@ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print "  tcp://" $$2 ":$(BRIDGE_PORT)"}'
	@echo ""
	@$(ROS2_SETUP) && cd bridge && ([ -f .venv/pyvenv.cfg ] && grep -q "include-system-site-packages = true" .venv/pyvenv.cfg || uv venv --system-site-packages) && uv sync --quiet && uv run python ios_bridge.py --port $(BRIDGE_PORT)

bridge-docker:
	@echo "Starting iOS bridge in Docker on port $(BRIDGE_PORT)..."
	@echo "Connect from iPhone using one of these addresses:"
	@ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print "  tcp://" $$2 ":$(BRIDGE_PORT)"}'
	@echo ""
	cd bridge && docker compose up --build

foxglove:
	@echo "Starting Foxglove WebSocket bridge on port 8765..."
	@echo "Connect from Foxglove app: ws://localhost:8765"
	@echo ""
	@$(ROS2_SETUP) && ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

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

rqt:
	@$(ROS2_SETUP) && rqt

image:
	@$(ROS2_SETUP) && ros2 run image_tools showimage --ros-args \
		-r image:=/iphone_charlie/camera/arkit/image \
		-p reliability:=best_effort

BAG_DIR ?= bags

bag:
	@mkdir -p $(BAG_DIR)
	@$(ROS2_SETUP) && ros2 bag record -a -o $(BAG_DIR)/$$(date +%Y-%m-%d_%H-%M-%S)

bag-play:
	@$(ROS2_SETUP) && ros2 bag play $(F)

BAG ?= bags/2026-01-29_12-10-44/walk_around_university_all_data.mcap
verify:
	uv run --with mcap,numpy,matplotlib python verify_bag.py $(BAG)

# === Jetson Deployment ===

deploy-jetson:
	@echo "Deploying latest code to Jetson..."
	ssh jetson-nano 'cd ~/couch-vision && git pull'
	@echo ""
	@echo "Done. To run the bridge on Jetson:"
	@echo "  ssh jetson-nano 'cd ~/couch-vision && make bridge'"

# === Utilities ===

ip:
	@echo "Your Mac's IP addresses:"
	@ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print $$2}'

check-ros2:
	@$(ROS2_SETUP) && ros2 --version && echo "ROS2 is working!"

quickstart:
	@echo "=== CouchVision Quick Start ==="
	@echo ""
	@echo "First time setup:"
	@echo "  make setup"
	@echo ""
	@echo "Development workflow:"
	@echo ""
	@echo "1. Get your Mac's IP address:"
	@echo "   make ip"
	@echo ""
	@echo "2. Start the ROS2 bridge (Terminal 1):"
	@echo "   make bridge"
	@echo ""
	@echo "3. Run iOS app on iPhone:"
	@echo "   make xcode  # then Cmd+R with iPhone selected"
	@echo ""
	@echo "4. In the iOS app:"
	@echo "   - Enter: tcp://<your-mac-ip>:7447"
	@echo "   - Tap Connect"
	@echo "   - Enable sensors, tap Start All"
	@echo ""
	@echo "5. View data (Terminal 2):"
	@echo "   make topics"
	@echo "   make hz T=/iphone/imu"
	@echo "   make image"

# === Perception ===

# Auto-detect platform for correct Python version
UNAME_S := $(shell uname -s)
UNAME_M := $(shell uname -m)
ifeq ($(UNAME_S)-$(UNAME_M),Linux-aarch64)
  PERC_PYTHON := python3.10
else
  PERC_PYTHON := python3.12
endif

perception:
	cd perception && uv run couch-perception --bag $(abspath $(BAG)) --output output/

bev-projection:
	cd perception && uv run couch-bev-projection --bag $(abspath $(BAG)) $(ARGS)

perception-node:
	@$(ROS2_SETUP) && cd perception && ([ -f .venv/pyvenv.cfg ] && grep -q "include-system-site-packages = true" .venv/pyvenv.cfg || uv venv --python $(PERC_PYTHON) --system-site-packages) && uv sync --quiet && uv run python -m couch_perception.ros_node $(ARGS)

perception-docker:
	cd perception && docker compose up --build

costmap:
	cd perception && uv run couch-costmap --bag $(abspath $(BAG)) $(ARGS)

costmap-docker:
	cd perception && BAG_FILE=$(notdir $(BAG)) PLAYBACK_RATE=$(or $(RATE),1.0) docker compose -f docker-compose.costmap.yml up --build

nav2-planner:
	cd perception && BAG_FILE=$(notdir $(BAG)) PLAYBACK_RATE=$(or $(RATE),1.0) GOAL_X=$(or $(GX),5.0) GOAL_Y=$(or $(GY),0.0) docker compose -f docker-compose.nav2.yml up --build

full-stack:
	@[ -f .env ] && set -a && . ./.env && set +a; \
	cd perception && BAG_FILE=$(notdir $(BAG)) PLAYBACK_RATE=$(or $(RATE),1.0) DEST_LAT=$(or $(DEST_LAT),38.036830) DEST_LON=$(or $(DEST_LON),-78.503577) LOOKAHEAD=$(or $(LOOKAHEAD),15.0) docker compose -f docker-compose.nav2.yml up --build

# === Linting & Formatting ===

lint:
	@echo "Running all linters..."
	pre-commit run --all-files

lint-fix:
	@echo "Running linters with auto-fix..."
	cd bridge && uv run ruff check --fix .
	cd bridge && uv run ruff format .
	swiftlint --fix --quiet || true
	swiftformat . --quiet || true

format:
	@echo "Formatting all code..."
	cd bridge && uv run ruff format .
	swiftformat . --quiet || true

# === Cleanup ===

clean:
	rm -rf build/
	rm -rf DerivedData/
	cd bridge && rm -rf .venv __pycache__
