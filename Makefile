# CouchVision - iOS Sensor Streamer for ROS2
# Run `make help` to see available commands

SHELL := /bin/bash
PROJECT_ROOT := $(shell pwd)
BRIDGE_PORT ?= 7447

# ROS2 setup - searches common install locations (Jazzy and Humble)
# Uses CycloneDDS as FastRTPS discovery hangs on macOS
ROS2_SETUP := export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
              source ~/ros2_jazzy/install/setup.bash 2>/dev/null || \
              source ~/ros2_ws/install/setup.bash 2>/dev/null || \
              source /opt/ros/jazzy/setup.bash 2>/dev/null || \
              source /opt/ros/humble/setup.bash 2>/dev/null || \
              (echo "Error: ROS2 not found. See README.md for install instructions." && exit 1)

.PHONY: help setup setup-ros2 setup-bridge \
        build-ios build-sim xcode regen \
        bridge foxglove topics echo hz rviz rqt image \
        deploy-jetson ip check-ros2 clean \
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

foxglove:
	@echo "Starting Foxglove WebSocket bridge on port 8765..."
	@echo "Connect from Foxglove app: ws://localhost:8765"
	@echo ""
	@$(ROS2_SETUP) && ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

# === ROS2 Tools ===

topics:
	@$(ROS2_SETUP) && ros2 topic list

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
		-r image:=/iphone/camera/back_wide/image/raw \
		-p reliability:=best_effort

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
