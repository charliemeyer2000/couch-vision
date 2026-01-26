# CouchVision - iOS Sensor Streamer for ROS2
# Run `make help` to see available commands

SHELL := /bin/bash
.SHELLFLAGS := -c
ROS2_SETUP := source ~/ros2_jazzy/install/setup.bash
BRIDGE_PORT ?= 7447

.PHONY: help build-ios build-sim xcode regen bridge topics echo hz rviz rqt image ip check-ros2 setup-bridge clean

help:
	@echo "CouchVision - iOS Sensor Streamer for ROS2"
	@echo ""
	@echo "iOS App:"
	@echo "  make build-ios    Build for device"
	@echo "  make build-sim    Build for simulator"
	@echo "  make xcode        Open Xcode project"
	@echo "  make regen        Regenerate Xcode project"
	@echo ""
	@echo "ROS2 Bridge:"
	@echo "  make setup-bridge Install Python dependencies"
	@echo "  make bridge       Run iOS bridge (PORT=7447)"
	@echo ""
	@echo "ROS2 Tools:"
	@echo "  make topics       List ROS2 topics"
	@echo "  make hz T=/topic  Show topic frequency"
	@echo "  make echo T=/topic Echo topic messages"
	@echo "  make rviz         Launch RViz2"
	@echo "  make rqt          Launch rqt"
	@echo "  make image        View camera in rqt_image_view"
	@echo ""
	@echo "Utilities:"
	@echo "  make ip           Show Mac IP addresses"
	@echo "  make check-ros2   Verify ROS2 installation"
	@echo "  make quickstart   Show quick start guide"

# === iOS App ===

build-ios:
	xcodebuild -project CouchVision.xcodeproj -scheme CouchVision -sdk iphoneos build

build-sim:
	xcodebuild -project CouchVision.xcodeproj -scheme CouchVision \
		-sdk iphonesimulator -destination 'platform=iOS Simulator,name=iPhone 17 Pro' build

xcode:
	open CouchVision.xcodeproj

regen:
	xcodegen generate

# === ROS2 Bridge ===

setup-bridge:
	cd bridge && uv sync

bridge:
	@echo "Starting iOS bridge on port $(BRIDGE_PORT)..."
	@echo "Waiting for iPhone to connect..."
	@$(ROS2_SETUP) && cd bridge && uv run ios_bridge.py --port $(BRIDGE_PORT)

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
	$(ROS2_SETUP) && rviz2

rqt:
	$(ROS2_SETUP) && rqt

image:
	$(ROS2_SETUP) && ros2 run rqt_image_view rqt_image_view /iphone/camera/back_wide/image/compressed

# === Utilities ===

ip:
	@echo "Your Mac's IP addresses:"
	@ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print $$2}'

check-ros2:
	@$(ROS2_SETUP) && ros2 -h > /dev/null && echo "ROS2 is installed and working!"

quickstart:
	@echo "=== CouchVision Quick Start ==="
	@echo ""
	@echo "1. Get your Mac's IP address:"
	@echo "   make ip"
	@echo ""
	@echo "2. Set up the bridge (first time only):"
	@echo "   make setup-bridge"
	@echo ""
	@echo "3. Start the ROS2 bridge (Terminal 1):"
	@echo "   make bridge"
	@echo ""
	@echo "4. Run iOS app on iPhone:"
	@echo "   make xcode  # then Cmd+R with iPhone selected"
	@echo ""
	@echo "5. In the iOS app:"
	@echo "   - Enter: tcp://<your-mac-ip>:7447"
	@echo "   - Tap Connect"
	@echo "   - Enable sensors, tap Start All"
	@echo ""
	@echo "6. View data (Terminal 2):"
	@echo "   make topics"
	@echo "   make hz T=/iphone/imu"
	@echo "   make image"

clean:
	rm -rf build/
	cd bridge && rm -rf .venv __pycache__
