#!/bin/bash
# Source this file to set up the CouchVision development environment
# Usage: source scripts/env.sh

# Find project root (parent of scripts dir)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export COUCH_VISION_ROOT="$(dirname "$SCRIPT_DIR")"

# ROS2 setup - try common locations
ROS2_LOCATIONS=(
    "$HOME/ros2_jazzy/install/setup.bash"
    "$HOME/ros2_ws/install/setup.bash"
    "/opt/ros/jazzy/setup.bash"
)

for loc in "${ROS2_LOCATIONS[@]}"; do
    if [[ -f "$loc" ]]; then
        source "$loc"
        echo "Sourced ROS2 from: $loc"
        break
    fi
done

if ! command -v ros2 &>/dev/null; then
    echo "Warning: ROS2 not found. Run 'make setup-ros2' to install."
fi
