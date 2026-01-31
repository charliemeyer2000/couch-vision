#!/bin/bash
set -e

# Source ROS2
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Launch Nav2 planner stack in background
echo "Starting Nav2 planner server..."
ros2 launch /perception/launch/nav2_planner.launch.py &
NAV2_PID=$!

# Give Nav2 time to initialize
sleep 5

# Run our planner node.
# Source the venv but keep ROS2 system packages on PYTHONPATH.
echo "Starting Nav2 planner runner..."
source /perception/.venv/bin/activate
export PYTHONPATH="/perception/src:${PYTHONPATH}"
python -m couch_perception.nav2_planner_runner "$@"

# Cleanup
kill $NAV2_PID 2>/dev/null || true
