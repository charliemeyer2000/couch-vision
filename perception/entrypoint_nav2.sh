#!/bin/bash
set -e

# Source ROS2 (install path differs between ros:jazzy and dustynv images)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/jazzy/install/setup.bash ]; then
    source /opt/ros/jazzy/install/setup.bash
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Launch Nav2 planner stack in background
echo "Starting Nav2 planner server..."
ros2 launch /perception/launch/nav2_planner.launch.py &
NAV2_PID=$!

# Give Nav2 time to initialize (planner_server needs time to set up
# costmap plugins before lifecycle_manager tries to configure it)
sleep 10

# Run our planner node.
# Source the venv but keep ROS2 system packages on PYTHONPATH.
echo "Starting Nav2 planner..."
if [ -f /perception/.venv/bin/activate ]; then
    source /perception/.venv/bin/activate
elif [ -f /opt/venv/bin/activate ]; then
    source /opt/venv/bin/activate
fi
export PYTHONPATH="/perception/src:${PYTHONPATH}"

if [ -n "$LIVE_MODE" ]; then
    python -m couch_perception.nav2_planner \
        --topic-prefix "${TOPIC_PREFIX:-/iphone}" \
        --dest-lat "${DEST_LAT:-38.036830}" \
        --dest-lon "${DEST_LON:--78.503577}" \
        --lookahead "${LOOKAHEAD:-15.0}" \
        "$@"
else
    python -m couch_perception.nav2_planner "$@"
fi

# Cleanup
kill $NAV2_PID 2>/dev/null || true
