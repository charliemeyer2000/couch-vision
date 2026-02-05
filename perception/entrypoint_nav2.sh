#!/bin/bash
set -e

# Source ROS2 (path varies: ros:jazzy, dustynv, or Isaac ROS Humble)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/jazzy/install/setup.bash ]; then
    source /opt/ros/jazzy/install/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# GXF libraries for Isaac ROS (Jetson only, if installed)
# These are in subdirectories under the isaac_ros_gxf share folder
GXF_BASE="/opt/ros/humble/share/isaac_ros_gxf/gxf/lib"
if [ -d "$GXF_BASE" ]; then
    for subdir in core std serialization cuda multimedia npp network logger behavior_tree; do
        [ -d "$GXF_BASE/$subdir" ] && export LD_LIBRARY_PATH="$GXF_BASE/$subdir:$LD_LIBRARY_PATH"
    done
    export LD_LIBRARY_PATH="$GXF_BASE:$LD_LIBRARY_PATH"
fi

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
