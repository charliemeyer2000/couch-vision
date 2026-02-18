#!/bin/bash
set -e

# Source ROS2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/jazzy/install/setup.bash ]; then
    source /opt/ros/jazzy/install/setup.bash
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source Python venv
if [ -f /perception/.venv/bin/activate ]; then
    source /perception/.venv/bin/activate
elif [ -f /opt/venv/bin/activate ]; then
    source /opt/venv/bin/activate
fi
export PYTHONPATH="/perception/src:${PYTHONPATH}"

exec python -m couch_perception.vesc_driver "$@"
