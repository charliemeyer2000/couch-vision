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

# Force LE-only mode on the Bluetooth adapter.
# The Jetson's Realtek adapter is dual-mode (BR/EDR + LE). BlueZ defaults to
# BR/EDR connections for dual-mode peers (like the Mac), which fails because
# we only expose a GATT service. Setting ControllerMode=le fixes this.
# NOTE: btmgmt hangs inside Docker (HCI netlink issue). Configure the
# Bluetooth adapter on the host instead:
#   sudo btmgmt bredr off && sudo btmgmt le on
# This persists across container restarts.

exec python -m couch_perception.ble_bridge "$@"
