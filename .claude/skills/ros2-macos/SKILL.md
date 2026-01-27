---
name: ros2-macos
description: ROS2 Jazzy on macOS Apple Silicon. Use when running ROS2 commands, debugging DDS discovery, launching rviz2, or working with the CouchVision bridge.
---

# ROS2 on macOS (Jazzy, Apple Silicon)

## Environment Setup

ROS2 Jazzy is installed from source at `~/ros2_jazzy/`.

**Source the environment before any ROS2 command:**

```zsh
source ~/ros2_jazzy/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Both lines are required. The shell is zsh — do not use `setup.bash`.

## Critical: CycloneDDS Required

FastRTPS (the default RMW) DDS discovery hangs indefinitely on macOS. Always set:

```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

This must be set for **every process** — the bridge, `ros2 topic list`, `rviz2`, etc. If any ROS2 command hangs, the most likely cause is a missing or mismatched `RMW_IMPLEMENTATION`.

## Python Version

ROS2 was built against Python 3.12 in `~/ros2_jazzy/.venv/`. The `ros2` CLI shebang points there. System python may be newer (3.14) — do not use system python for ROS2 packages.

The bridge uses `uv` with `--system-site-packages` to access rclpy from the ROS2 install.

## Common Commands

All commands assume the environment is sourced.

```zsh
# List topics (use --no-daemon if daemon is broken)
ros2 topic list
ros2 topic list --no-daemon

# Echo a topic
ros2 topic echo /iphone/imu

# Topic frequency
ros2 topic hz /iphone/imu

# Launch rviz2
rviz2

# Node list
ros2 node list
```

Or use the Makefile which handles setup automatically:

```zsh
make bridge    # Start iOS bridge
make topics    # List topics
make rviz      # Launch rviz2
make hz T=/iphone/imu
make echo T=/iphone/imu
```

## ROS2 Daemon Issues

The `ros2` daemon can become stale and cause all CLI commands to hang. Fix:

```zsh
# Kill stale daemon
pkill -9 -f "_ros2_daemon"
# Or bypass it entirely
ros2 topic list --no-daemon
```

## CouchVision Bridge

The bridge (`bridge/ios_bridge.py`) receives CDR-encoded sensor data from the iPhone over TCP and republishes to ROS2 topics. It listens on port 7447.

**iPhone connection over USB-C:** The Mac gets a link-local IP on the `en*` USB interface (typically `169.254.x.x`). Use that IP in the app, not the WiFi IP. Run `ifconfig` to find it.

## Available RMW Implementations

- `rmw_cyclonedds_cpp` — works on macOS, use this
- `rmw_fastrtps_cpp` — default, discovery hangs on macOS
- `rmw_zenoh_cpp` — installed but requires zenoh router
