# CouchVision

iOS app that streams iPhone sensor data to ROS2 over TCP.

**Sensors:** Camera, LiDAR depth, IMU
**iOS Target:** iPhone 12 Pro+ (iOS 16+)
**ROS2:** Jazzy or Humble

## Platform Requirements

| Component | macOS | Linux | Windows |
|-----------|-------|-------|---------|
| iOS App (Xcode) | ✅ | ❌ | ❌ |
| ROS2 Bridge | ✅ | ✅ | WSL2 |

The iOS app requires macOS with Xcode. The ROS2 bridge runs anywhere with Python 3.11+ and ROS2.

## Setup

### macOS (Full Setup)

```bash
make setup
```

Installs: uv, xcodegen, SwiftLint, SwiftFormat, pre-commit, generates Xcode project.

For ROS2 (Apple Silicon native install):
```bash
make setup-ros2
```

### Linux (Bridge Only)

**Ubuntu 24.04 (Jazzy):**
```bash
# Install ROS2 Jazzy
sudo apt install ros-jazzy-desktop

# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# Setup bridge
cd bridge && uv sync
```

**Ubuntu 22.04 (Humble):**
```bash
# Install ROS2 Humble
sudo apt install ros-humble-desktop

# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# Setup bridge
cd bridge && uv sync
```

### Windows

Use WSL2 with Ubuntu, then follow Linux instructions.

## Running

### 1. Start the bridge

**macOS:**
```bash
make bridge
```

**Linux:**
```bash
source /opt/ros/jazzy/setup.bash  # or humble
cd bridge && uv run ios_bridge.py
```

### 2. Run the iOS app (macOS only)

```bash
make xcode  # Opens Xcode, build to iPhone (Cmd+R)
```

### 3. Connect

- Get bridge machine IP: `make ip` or `hostname -I`
- In the app: enter `tcp://<ip>:7447`
- Tap Connect, enable sensors, tap Start All

### 4. Verify

```bash
ros2 topic list
ros2 topic hz /iphone/imu
ros2 run rqt_image_view rqt_image_view /iphone/camera/back_wide/image/compressed
```

## Development

### Project structure

```
CouchVision/         # iOS app (Swift)
├── App/             # SwiftUI views
├── Core/            # Coordinator, protocols, utilities
├── Sensors/         # Camera, LiDAR, Motion managers
├── ROS/             # Message types, CDR encoder
└── Networking/      # Publisher implementations
bridge/              # Python ROS2 bridge
scripts/             # Setup scripts
```

### Commands (macOS)

```bash
make lint           # Run all linters
make lint-fix       # Auto-fix lint issues
make format         # Format all code
make build-ios      # Build for device
make regen          # Regenerate Xcode project
```

### Adding a sensor

1. Create manager in `CouchVision/Sensors/`
2. Add `@Published` state and Combine publisher
3. Register in `SensorCoordinator`
4. Add UI toggle in `ContentView`

### Pre-commit hooks

Runs on commit: SwiftFormat, SwiftLint, ruff (Python).

Manual: `pre-commit run --all-files`

## Architecture

```
iPhone Sensors → Sensor Managers → SensorCoordinator → TCP
                                                        ↓
ROS2 Topics ← rclpy publishers ← ios_bridge.py ←←←←←←←←
```

## ROS2 Topics

| Topic | Type | Rate |
|-------|------|------|
| `/iphone/camera/back_wide/image/compressed` | `sensor_msgs/CompressedImage` | 30 Hz |
| `/iphone/lidar/depth/image` | `sensor_msgs/Image` | 60 Hz |
| `/iphone/lidar/points` | `sensor_msgs/PointCloud2` | 60 Hz |
| `/iphone/imu` | `sensor_msgs/Imu` | 100 Hz |
