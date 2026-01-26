# CouchVision

iOS app that streams iPhone sensor data to ROS2 over TCP.

**Sensors:** Camera, LiDAR depth, IMU
**Target:** iPhone 12 Pro+ (iOS 16+), ROS2 Jazzy on macOS

## Setup

```bash
make setup
```

This installs dependencies (uv, xcodegen, SwiftLint, SwiftFormat, pre-commit) and generates the Xcode project.

For ROS2 (optional, needed for bridge):
```bash
make setup-ros2
```

## Running

**1. Start the bridge (Mac terminal):**
```bash
make bridge
```

**2. Run the iOS app:**
```bash
make xcode  # Opens Xcode, build to your iPhone (Cmd+R)
```

**3. Connect:**
- Get your Mac's IP: `make ip`
- In the app, enter `tcp://<mac-ip>:7447`
- Tap Connect, enable sensors, tap Start All

**4. Verify data:**
```bash
make topics           # List ROS2 topics
make hz T=/iphone/imu # Check IMU rate
make image            # View camera feed
```

## Development

### Project structure

```
CouchVision/
├── App/           # SwiftUI views
├── Core/          # Coordinator, protocols, utilities
├── Sensors/       # Camera, LiDAR, Motion managers
├── ROS/           # Message types, CDR encoder
└── Networking/    # Publisher implementations
bridge/            # Python ROS2 bridge
scripts/           # Setup scripts
```

### Commands

```bash
make lint           # Run all linters
make lint-fix       # Auto-fix lint issues
make format         # Format all code
make build-ios      # Build for device
make build-sim      # Build for simulator
make regen          # Regenerate Xcode project
```

### Adding a new sensor

1. Create `SensorManager` in `CouchVision/Sensors/`
2. Add `@Published` state and Combine publisher for data
3. Register in `SensorCoordinator` (subscribe to data, add to `forwardManagerChanges`)
4. Add UI toggle in `ContentView`

### Pre-commit hooks

Hooks run automatically on commit:
- SwiftFormat (formatting)
- SwiftLint (linting)
- ruff (Python linting/formatting)

To run manually: `pre-commit run --all-files`

## Architecture

```
iPhone Sensors → Sensor Managers → SensorCoordinator → ZenohPublisher → TCP
                                                                          ↓
ROS2 Topics ← rclpy publishers ← ios_bridge.py ←←←←←←←←←←←←←←←←←←←←←←←←←←
```

- **SensorCoordinator**: Central hub, manages sensor lifecycle and publishes data
- **Publisher protocol**: Abstraction for network layer (currently TCP, Zenoh planned)
- **CDREncoder**: Serializes ROS2 messages in CDR format

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/iphone/camera/back_wide/image/compressed` | `sensor_msgs/CompressedImage` | JPEG camera feed |
| `/iphone/lidar/depth/image` | `sensor_msgs/Image` | 32-bit float depth |
| `/iphone/lidar/points` | `sensor_msgs/PointCloud2` | XYZI point cloud |
| `/iphone/imu` | `sensor_msgs/Imu` | Fused orientation + accel + gyro |
