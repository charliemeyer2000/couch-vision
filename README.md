# CouchVision

iOS app that streams iPhone sensor data to ROS2 over TCP.

**Sensors:** Camera, LiDAR depth, IMU
**iOS Target:** iPhone 12 Pro+ (iOS 16+)
**ROS2:** Jazzy or Humble

## Platform Requirements

| Component | macOS | Jetson (Linux) |
|-----------|-------|----------------|
| iOS App (Xcode) | ✅ | ❌ |
| ROS2 Bridge | ✅ (dev) | ✅ (deploy) |
| Foxglove Bridge | ❌ | ✅ |
| Foxglove Viewer | ✅ | ❌ (headless) |

**macOS** is for iOS development (Xcode + Swift) and visualization (Foxglove app).
**Jetson Orin Nano** is the deployment target — runs ROS2, the TCP bridge, and Foxglove bridge. Accessible via `ssh jetson-nano` (Tailscale).

All contributors on the tailnet can access the Jetson with `ssh jetson-nano`.

## Setup

### macOS (iOS development)

```bash
make setup
```

Installs: uv, xcodegen, SwiftLint, SwiftFormat, pre-commit, generates Xcode project.

Optional — ROS2 for local testing (Apple Silicon native):
```bash
make setup-ros2
```

### Jetson Orin Nano (deployment)

The Jetson (`ssh jetson-nano`) has ROS2 Jazzy built from source at `~/ros2_jazzy/` and this repo cloned at `~/couch-vision/`.

**foxglove_bridge** is already built from source in the ROS2 workspace (the SDK binary only supports Linux aarch64, not macOS). The build required patching `rosx_introspection` for Jazzy compatibility — see [Agent Notes in SELF_DRIVING_STACK.md](SELF_DRIVING_STACK.md#agent-notes--learnings).

To deploy latest code to Jetson:
```bash
make deploy-jetson
```

## Running

### 1. Start the bridge

**On Mac (local dev):**
```bash
make bridge
```

**On Jetson (deployment):**
```bash
ssh jetson-nano
cd ~/couch-vision && make bridge
```

### 2. Run the iOS app (Mac)

```bash
make xcode  # Opens Xcode, build to iPhone (Cmd+R)
```

### 3. Connect

- Get bridge machine IP: `make ip` (Mac) or `ssh jetson-nano hostname -I`
- In the app: enter `tcp://<ip>:7447`
- Tap Connect, enable sensors, tap Start All

### 4. Verify

```bash
make topics
make hz T=/iphone/imu
```

### 5. Visualize with Foxglove

Start the Foxglove bridge on the Jetson:
```bash
ssh jetson-nano 'cd ~/couch-vision && make foxglove'
```

Then open the [Foxglove desktop app](https://foxglove.dev) on your Mac and connect to:
```
ws://jetson-nano:8765
```

All ROS2 topics on the Jetson are visible in Foxglove — compressed images, point clouds, IMU, GPS map, TF tree. No ROS2 installation needed on the viewing machine.

> **Note:** `foxglove_bridge` only runs on the Jetson (Linux). The Foxglove *viewer* app runs on any OS. `make rviz` is available for local RViz2 on machines with ROS2 installed.

## Bag Files

Recorded ROS2 bag files are stored in a public S3 bucket:

**https://couch-vision-bags.s3.amazonaws.com**

Download a bag:
```bash
aws s3 cp s3://couch-vision-bags/<filename> .
# or just curl/wget the URL directly
```

## Docker

### iOS Bridge
```bash
docker run --network host \
  -v $(pwd)/cyclonedds.xml:/bridge/cyclonedds.xml:ro \
  charliemeyer2000/couch-vision-bridge:latest
```

### Full Stack (Nav2 + Perception)
```bash
make full-stack BAG=bags/your_bag.mcap
# Connect Foxglove to ws://localhost:8765
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
perception/          # Python perception package (uv project)
├── configs/         # Pipeline presets (default, fast, accurate)
├── src/couch_perception/
│   ├── config.py            # PipelineConfig + YAML loading
│   ├── perception_pipeline.py
│   ├── yolov8_detector.py   # YOLOv8 (TensorRT > CUDA > MPS > CPU)
│   ├── yolop_detector.py    # YOLOP (TensorRT FP16 > CUDA > CPU)
│   ├── costmap_runner.py    # Costmap generation
│   ├── bev_projection_runner.py
│   ├── nav2_planner_runner.py
│   └── ...
└── tests/           # pytest + pytest-benchmark
scripts/             # Setup scripts
```

### Commands

```bash
# iOS (Mac only)
make xcode          # Open Xcode project

# ROS2 Bridge (Mac or Jetson)
make bridge         # Start iOS TCP bridge

# Perception
make perception BAG=path.mcap              # Run YOLOv8+YOLOP on bag
make costmap BAG=path.mcap                 # Generate costmap
make bev-projection BAG=path.mcap          # BEV point cloud projection
make full-stack BAG=path.mcap              # Perception + Nav2 (Docker)
make perception BAG=path.mcap CONFIG=perception/configs/fast.yaml  # Use preset

# Testing
make test           # Run perception tests
make benchmark      # Run component benchmarks

# Deployment
make deploy-jetson  # Pull latest code on Jetson

# Linting
make lint           # Run all linters (pre-commit)
```

### Adding a sensor

1. Create manager in `CouchVision/Sensors/`
2. Add `@Published` state and Combine publisher
3. Register in `SensorCoordinator`
4. Add UI toggle in `ContentView`

### Pre-commit hooks

Runs on commit: SwiftFormat, SwiftLint, ruff (Python).

Manual: `pre-commit run --all-files`

**Important:** The Swift hooks use your system-installed SwiftFormat/SwiftLint. CI uses the latest Homebrew versions, so your local versions must match. Run `make setup` periodically to stay in sync — it upgrades these tools automatically. If CI fails on formatting but local hooks pass, this version mismatch is likely the cause:

```bash
make setup        # upgrades swiftformat/swiftlint to latest
swiftformat .     # reformat with new version
```

## Architecture

```
iPhone Sensors → Sensor Managers → SensorCoordinator → TCP
                                                        ↓
ROS2 Topics ← rclpy publishers ← ios_bridge.py ←←←←←←←←
```

## ROS2 Topics

Topic prefix is `/<device_name>/` (configured in the iOS app, e.g. `/iphone_charlie/`).

| Topic suffix | Type | Rate |
|-------------|------|------|
| `camera/arkit/image` | `sensor_msgs/Image` | ~13 Hz |
| `camera/arkit/image/compressed` | `sensor_msgs/CompressedImage` | ~11 Hz |
| `camera/arkit/camera_info` | `sensor_msgs/CameraInfo` | ~30 Hz |
| `lidar/depth/image` | `sensor_msgs/Image` | ~30 Hz |
| `lidar/points` | `sensor_msgs/PointCloud2` | ~30 Hz |
| `imu` | `sensor_msgs/Imu` | ~100 Hz |
| `odom` | `nav_msgs/Odometry` | ~50 Hz |
| `gps/fix` | `sensor_msgs/NavSatFix` | ~1 Hz |
| `gps/velocity` | `geometry_msgs/TwistStamped` | ~1 Hz |
| `heading` | `std_msgs/Float64` | ~17 Hz |
| `magnetic_field` | `sensor_msgs/MagneticField` | ~100 Hz |
| `pressure` | `sensor_msgs/FluidPressure` | ~1 Hz |
| `altitude` | `std_msgs/Float64` | ~1 Hz |
| `battery` | `sensor_msgs/BatteryState` | ~1 Hz |
| `tf` | `tf2_msgs/TFMessage` | ~170 Hz |
