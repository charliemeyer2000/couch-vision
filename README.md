# CouchVision

iOS app that streams iPhone sensor data to ROS2 over TCP, with perception, Nav2 path planning, and VESC motor control.

**Sensors:** Camera, LiDAR depth, IMU, GPS, magnetometer, barometer
**iOS Target:** iPhone 12 Pro+ (iOS 16+)
**ROS2:** Jazzy with CycloneDDS

## Platform Requirements

| Component | macOS | Jetson (Linux) |
|-----------|-------|----------------|
| iOS App (Xcode) | ✅ | ❌ |
| ROS2 Bridge | ✅ (dev) | ✅ (deploy) |
| Perception + Nav2 | ✅ (CPU) | ✅ (TensorRT) |
| VESC Motor Driver | ❌ | ✅ (requires /dev/ttyACM0) |
| Foxglove Viewer | ✅ | ❌ (headless) |

**macOS** is for iOS development (Xcode + Swift) and visualization (Foxglove app).
**Jetson Orin Nano** is the deployment target — runs ROS2, the TCP bridge, perception, and motor control. Accessible via `ssh jetson-nano` (Tailscale).

## Setup

### macOS

```bash
make setup
```

Installs: uv, xcodegen, SwiftLint, SwiftFormat, pre-commit, generates Xcode project. Optionally installs ROS2 Jazzy (Apple Silicon native) for local testing.

### Jetson Orin Nano

The Jetson (`ssh jetson-nano`) has ROS2 Jazzy built from source at `~/ros2_jazzy/` and this repo cloned at `~/couch-vision/`.

## Running

### Full stack on Jetson

Starts the iOS bridge + perception + Nav2 planner in Docker:

```bash
ssh jetson-nano
cd ~/couch-vision
make full-stack                       # perception + Nav2 only
make full-stack VESC=1                # with VESC motor driver
```

**`VESC=1` is required to run motors.** Without it, the VESC driver container is not started and `/cmd_vel` messages have no effect.

Monitor logs in separate terminals:
```bash
make logs-bridge    # iOS bridge
make logs-nav2      # Nav2 + perception
make logs-vesc      # VESC motor driver (only with VESC=1)
make logs           # all services interleaved
make stop           # bring everything down
```

Connect iPhone via USB-C to the Jetson. USB-C device mode gives iPhone `192.168.55.100`, Jetson `192.168.55.1`. In CouchVision app, connect to `tcp://192.168.55.1:7447`.

#### Full stack options

| Option | Default | Description |
|--------|---------|-------------|
| `VESC=1` | off | Enable VESC motor driver |
| `BAG=path.mcap` | (live mode) | Replay a recorded bag file (visualization only, no motor output) |
| `RATE=2.0` | 1.0 | Bag playback speed |
| `PREFIX=/name` | /iphone_charlie | Topic prefix for live mode |
| `SLAM_BACKEND=rtabmap` | none | SLAM algorithm (none or rtabmap) |
| `DEST_LAT`, `DEST_LON` | UVA grounds | Navigation destination |
| `LOOKAHEAD=15.0` | 15.0 | Path following lookahead (meters) |

Run `make full-stack HELP=1` for detailed help.

### Standalone bridge (Mac dev)

For local development without Docker:

```bash
make bridge         # start bridge on Mac (PORT=7447 default)
make xcode          # open Xcode, build to iPhone (Cmd+R)
```

Bridge prints local IP addresses on start. Connect the iPhone to `tcp://<mac_ip>:7447`.

### Visualize

Open [Foxglove](https://foxglove.dev) and connect to `ws://localhost:8765` (local Docker) or `ws://jetson-nano:8765` (remote Jetson).

Import `foxglove/couch_layout.json` for the default layout.

## Foxglove Panels

Two custom Foxglove extensions provide the control UI:

### Hardware Safety Panel

E-stop, teleop controls, motor telemetry, system status.

**Control modes:**
- **Gamepad** (default) — Sticks drive directly (no deadman button). Shows warning if no gamepad connected. WASD keys and mouse joystick disabled.
- **WASD** — Keyboard WASD + mouse drag joystick + gamepad sticks all active.
- **Nav2** — Teleop inputs disabled. Shows path follower status (active/goal reached/waiting) and inline tuning controls for speed, lookahead, and goal tolerance.

**Motor config:**
- **Max RPM** — ERPM safety clamp sent to VESC driver.
- **Ramp Up / Ramp Down (RPM/s)** — Separate slew rate limits for acceleration and deceleration. Prevents torque spikes. Default 500 RPM/s each. E-stop always bypasses the ramp for immediate stop.
- **Stop RPM** — ERPM threshold below which brake is released (motor considered stopped). Default 50. Braking uses COMM_SET_CURRENT_BRAKE (5A) which avoids VESC PID oscillation.

**E-stop sources:** Panel button, keyboard, gamepad B/Circle (arm with A/Cross). Gamepad e-stop/arm works in all modes.

### Nav Control Panel

Set navigation destinations on a map, view route status. Publishes `/nav/set_destination`, displays `/nav/status`.

### Building extensions

```bash
make build-extension    # build both panels
make install-extension  # install to local Foxglove
make lint-extension     # typecheck + lint + format check
```

## VESC Motor Hardware

- **ESC:** Flipsky Dual FSESC 6.7 (two controllers, one board)
- **PSU:** 25A, 24V power supply
- **USB:** Master controller only (STM32 VCP, `vid=0x0483 pid=0x5740`). Slave CAN ID 19.
- **Driver:** `perception/src/couch_perception/vesc_driver.py` — ROS2 node, differential drive, ERPM ramp rate, wheel odometry
- **Scripts:** `scripts/vesc_*.py` — standalone motor testing (WASD, hold tests, RPM tests)

See [CLAUDE.md](CLAUDE.md) for detailed VESC configuration, known firmware bugs, and tuning notes.

## Perception Stack

YOLOv8 (object detection) + YOLOP (drivable area / lane segmentation), feeding into Nav2 for path planning.

Auto-detects platform:
- **Jetson (aarch64):** NVIDIA runtime, CUDA + TensorRT inference
- **Mac/x86 (amd64):** CPU-only PyTorch inference

TensorRT engines are auto-exported on first CUDA run (~10-18 min on Orin). Engines persist in `perception/weights/` between container restarts. Delete `*.engine` to force re-export.

## Architecture

```
iPhone Sensors → Sensor Managers → SensorCoordinator → TCP
                                                        ↓
ROS2 Topics ← rclpy publishers ← ios_bridge.py ←←←←←←←←
                    ↓
              Perception (YOLO) → Costmap → Nav2 → /cmd_vel
                                                       ↓
                              Foxglove Panel ← /motor/status ← VESC Driver → Motors
                                            → /motor/config →      ↑
                                            → /e_stop ─────────────┘
```

## ROS2 Topics

### Sensor Topics (iPhone → Bridge)

Topic prefix is `/<device_name>/` (e.g. `/iphone_charlie/`).

| Topic suffix | Type | Rate |
|-------------|------|------|
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
| `thermal` | `std_msgs/Int32` | On change |
| `tf` | `tf2_msgs/TFMessage` | ~170 Hz |

### Navigation & Control Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Panel/Nav2 → VESC | Velocity commands |
| `/e_stop` | `std_msgs/Bool` | Panel → Planner + VESC | Emergency stop (`true`=stopped) |
| `/motor/config` | `std_msgs/String` | Panel → VESC | `{mode, max_rpm, stop_rpm, ramp_up_rpm_s, ramp_down_rpm_s}` |
| `/motor/status` | `std_msgs/String` | VESC → Panel | RPM, temps, voltage, faults, ramp config |
| `/motor/battery` | `sensor_msgs/BatteryState` | VESC → Panel | Combined ESC voltage, current, FET temperature |
| `/wheel_odom` | `nav_msgs/Odometry` | VESC → ROS2 | Wheel encoder odometry |
| `/teleop/status` | `std_msgs/String` | Panel → All | `{mode, active_source, gamepad_connected, e_stopped}` (2Hz) |
| `/nav/set_destination` | `std_msgs/String` | Nav Panel → Planner | JSON destination command |
| `/nav/status` | `std_msgs/String` | Planner → Panels | 1Hz JSON status |
| `/nav/planned_path` | `nav_msgs/Path` | Planner → Foxglove | Nav2-planned path |
| `/nav/google_maps_path` | `nav_msgs/Path` | Planner → Foxglove | Google Maps walking route |
| `/nav/path_follower/config` | `std_msgs/String` | Panel → Follower | `{linear_speed, lookahead, goal_tolerance, max_angular_vel}` |
| `/nav/path_follower/status` | `std_msgs/String` | Follower → Panel | Active state, path points, dist to goal |
| `/perception/image_annotated/compressed` | `sensor_msgs/CompressedImage` | Planner → Foxglove | Camera with detection overlays |

## Project Structure

```
CouchVision/              # iOS app (Swift)
├── App/                  # SwiftUI views (ContentView, SettingsView)
├── Core/                 # Coordinator, protocols, utilities
├── Sensors/              # Camera, LiDAR, Motion, Location, DeviceStatus managers
├── ROS/                  # Message structs, CDR encoder
├── Networking/           # TCP publisher
└── Resources/            # Info.plist
CouchVisionWidgets/       # iOS Live Activity widget
bridge/                   # Python iOS→ROS2 TCP bridge
perception/               # Python perception + planning (uv project)
├── configs/              # Pipeline presets (default, fast, accurate)
├── src/couch_perception/
│   ├── vesc_driver.py    # VESC motor driver (ROS2 node)
│   ├── path_follower.py  # Pure pursuit path follower (ROS2 node)
│   ├── nav2_planner.py   # Nav2 + EKF + perception orchestrator
│   ├── perception_pipeline.py
│   ├── yolov8_detector.py
│   ├── yolop_detector.py
│   ├── costmap.py
│   ├── ekf.py            # Extended Kalman Filter
│   ├── frame_source.py   # BagSource + LiveSource
│   ├── geo.py            # GPS/coordinate utilities
│   └── ...
└── tests/
foxglove/                 # Foxglove config + panel extensions
├── couch_layout.json     # Default panel layout
├── nav-control-panel/    # Navigation destination + map
└── hardware-safety-panel/ # E-stop, teleop, motor config
scripts/                  # Setup + VESC test scripts
urdf/                     # Robot description (iphone_sensor.urdf)
infra/                    # Terraform (S3 bag storage)
bags/                     # Recorded MCAP bag files
```

## Commands Reference

```bash
# Full stack (Docker, run on Jetson)
make full-stack                     # perception + Nav2
make full-stack VESC=1              # with motor driver
make full-stack BAG=bags/walk.mcap  # bag replay (visualization only)
make logs                           # tail all container logs
make logs-bridge                    # tail bridge logs
make logs-nav2                      # tail Nav2 logs
make logs-vesc                      # tail VESC driver logs
make stop                           # stop Docker stack

# Bridge (standalone, no Docker, run on Mac)
make bridge                         # start TCP bridge (PORT=7447)
make xcode                          # open Xcode project

# ROS2 debugging
make topics                         # list all topics
make hz T=/iphone_charlie/imu       # topic publish rate
make echo T=/iphone_charlie/odom    # print topic messages

# Foxglove extensions
make build-extension                # build both panels
make install-extension              # install to local Foxglove
make lint-extension                 # typecheck + lint + format check

# Development
make setup                          # install all dependencies
make test                           # run perception tests
make test ARGS='-k test_ekf'        # run specific tests
make bag                            # record all topics to MCAP
make lint                           # run all pre-commit linters
make clean                          # remove build artifacts + venvs
```

## Bag Files

Recorded ROS2 bags are stored in a public S3 bucket:

```bash
aws s3 cp s3://couch-vision-bags/<filename> .
# or curl/wget from https://couch-vision-bags.s3.amazonaws.com
```

## Pre-commit Hooks

Runs on commit: SwiftFormat, SwiftLint, ruff (Python), ESLint + Prettier (TypeScript).

Run manually: `pre-commit run --all-files`

The Swift hooks use system-installed SwiftFormat/SwiftLint. If CI fails but local hooks pass, run `make setup` to upgrade to matching versions.
