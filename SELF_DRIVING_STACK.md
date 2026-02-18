# Self-Driving Couch Project

## How To Use This Document

This is the **master tracking document** for the self-driving couch project. It is designed for iterative, agent-driven development:

1. **Read this entire document** to understand the high-level goal, architecture, what's been built, and what's left.
2. **Find the next unfinished phase/task** — look for unchecked `[ ]` items, starting from the earliest incomplete phase.
3. **Plan a chunk of work** — scope a reasonable unit (e.g. one phase, one critical gap, one subsystem). Enter plan mode, explore the codebase, and propose your implementation.
4. **Implement it** — write the code, test it (locally on Mac or via `ssh jetson-nano`).
5. **Update this document** — check off completed items `[x]`, add notes to the "Agent Notes & Learnings" section, record any gotchas or commands discovered.
6. **Create a pull request** — never commit directly to `main`. Create a feature branch, commit there, and open a PR. Label the PR with the phase/task it completes (e.g. `phase-1`, `phase-2-gps`). Only once the PR is merged into `main` should the next task begin.
7. **Repeat** — the next agent (or next session) re-reads this doc from `main` and picks up where the last merged PR left off.

**Branch & PR rules:**
- **Never commit directly to `main`.** All work goes through pull requests.
- Branch naming: `phase-N/short-description` (e.g. `phase-1/foxglove-layout`, `phase-2/gps-waypoints`)
- A task is only considered complete when its PR is merged into `main`.
- Do not start dependent work until the blocking PR is merged.

**Important for agents:**
- Always re-read `CLAUDE.md` for iOS app conventions and `SELF_DRIVING_STACK.md` for overall project state before starting work.
- The "Agent Notes & Learnings" section at the bottom is yours to write to. Add anything future agents would find useful.
- If you discover a new task or blocker, add it to the relevant phase.
- Ask the user before making major architectural decisions not already documented here.

---

## Project Overview

Building a self-driving couch using iPhones as the primary sensor platform, with a Jetson Orin Nano for computation, Nav2 for navigation, and GPU-accelerated perception.

**Hardware Platform:**
- Compute: NVIDIA Jetson Orin Nano (8GB shared GPU/CPU memory)
- Workstation: Desktop with RTX 5090 (32GB VRAM) — accessible via `ssh workstation`. For heavier ML models if needed.
- Sensors: iPhone 12 Pro+ (1 phone currently, expandable)
- Drive: Differential drive (two powered wheels, varying left/right speeds to turn)
- Motor Controllers: Flipsky Dual FSESC 6.7 (two independent VESC6 controllers on one PCB, CAN-linked)
- Motor: Flipsky 7070 110KV sensored outrunner BLDC (hall sensors, 7 pole pairs, 12N/14P)

**Software Stack:**
- ROS2 Jazzy
- Nav2 for planning and control (including GPS waypoint navigation)
- Perception: YOLOv8 on Jetson (DeepStream + TensorRT) for object detection
- SLAM: RTAB-Map visual SLAM
- iPhone sensor streaming via CouchVision iOS app (this repo)
- Transport: Zenoh (rmw_zenoh_cpp) over USB-C wired or WiFi
- Visualization: Foxglove (remote from Mac, bridge on Jetson)

**Environment:** Indoor + outdoor (GPS-based outdoor waypoint navigation planned)

---

## Development Environment

### Machines
- **Mac (local):** iOS development (Xcode/Swift), Claude Code, and visualization (Foxglove app). ROS2 Jazzy installed for local testing but Jetson is the primary ROS2 host.
- **Jetson Orin Nano:** Accessible via `ssh jetson-nano` (Tailscale). Has ROS2 Jazzy (built from source at `~/ros2_jazzy/`). Runs the TCP bridge, foxglove_bridge, and all ROS2 nodes. This repo is cloned at `~/couch-vision/`. This is the target compute platform on the couch.
- **Workstation:** Accessible via `ssh workstation` (Tailscale). RTX 5090 (32GB VRAM). For running larger ML models (VLAs, large detection models) if needed. Not part of the critical path.
- **iPhone 12 Pro+:** Single phone. Can connect to Mac (USB-C or WiFi) or directly to Jetson (USB-C).

All contributors on the tailnet can `ssh jetson-nano` and access the Jetson.

### Connectivity Options
```
Option A (development):   iPhone --USB-C--> Mac (bridge) --tailnet--> Jetson
Option B (deployment):    iPhone --USB-C--> Jetson (bridge, lowest latency)
Option C (wireless):      iPhone --WiFi--> Jetson (bridge, higher latency)

Visualization:            Jetson (foxglove_bridge:8765) --tailnet--> Mac (Foxglove app)
```

### Key Commands
```bash
# Build iOS app
xcodebuild -project CouchVision.xcodeproj -scheme CouchVision -sdk iphoneos build

# Build ROS2 workspace
cd ~/couch_ws && colcon build --symlink-install

# Source workspace
source ~/couch_ws/install/setup.bash

# Deploy latest code to Jetson
make deploy-jetson

# SSH to Jetson
ssh jetson-nano

# Launch full stack (once implemented)
ros2 launch couch_bringup couch_bringup.launch.py

# Teleop for testing
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Check TF tree
ros2 run tf2_tools view_frames

# Foxglove visualization (Linux/Jetson only)
make foxglove  # WebSocket bridge on port 8765, connect from Foxglove app

# Echo topics
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic hz /iphone/camera/back_wide/image/compressed

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/maps/house

# Send nav goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}}}"
```

---

## CouchVision App — Actual Topic Map

The iOS app publishes these topics. The topic prefix is configurable in-app (default `/iphone`). For multi-phone, each phone gets a different prefix (`/iphone1`, `/iphone2`).

| Topic | Message Type | Source | Rate | Notes |
|-------|-------------|--------|------|-------|
| `{prefix}/camera/{id}/image/compressed` | sensor_msgs/CompressedImage | CameraManager or LiDARManager (ARKit) | ~30 Hz | **JPEG**, not raw. `{id}` = `back_wide`, `front`, etc. |
| `{prefix}/camera/{id}/camera_info` | sensor_msgs/CameraInfo | CameraManager or LiDARManager | ~30 Hz | Intrinsics (estimated for CameraManager, real for LiDAR/ARKit) |
| `{prefix}/lidar/depth/image` | sensor_msgs/Image | LiDARManager | ~30 Hz | 32FC1 uncompressed depth in meters, 256x192 |
| `{prefix}/lidar/points` | sensor_msgs/PointCloud2 | LiDARManager | ~30 Hz | XYZI float32, already generated on-device |
| `/tf` | tf2_msgs/TFMessage | LiDARManager | ~30 Hz | ARKit pose: `world → iphone_base_link`, plus identity frames for lidar/camera |
| `{prefix}/imu` | sensor_msgs/Imu | MotionManager | 100 Hz | Quaternion orientation + angular vel + linear accel with covariance |
| `{prefix}/accelerometer` | geometry_msgs/Vector3Stamped | MotionManager | 100 Hz | Raw accelerometer (optional, off by default) |
| `{prefix}/gyroscope` | geometry_msgs/Vector3Stamped | MotionManager | 100 Hz | Raw gyroscope (optional, off by default) |
| `{prefix}/gps/fix` | sensor_msgs/NavSatFix | LocationManager | GPS rate | Lat/lon/alt with covariance |
| `{prefix}/gps/velocity` | geometry_msgs/TwistStamped | LocationManager | GPS rate | Speed + course → vx, vy |
| `{prefix}/heading` | std_msgs/Float64 | LocationManager | Heading events | Magnetic/true heading in degrees |
| `{prefix}/magnetic_field` | sensor_msgs/MagneticField | EnvironmentManager | 50 Hz | Tesla |
| `{prefix}/pressure` | sensor_msgs/FluidPressure | EnvironmentManager | Barometer rate | Pascals |
| `{prefix}/altitude` | std_msgs/Float64 | EnvironmentManager | Barometer rate | Relative altitude (meters) |
| `{prefix}/battery` | sensor_msgs/BatteryState | DeviceStatusManager | 1 Hz | |
| `{prefix}/thermal` | std_msgs/Int32 | DeviceStatusManager | On change | 0-3 (nominal to critical) |
| `{prefix}/proximity` | std_msgs/Bool | DeviceStatusManager | On change | |
| `{prefix}/odom` | nav_msgs/Odometry | LiDARManager | ~30 Hz | ARKit VIO pose + velocity from pose differencing. Covariance included. |

### Critical Gap: Compressed vs Raw Images

The app publishes **JPEG CompressedImage**, but RTAB-Map expects raw `sensor_msgs/Image`. Options:
1. **Jetson-side:** Use `image_transport` `republish` node to decompress JPEG → raw
2. **iOS-side:** Add a raw image publishing mode (higher bandwidth but no decompression needed)
3. **RTAB-Map:** Can accept compressed images directly with `image_transport` plugin

Option 3 is simplest for RTAB-Map.

### Coordinate System

- ARKit uses (right, up, back). App converts to ROS (forward, left, up) in LiDARManager.
- Point cloud coordinates: `(depth, -ptX, -ptY, intensity)` — already in ROS convention.
- TF frames published: `world`, `iphone_base_link`, `iphone_lidar`, `iphone_camera_arkit`

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              iPHONE LAYER                                   │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │  iPhone (CouchVision app)                                            │  │
│  │                                                                      │  │
│  │  Topics published:                                                   │  │
│  │  /iphone/camera/back_wide/image/compressed  (JPEG, ~30Hz)           │  │
│  │  /iphone/camera/back_wide/camera_info       (intrinsics)            │  │
│  │  /iphone/lidar/depth/image                  (32FC1, 256×192)        │  │
│  │  /iphone/lidar/points                       (PointCloud2 XYZI)      │  │
│  │  /iphone/imu                                (100Hz, with covariance)│  │
│  │  /iphone/gps/fix                            (NavSatFix)             │  │
│  │  /iphone/odom                               (nav_msgs/Odometry)     │  │
│  │  /tf                                        (ARKit 6DOF pose)       │  │
│  └──────────────────────────────┬────────────────────────────────────────┘  │
│                                 │ USB-C / WiFi / Zenoh                      │
└─────────────────────────────────┼──────────────────────────────────────────┘
                                  │
                                  ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                        JETSON ORIN NANO                                      │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                    BRIDGE / PREPROCESSING LAYER                        │ │
│  │                                                                        │ │
│  │  ┌──────────────────────┐    ┌──────────────────────────────────────┐  │ │
│  │  │ image_transport      │    │ navsat_transform_node               │  │ │
│  │  │ republish node       │    │ /iphone/gps/fix → UTM local frame  │  │ │
│  │  │                      │    │ Datum: first GPS fix or configured  │  │ │
│  │  │ compressed → raw     │    │                                    │  │ │
│  │  │ (only if SLAM needs  │    └──────────────────────────────────────┘  │ │
│  │  │  raw images)         │                                              │ │
│  │  └──────────────────────┘                                              │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                      PERCEPTION LAYER                                  │ │
│  │                                                                        │ │
│  │  ┌──────────────────┐    ┌─────────────────────────────────────────┐  │ │
│  │  │ YOLOv8n (INT8)   │    │ pointcloud_to_laserscan                │  │ │
│  │  │ DeepStream +     │    │ /iphone/lidar/points → /scan           │  │ │
│  │  │ TensorRT         │    │ (LaserScan for Nav2 costmap)           │  │ │
│  │  │                  │    └─────────────────────────────────────────┘  │ │
│  │  │ Detects:         │                                                  │ │
│  │  │ - pedestrians    │                                                  │ │
│  │  │ - vehicles       │                                                  │ │
│  │  │ - stop signs     │                                                  │ │
│  │  │ - lanes          │                                                  │ │
│  │  │                  │                                                  │ │
│  │  │ Output:          │                                                  │ │
│  │  │ /detections      │                                                  │ │
│  │  │ (Detection2D     │                                                  │ │
│  │  │  Array)          │                                                  │ │
│  │  └──────────────────┘                                                  │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                      SENSOR FUSION LAYER                               │ │
│  │                                                                        │ │
│  │  ┌──────────────────┐    ┌─────────────────────────────────────────┐  │ │
│  │  │ TF Manager       │    │ robot_localization (EKF)                │  │ │
│  │  │                  │    │                                         │  │ │
│  │  │ Static TFs:      │    │ Fuses:                                  │  │ │
│  │  │ - odom→base_link │    │ - /iphone/odom (ARKit VIO)              │  │ │
│  │  │ - base_link→     │    │ - /wheel_odom (from encoders)           │  │ │
│  │  │   iphone_link    │    │ - /iphone/imu                           │  │ │
│  │  └──────────────────┘    │ - /iphone/gps/fix (via navsat)          │  │ │
│  │                          │                                         │  │ │
│  │                          │ Output: /odometry/filtered               │  │ │
│  │                          └─────────────────────────────────────────┘  │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                         SLAM LAYER                                     │ │
│  │                                                                        │ │
│  │  RTAB-Map (works with compressed images via image_transport)           │ │
│  │                                                                        │ │
│  │  Inputs:                              Outputs:                         │ │
│  │  - /iphone/camera/.../image/compressed  - /map (OccupancyGrid)        │ │
│  │  - /iphone/lidar/depth/image            - map→odom transform          │ │
│  │  - /iphone/camera/.../camera_info                                     │ │
│  │  - /odometry/filtered                                                 │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                         NAV2 LAYER                                     │ │
│  │                                                                        │ │
│  │  Two modes:                                                            │ │
│  │  1. Indoor: Costmap2D → SmacPlanner → DWB Controller                  │ │
│  │  2. Outdoor GPS: GPS waypoint follower → nav_msgs/Path                │ │
│  │     navsat_transform_node converts WGS84 → UTM local frame            │ │
│  │                                                                        │ │
│  │  Output: /cmd_vel (Twist)                                              │ │
│  │    linear.x  = forward velocity                                        │ │
│  │    angular.z = rotation velocity                                       │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
│                                                                              │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                      MOTOR CONTROL LAYER                               │ │
│  │                                                                        │ │
│  │  Differential Drive Controller (ros2_control)                          │ │
│  │  /cmd_vel → Kinematics → Wheel velocities → Hardware Interface        │ │
│  │                                                                        │ │
│  │  v_left  = linear.x - (angular.z * wheel_separation / 2)              │ │
│  │  v_right = linear.x + (angular.z * wheel_separation / 2)              │ │
│  │                                                                        │ │
│  │  Publishes: /wheel_odom (from encoders)                                │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## Implementation Phases

Phases are ordered so software work can proceed in parallel with hardware procurement. Phases 1-3 are **software-only** and unblocked right now. Phase 0 (hardware) can happen in parallel whenever parts arrive.

### Phase 0: Hardware Setup ⚙️
**Goal:** Couch drives manually via teleop

**Status:** In Progress — ESC and motor verified, programmatic control working

#### Hardware Selected
- **ESC:** Flipsky Dual FSESC 6.7 (two independent VESC6 controllers on one PCB, internal CAN bus)
  - 100A continuous per side, 8V-60V input, FOC/BLDC/DC modes
  - USB shows as `STMicroelectronics Virtual COM Port` at `/dev/ttyACM0`
  - Firmware: v5.02, HW 60 (Flipsky variant)
  - Correct custom firmware target: `flipsky_60` or `flipsky_60_mk5` (built from `~/bldc/`)
- **Motor:** Flipsky 7070 110KV sensored outrunner BLDC (one motor currently, on one side of dual ESC)
  - 7 pole pairs (12N/14P), hall sensors + temperature sensor
  - Internal resistance: 0.055 ohm, max 100A, 4200W peak
  - ERPM = mechanical RPM × 7 (e.g. 200 RPM = 1400 ERPM)
- **Current power:** Bench PSU at 31.8V, 5A limit (temporary — production will use battery)

#### Tasks
- [x] Select and purchase motors, wheels, motor controllers
- [x] Wire motor to ESC (3 phase wires + hall sensor JST connector)
- [x] Connect ESC to Jetson via USB, verify serial communication
- [x] Install VESC Tool on Jetson (Flatpak aarch64 from Flathub)
- [x] Run FOC motor detection and hall sensor detection in VESC Tool
- [x] Verify programmatic motor control from Python over USB serial (`~/bldc/spin_test.py`)
- [x] Verify telemetry readback (RPM, current, temperature, voltage, duty cycle, fault codes)
- [ ] Mount motor + wheel on couch frame
- [ ] Wire second motor to other side of dual ESC, run motor detection on slave side
- [ ] Set up CAN forwarding: master (CAN ID 0) controls slave (CAN ID 1)
- [ ] Implement ROS2 VESC driver node (subscribes `/cmd_vel`, sends VESC UART commands)
- [ ] Implement differential drive kinematics in driver node
- [ ] Test with `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- [ ] Add wheel odometry from VESC tachometer (hall sensor counts)
- [ ] Verify `/wheel_odom` publishes correctly
- [ ] Measure and document wheel separation and wheel radius
- [ ] Determine power system (batteries, voltage regulators)
- [ ] Set production current limits in VESC Tool (motor max, battery max, brake current)

#### Hardware Specs (Fill in as determined)
```yaml
wheel_separation: _____ m        # distance between left/right wheel centers
wheel_radius: _____ m            # wheel radius
max_wheel_velocity: _____ rad/s  # motor max speed
motor_pole_pairs: 7              # Flipsky 7070
motor_kv: 110                    # RPM per volt
esc_firmware: "v5.02"            # VESC firmware version
esc_hw: "HW 60"                  # Flipsky VESC6 variant
```

#### VESC Communication Protocol (for driver node implementation)
The VESC uses a binary UART protocol over USB serial (`/dev/ttyACM0` at 115200 baud).
A working reference implementation exists at `~/bldc/spin_test.py`.

**Packet format:**
```
[0x02][length:1][payload][crc16:2][0x03]     # short packet (payload < 256 bytes)
[0x03][length:2][payload][crc16:2][0x03]     # long packet (payload >= 256 bytes)
```
CRC is CRC-CCITT (poly 0x1021, init 0x0000) over the payload only.

**Key command IDs (payload byte 0):**
| ID | Command | Payload | Notes |
|----|---------|---------|-------|
| 4  | `COMM_GET_VALUES` | (none) | Returns telemetry (see below) |
| 5  | `COMM_SET_DUTY` | int32 (duty × 100000) | Open-loop, good for low speed |
| 6  | `COMM_SET_CURRENT` | int32 (amps × 1000) | Torque control mode |
| 7  | `COMM_SET_CURRENT_BRAKE` | int32 (amps × 1000) | Regen braking |
| 8  | `COMM_SET_RPM` | int32 (ERPM) | PID speed control using hall sensors |

**COMM_GET_VALUES response layout (payload after cmd byte 4):**
| Offset | Size | Field | Scale | Unit |
|--------|------|-------|-------|------|
| 1 | 2 | temp_fet | /10 | °C |
| 3 | 2 | temp_motor | /10 | °C |
| 5 | 4 | avg_motor_current | /100 | A |
| 9 | 4 | avg_input_current | /100 | A |
| 13 | 4 | avg_id | /100 | A |
| 17 | 4 | avg_iq (torque current) | /100 | A |
| 21 | 2 | duty_cycle | /1000 | ratio |
| 23 | 4 | erpm | /1 | ERPM |
| 27 | 2 | v_in | /10 | V |
| 29 | 4 | amp_hours | /10000 | Ah |
| 33 | 4 | amp_hours_charged | /10000 | Ah |
| 37 | 4 | watt_hours | /10000 | Wh |
| 41 | 4 | watt_hours_charged | /10000 | Wh |
| 45 | 4 | tachometer | raw | hall counts |
| 49 | 4 | tachometer_abs | raw | hall counts |
| 53 | 1 | fault_code | enum | 0=NONE |

**Important timing:** Commands must be re-sent within 1000ms (VESC timeout). The spin_test script sends every 50ms. Telemetry reads take ~20ms; always re-send the motor command immediately after a telemetry read to avoid stuttering.

**Wheel odometry from tachometer:** The tachometer counts hall sensor transitions. To convert to distance:
```
wheel_revs = tachometer / (pole_pairs * 6)    # 6 hall transitions per electrical revolution
distance = wheel_revs * (2 * pi * wheel_radius)
```

#### Deliverable
Couch drives with keyboard control, `/wheel_odom` publishes from VESC tachometer

---

### Phase 1: Sensor Verification & Visualization
**Goal:** iPhone data streaming end-to-end, visualized in Foxglove

**Status:** In Progress — all sensor topics verified including PointCloud2, Foxglove layout complete

The iOS app is built and streaming 18 topics. The bridge and foxglove_bridge are running on Jetson. This phase is about verifying data quality and building a proper visualization layout.

#### Tasks
- [x] Deploy CouchVision app to iPhone
- [x] TCP bridge running on Jetson (`make bridge`)
- [x] Foxglove bridge running on Jetson (`make foxglove`)
- [x] Foxglove app on Mac connects to Jetson (`ws://jetson-nano:8765`)
- [x] Backpressure handling for slow connections (publishGated pattern)
- [x] **Create Foxglove layout** (`foxglove/couch_layout.json`) with panels for:
  - [x] 3D panel — point cloud (`/iphone/lidar/points`), TF tree, couch bounding box
  - [x] Image panel — camera feed (`/iphone/camera/back_wide/image/compressed`)
  - [x] Map panel — GPS position (`/iphone/gps/fix`)
  - [x] IMU/diagnostics plots (orientation, velocity)
  - [x] Topic graph panel
- [x] **Publish couch bounding box marker** — `visualization_msgs/Marker` from bridge on `/couch/marker` at `base_link`
- [x] Verify topic data quality (via `make verify`, see `verify_bag.py`):
  - [x] `/iphone/camera/arkit/image/compressed` — 10.5–18.7 Hz effective, clear JPEG frames
  - [x] `/iphone/lidar/points` — 8,094 messages at 38.6 Hz, XYZI float32, valid depth range 0.1–5.9m
  - [x] `/iphone/imu` — 98.9 Hz effective, orientation/angular vel/linear accel all reasonable
  - [x] `/iphone/gps/fix` — 1 Hz, 156 fixes, clear walk path on scatter plot
  - [x] `/tf` — 27876 messages, ARKit pose transforms present
  - [x] `/iphone/odom` — 22.7 Hz, ~100m walk trail visible
- [ ] Measure end-to-end latency
- [ ] Mount iPhone rigidly on couch (front-facing)
- [ ] Measure iPhone mount position/orientation relative to base_link
- [ ] Configure static TF: `base_link → iphone_link` on Jetson

#### iPhone Mount Transform (Fill in)
```yaml
# Static TF: base_link → iphone_link
x: _____      # meters forward from base_link origin
y: _____      # meters left
z: _____      # meters up
roll: _____   # radians
pitch: _____
yaw: _____
```

#### Deliverable
Foxglove layout showing live camera, point cloud, GPS on map, IMU plots, and couch bounding box. Saved as a reusable `.json` layout file.

---

### Phase 2: GPS Waypoint Navigation 🗺️
**Goal:** Given two GPS coordinates (start, destination), plan and visualize a path

**Status:** In Progress — Google Maps routing integrated into nav2_planner, EKF consolidated into perception package

**Prerequisites:** Phase 1 (GPS streaming verified in Foxglove)

**Does NOT require hardware** — path planning and visualization can be tested without motors. The couch just won't move yet.

#### Existing Work: `nav2_planner.py` (PR #12)
Google Maps routing is now integrated into the full Nav2 planning pipeline (`perception/src/couch_perception/nav2_planner.py`). It:
- Calls Google Maps Directions API for a walking route between two GPS points
- Converts GPS waypoints to local ENU frame (origin: UVA Rotunda `38.035853,-78.503307`)
- Runs full perception pipeline (YOLOv8 + YOLOP) to build ego-centric costmaps
- Feeds costmaps into Nav2 planner server (running in Docker) to generate obstacle-aware paths
- Publishes via ROS2: `/map/costmap` (OccupancyGrid), `/plan` (nav_msgs/Path), `/goal_pose`, sensor data
- Uses EKF (consolidated in `perception/src/couch_perception/ekf.py`) for IMU+GPS fusion
- Runs in Docker via `make full-stack` with Nav2 + foxglove_bridge + perception
- Requires `GOOGLE_MAPS_API_KEY` env var (stored in `.env`, gitignored)

The standalone `nav/` directory has been deleted — all routing logic lives in nav2_planner now.

#### Architecture
```
/iphone/gps/fix (NavSatFix)
       │
       ▼
navsat_transform_node          ← converts WGS84 → UTM local frame
       │                         uses a GPS datum as world origin
       ▼
robot_localization (EKF)       ← fuses GPS + IMU + ARKit odom
       │
       ▼
/odometry/filtered             ← couch position in local frame
       │
       ▼
Nav2 GPS waypoint follower     ← accepts GPS waypoints, converts to local goals
       │
       ▼
/plan (nav_msgs/Path)          ← visualized in Foxglove 3D panel
       │
       ▼
/cmd_vel (Twist)               ← drives motors (Phase 0 needed for actual motion)
```

#### Tasks


HUMAN WRITTEN INSTRUCTIONS:

---

Make your own ekf using numpy in map frame
Read the data from the bag file -> tell it the topics
Convert navsat fix to enu relative to the rotunda
Use imu and gps data for localization
ROTUNDA = "38.035853,-78.503307"
Output localization at 100 hertz
Create a dashboard showing the covariance over time and the localized path using matplotlib

we can test this locally on the macbook we don't need to do this using the `jetson-nano`, so let's get this going. we have some bags you can use to test this with, bags/university_intersect_gps_only.mcap is gps only good walk, and then we have bags/2026-01-29_12-10-44/walk_around_university_all_data.mcap that is the same one but all data (lidar, camera, etc).

get this working please.

---


- [x] Build custom EKF (numpy) fusing IMU + GPS in map frame (now at `perception/src/couch_perception/ekf.py`)
- [x] Build UKF variant for comparison (removed — EKF sufficient)
- [x] Convert NavSatFix to ENU relative to rotunda (38.035853, -78.503307) (`perception/src/couch_perception/geo.py`)
- [x] Read from MCAP bag files for offline testing (`perception/src/couch_perception/bag_reader.py`)
- [x] Output localization at 100 Hz
- [x] Create matplotlib dashboard (covariance over time + localized path)
- [x] Test with `bags/university_intersect_gps_only.mcap` and `bags/2026-01-29_12-10-44/walk_around_university_all_data.mcap`
- [x] Bag files uploaded to public S3 bucket (`s3://couch-vision-bags/`)
- [x] Consolidated EKF/geo/bag_reader into `perception/` package (PR #12) — deleted standalone `ekf/` directory
- [x] Integrated EKF into nav2_planner for real-time localization during planning
- [x] ARKit VIO odometry (~30Hz) fused into EKF alongside GPS+IMU (PR #17) — frame alignment computed on first GPS fix

#### EKF Configuration
```yaml
# File: ~/couch_ws/src/couch_bringup/config/ekf_params.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true     # constrain to 2D for ground robot

    # ARKit visual-inertial odometry (high quality 6DOF)
    odom0: /iphone/odom
    odom0_config: [true, true, false,      # x, y, z
                   false, false, true,      # roll, pitch, yaw
                   true, true, false,       # vx, vy, vz
                   false, false, true,      # vroll, vpitch, vyaw
                   false, false, false]     # ax, ay, az
    odom0_differential: false

    # iPhone IMU
    imu0: /iphone/imu
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  true, true, true,         # angular velocities
                  true, true, false]        # linear accelerations (not z)

    # Wheel encoders (enable once Phase 0 complete)
    # odom1: /wheel_odom
    # odom1_config: [false, false, false,
    #                false, false, false,
    #                true, false, false,      # vx only
    #                false, false, true,      # vyaw
    #                false, false, false]
    # odom1_differential: false
```

#### navsat_transform_node Configuration
```yaml
# File: ~/couch_ws/src/couch_bringup/config/navsat_params.yaml
navsat_transform_node:
  ros__parameters:
    frequency: 30.0
    delay: 3.0
    magnetic_declination_radians: 0.0  # look up for your location
    yaw_offset: 0.0
    zero_altitude: true
    broadcast_utm_transform: true
    publish_filtered_gps: true
    use_odometry_yaw: true
    wait_for_datum: false  # use first GPS fix as datum
```

#### Deliverable
Given two GPS coordinates, a `nav_msgs/Path` is planned and visible in Foxglove. The path updates as the couch (phone) moves.

---

### Phase 3: Perception — Object Detection 👁️
**Goal:** Detect pedestrians, vehicles, stop signs, and lanes in real-time on Jetson

**Status:** In Progress — offline bag processing, costmap generation, BEV projection, Nav2 integration all working (PRs #10, #12). Perception pipeline refactored into shared modules. TensorRT optimization and configurable pipeline added.

**Prerequisites:** Phase 1 (camera streaming verified)

**Does NOT require hardware** — detection runs on camera frames regardless of whether the couch moves.

#### Approach
- **YOLOv8n with INT8 quantization** via TensorRT on Jetson Orin Nano
- **YOLOP segmentation** with TensorRT FP16 backend (2.1x faster than PyTorch)
- Configurable pipeline via `PipelineConfig` + YAML presets (`configs/default.yaml`, `fast.yaml`, `accurate.yaml`)
- Publishes `vision_msgs/Detection2DArray` on `/perception/detections`
- Detection classes: pedestrian, vehicle, stop sign, traffic light, lane markings
- YOLOP segmentation for drivable area + lane lines (can be skipped via `segmentation_model: "none"` for ~3x speedup)
- Feed detections into Nav2 costmap for obstacle avoidance

#### Alternative Approaches (if YOLOv8 doesn't fit)
- `jetson-inference` + `ros_deep_learning` — simpler setup, built-in ROS2 node, has Vehicle/Person/RoadSign
- DeepStream reference app — more complex but better optimized for multi-stream
- Fine-tuned model on Roboflow — if default COCO classes aren't sufficient

#### Tasks
- [x] Export YOLOv8n to TensorRT INT8 engine for Jetson (`perception/scripts/export_tensorrt.py`)
- [x] Create ROS2 perception node (`perception/src/couch_perception/ros_node.py`)
- [x] Dockerized deployment (`perception/Dockerfile`, `perception/docker-compose.yml`)
- [x] Offline bag processing with YOLOv8 + YOLOP (PR #10)
- [x] Benchmark script (`perception/scripts/benchmark.py`)
- [x] Platform-specific torch: CUDA on Jetson, CPU/MPS on Mac (`pyproject.toml` uv sources)
- [x] Auto-detect device (cuda > mps > cpu) and prefer TensorRT `.engine` over `.pt`
- [x] Test detection on live camera feed (via bag playback on Jetson)
- [x] Verify FPS is acceptable — **37 FPS (CUDA), 57 FPS (TensorRT INT8)** on Jetson Orin Nano
- [x] Add detection overlay to rviz layout (perception overlay panel)
- [x] BEV projection: depth + camera intrinsics → 3D point clouds with IMU rotation (`bev_projection_runner.py`)
- [x] Ego-centric costmap generation: drivable area (cost 0), lane lines (cost 50), obstacles (cost 100), unseen (cost 98) (`costmap.py`)
- [x] Nav2 integration: costmap → OccupancyGrid → Nav2 planner → nav_msgs/Path (`nav2_planner.py`)
- [x] Docker full-stack: Nav2 + foxglove_bridge + perception in one container (`Dockerfile.nav2`, `make full-stack`)
- [x] YOLOP drivable area + lane line segmentation integrated into costmap
- [x] Perception pipeline refactored into shared modules (PR #12):
  - `projection.py` — shared geometry (IMU rotation, depth projection, mask extraction)
  - `perception_pipeline.py` — PerceptionPipeline class (detect → project → rotate)
  - `frame_source.py` — BagSource with playback pacing
- [x] pytest-benchmark infrastructure (`make test`, `make benchmark`) for profiling on Mac and Jetson
- [x] TensorRT auto-export on first CUDA run: YOLOP (PyTorch → ONNX → TRT FP16) and YOLOv8 (.pt → engine via ultralytics). Engines saved to volume-mounted `weights/` dir and persist across container restarts. (PR #19)
- [x] Streaming bag reader: single pass for scalar data (GPS/IMU/odom), lazy second pass for image+depth. No longer loads all frames into memory — fixed Jetson OOM on large bags.
- [x] Nav2 planner costmap fix: COST_UNSEEN changed from 98 to -1 (OccupancyGrid unknown), `allow_unknown: true` and `track_unknown_space: true` in nav2_planner_params.yaml. Config volume-mounted to Docker for hot-reload without rebuild.
- [x] YOLOP TensorRT FP16 backend: exported seg-only ONNX → TRT engine, 2.1x speedup (80ms → 37.5ms on Jetson)
- [x] Configurable perception pipeline (`PipelineConfig` dataclass + YAML presets):
  - `configs/default.yaml` — current behavior (YOLOP + YOLOv8n)
  - `configs/fast.yaml` — skip YOLOP, higher subsampling (~45 FPS)
  - `configs/accurate.yaml` — dense subsampling + CUDA streams
- [x] YOLOP postprocessing optimization: argmax at model resolution then nearest-neighbor upsample (saves computation on all devices)
- [x] `--config` CLI arg wired through all runners + `CONFIG=` Makefile variable
- [ ] CUDA stream concurrency for parallel YOLOP + YOLOv8 inference (implemented but untested on Jetson)
- [ ] Evaluate detection quality for target classes (pedestrians, stop signs, vehicles)
- [ ] If COCO classes insufficient, fine-tune on custom dataset (Roboflow)
- [x] Live data source: `LiveSource` in `frame_source.py` subscribes to ROS2 topics (PR #15)

#### Jetson Benchmark Results (Orin Nano, 640×640)

**Individual models:**
| Backend | Avg Latency | FPS |
|---------|------------|-----|
| YOLOv8n CPU (PyTorch) | ~500ms | ~2 |
| YOLOv8n CUDA (PyTorch) | 26.7ms | 37.4 |
| YOLOv8n TensorRT INT8 | 17.7ms | 56.6 |
| YOLOP CUDA (PyTorch) | 80.1ms | 12.5 |
| YOLOP TensorRT FP16 | 37.5ms | 26.7 |

**Full pipeline + costmap (end-to-end):**
| Config | YOLOP | YOLOv8 | Total | FPS |
|--------|-------|--------|-------|-----|
| default (TRT) | 37.5ms | 20.9ms | ~64ms | ~15.6 |
| fast (no seg) | skip | 20.9ms | ~22ms | ~45 |

Non-inference code (projection, rotation, costmap build) is sub-millisecond — not a bottleneck.

**Full-stack benchmarks (Jetson Orin Nano, 6.9GB bag, TRT engines):**
| Metric | Value |
|--------|-------|
| Perception FPS | 8.6–17.3, median ~12 |
| GPU utilization | 99% (GR3D_FREQ) |
| CPU utilization | 38% user, 52% idle |
| RAM usage | 2.1 / 7.6 GB (28%) |
| Power draw | 5.9W total, 2.0W CPU+GPU |
| Thermal | 47°C (well below 97°C throttle) |

#### Deliverable
Live bounding boxes on camera feed in Foxglove. `/detections` topic publishing `Detection2DArray` at 10+ FPS.

---

### Phase 4: SLAM 🗺️
**Goal:** Build and localize in a map

**Status:** In Progress — RTAB-Map integrated into Nav2 Docker stack (PR #20), WM=6+ keyframes working

**Prerequisites:** Phase 1 (camera + depth streaming), Phase 2 (EKF odometry)

#### Approach Decision
- [x] **RTAB-Map** — mature, works with RGB-D, supports compressed images via image_transport, well-documented

#### Tasks — RTAB-Map
- [x] Install: `ros-jazzy-rtabmap-ros` added to Dockerfile.nav2
- [x] Set up image_transport compressed subscriber (republish node decompresses to raw)
- [x] Create launch file with correct topic remapping (`nav2_planner.launch.py`)
- [x] Fix RGB/depth resolution mismatch: resize both to 512×384 for SLAM (was 1920×1440 RGB, 256×192 depth)
- [x] Scale camera intrinsics to match resized images
- [x] Tune RTAB-Map params for keyframe creation (ORB features, rehearsal threshold, intermediate nodes)
- [x] Test SLAM with bag replay — WM=6+ keyframes, `/map` OccupancyGrid published
- [ ] Test loop closure detection
- [ ] Verify map quality in Foxglove (map panel + trajectory)
- [ ] Test with live iPhone data

#### Tasks — Common
- [ ] Drive couch around entire operating area
- [ ] Verify map covers all needed spaces
- [ ] Save map: `ros2 run nav2_map_server map_saver_cli -f ~/maps/house`
- [ ] Test localization with saved map (restart and relocalize)

#### RTAB-Map Integration (Working)

RTAB-Map is integrated into the Nav2 Docker stack via `perception/launch/nav2_planner.launch.py`. Key configuration:

**Launch file** (`nav2_planner.launch.py`):
- Republish node: decompresses `/camera/image/compressed` → `/camera/image` (raw)
- RTAB-Map node: subscribes to `/camera/image`, `/camera/depth/image`, `/camera/camera_info`
- Static TF: `odom→base_link`, `base_link→camera`
- SLAM always enabled (was optional via `ENABLE_SLAM` env var, now always on)

**Critical fix: RGB/depth resolution mismatch** (`nav2_planner.py`):
- Original: RGB 1920×1440, depth 256×192 — RTAB-Map ignored depth for feature detection
- Fix: Both resized to 512×384 in `nav2_planner.py` before publishing
- Camera intrinsics scaled proportionally via `_make_camera_info_scaled()`

**RTAB-Map params** (`perception/config/rtabmap_params.yaml`):
```yaml
Kp/MaxFeatures: "500"           # ORB features per frame
Kp/DetectorStrategy: "6"        # ORB detector
Vis/FeatureType: "6"            # Must match detector
Mem/RehearsalSimilarity: "0.2"  # Lower = more distinct keyframes
Vis/MinInliers: "15"            # Reject bad keyframes
Rtabmap/CreateIntermediateNodes: "true"  # Denser graph
```

**Run:**
```bash
make full-stack BAG=bags/walk_around_university_all_data.mcap
# Connect Foxglove to ws://localhost:8765
# Look for WM=N in logs (N > 1 means SLAM is working)
```

**Verify SLAM is working:**
```bash
docker compose -f perception/docker-compose.nav2.yml logs | grep "WM="
# Should show: (local map=2, WM=6) or higher
```

#### Deliverable
2D occupancy grid map of operating space, working localization

---

### Phase 5: Nav2 Full Integration 🚗
**Goal:** Autonomous point-to-point navigation with obstacle avoidance

**Status:** Not Started

**Prerequisites:** Phase 0 (motors), Phase 2 (GPS/EKF), Phase 3 (perception), Phase 4 (SLAM)

This is where everything comes together. The couch can navigate autonomously.

#### Tasks
- [ ] Create couch URDF/Xacro with correct dimensions
- [ ] Install pointcloud_to_laserscan: `sudo apt install ros-jazzy-pointcloud-to-laserscan`
- [ ] Configure pointcloud → LaserScan for Nav2 costmap
- [ ] Create Nav2 params file (see config below)
- [ ] Launch Nav2 with map
- [ ] Test sending goal via Foxglove or CLI
- [ ] Integrate `/detections` into costmap (pedestrian/vehicle avoidance)
- [ ] Tune costmap inflation radius for couch size
- [ ] Tune velocity limits for comfortable/safe operation
- [ ] Tune acceleration limits
- [ ] Test recovery behaviors (what happens when stuck)
- [ ] Test with dynamic obstacles (walk in front of couch)
- [ ] Test outdoor GPS waypoint following end-to-end

#### Pointcloud to LaserScan Config
```yaml
min_height: 0.1       # ignore ground
max_height: 1.5       # ignore ceiling
angle_min: -1.57      # -90 degrees
angle_max: 1.57       # +90 degrees
range_min: 0.1
range_max: 5.0        # iPhone LiDAR max range
```

#### Nav2 Parameters
```yaml
# File: ~/couch_ws/src/couch_bringup/config/nav2_params.yaml

bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered
    bt_loop_duration: 10
    default_server_timeout: 20
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: true
      min_vel_x: -0.3              # reverse speed
      max_vel_x: 1.0               # forward speed (m/s) — TUNE for couch
      min_vel_y: 0.0               # differential drive, no sideways
      max_vel_y: 0.0
      max_vel_theta: 1.0           # rotation speed (rad/s)
      min_speed_xy: 0.0
      max_speed_xy: 1.0
      min_speed_theta: 0.0
      acc_lim_x: 0.5
      acc_lim_y: 0.0
      acc_lim_theta: 1.0
      decel_lim_x: -1.0
      decel_lim_y: 0.0
      decel_lim_theta: -1.0
      vx_samples: 20
      vy_samples: 1                # 1 for diff drive
      vtheta_samples: 20
      sim_time: 1.5
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.5
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: true
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.5            # ADJUST: half of couch width
      plugins: ["voxel_layer", "inflation_layer"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.5
          obstacle_min_range: 0.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.7       # ADJUST: robot_radius + safety margin

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.5             # ADJUST: half of couch width
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.5
          obstacle_min_range: 0.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.7

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      tolerance: 0.25
      downsample_costmap: false
      downsampling_factor: 1
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 5.0
      motion_model_for_search: "DUBIN"
      angle_quantization_bins: 72
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 3.0
      minimum_turning_radius: 0.4   # ADJUST: based on couch turning ability
      reverse_penalty: 2.0
      change_penalty: 0.0
      non_straight_penalty: 1.2
      cost_penalty: 2.0
      retrospective_penalty: 0.015
      lookup_table_size: 20.0
      cache_obstacle_heuristic: false
      smooth_path: true

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
    wait:
      plugin: "nav2_behaviors/Wait"
```

#### Couch Dimensions (Fill in)
```yaml
couch_length: _____ m
couch_width: _____ m
couch_height: _____ m
wheel_separation: _____ m
wheel_radius: _____ m
```

#### Deliverable
Couch navigates autonomously to goals (indoor via map, outdoor via GPS waypoints), avoiding obstacles including detected pedestrians/vehicles.

---

### Phase 6: Multi-iPhone Support (Future)
**Goal:** 360° perception coverage

**Status:** Not Started

CouchVision already supports configurable topic prefixes. Multi-phone just means running separate app instances with `/iphone1`, `/iphone2` prefixes.

#### Tasks
- [ ] Mount additional iPhone(s) (rear, sides as needed)
- [ ] Measure each iPhone's transform from base_link
- [ ] Configure static TFs on Jetson for each phone
- [ ] Set distinct topic prefix per phone in CouchVision settings
- [ ] Create pointcloud merger node (transform each phone's cloud to base_link, merge)
- [ ] Pick one primary phone for VIO odom (or fuse multiple — complex)
- [ ] Update Nav2 costmap to use merged `/scan`
- [ ] Test obstacle detection from all angles

#### Deliverable
Full surround obstacle detection with no blind spots

---

### Phase 7: Safety & Polish 🛡️
**Goal:** Reliable, safe operation

**Status:** In Progress — Foxglove Hardware Safety Panel implemented (PR #25), e-stop wired to nav2_planner

#### Tasks
- [x] Add software e-stop (Foxglove Hardware Safety Panel → `/e_stop` topic → nav2_planner pauses)
- [x] Teleop joystick/WASD in Foxglove → `/cmd_vel` (geometry_msgs/Twist)
- [x] Velocity monitor bars, battery/thermal status display
- [ ] Add physical emergency stop button (hardware button → GPIO → `/e_stop` topic)
- [ ] Implement e-stop handler in motor controller (subscribe to `/e_stop`, hard-stop motors)
- [ ] Add velocity smoother for comfortable acceleration
- [ ] Implement collision detection (if obstacle <0.3m, hard stop)
- [ ] Create unified launch file
- [ ] Test full system restart and recovery
- [ ] Test with dynamic obstacles (people walking)
- [ ] Outdoor testing with GPS waypoint following
- [ ] Optional: voice commands, web interface

#### Deliverable
Safe, reliable self-driving couch

---

## Dependencies & Installation

### ROS2 Packages (Jazzy)
```bash
# Core navigation
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-nav2-bringup

# SLAM
sudo apt install ros-jazzy-rtabmap-ros

# Sensor processing
sudo apt install ros-jazzy-robot-localization
sudo apt install ros-jazzy-pointcloud-to-laserscan
sudo apt install ros-jazzy-tf2-sensor-msgs
sudo apt install ros-jazzy-image-transport
sudo apt install ros-jazzy-image-transport-plugins    # for compressed ↔ raw

# Visualization
sudo apt install ros-jazzy-rviz2
sudo apt install ros-jazzy-rqt*
sudo apt install ros-jazzy-foxglove-bridge  # WebSocket bridge for Foxglove app

# Control
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-ros2-controllers

# Perception
# DeepStream: follow NVIDIA docs for Jetson
# YOLOv8: pip install ultralytics

# Utilities
sudo apt install ros-jazzy-teleop-twist-keyboard
sudo apt install ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-xacro
```

### Python Dependencies
```bash
pip3 install numpy scipy transforms3d
```

---

## Project Directory Structure

```
couch-vision/
├── perception/                      # Python perception package (uv project)
│   ├── pyproject.toml
│   ├── Dockerfile                   # Perception-only container
│   ├── Dockerfile.nav2              # Full-stack: Nav2 + foxglove_bridge + perception
│   ├── docker-compose.yml
│   ├── configs/                     # Pipeline presets
│   │   ├── default.yaml             # YOLOP + YOLOv8n (current behavior)
│   │   ├── fast.yaml                # Skip YOLOP, high subsampling (~45 FPS)
│   │   └── accurate.yaml            # Dense subsampling + CUDA streams
│   ├── src/couch_perception/
│   │   ├── __init__.py
│   │   ├── config.py                # PipelineConfig dataclass + YAML loading
│   │   ├── perception_pipeline.py   # PerceptionPipeline: detect → project → rotate
│   │   ├── projection.py            # Shared geometry (IMU rotation, depth projection)
│   │   ├── frame_source.py          # BagSource + LiveSource + SensorStreams
│   │   ├── costmap.py               # Costmap building logic (build_costmap + grid constants)
│   │   ├── costmap_visualizer.py    # Costmap → color image rendering
│   │   ├── nav2_planner.py          # Full Nav2 pipeline: perception + EKF + routing + planning
│   │   ├── ros_node.py              # ROS2 perception node
│   │   ├── yolov8_detector.py       # YOLOv8 detection (auto: TensorRT > CUDA > MPS > CPU)
│   │   ├── yolop_detector.py        # YOLOP segmentation (auto: TensorRT FP16 > CUDA > CPU)
│   │   ├── camera_model.py          # Camera intrinsics + projection
│   │   ├── bag_reader.py            # MCAP bag reader (SyncedFrame output)
│   │   ├── ekf.py                   # EKF: IMU + GPS + ARKit odom fusion
│   │   ├── geo.py                   # GPS → ENU conversion
│   │   └── visualization.py         # 2D detection overlay
│   ├── tests/
│   │   ├── conftest.py              # Shared fixtures (device, dummy frames)
│   │   └── test_benchmark.py        # pytest-benchmark tests for all components
│   └── scripts/
│       ├── benchmark.py
│       ├── export_tensorrt.py
│       └── diagnose_projection.py
├── bridge/
│   └── ios_bridge.py                # iPhone TCP → ROS2 bridge
├── CouchVision/                     # iOS app (Swift/Xcode)
├── bags/                            # MCAP bag files (gitignored)
├── foxglove/
│   ├── couch_layout.json
│   ├── nav-control-panel/          # Foxglove extension (pnpm, TypeScript/React)
│   │   ├── src/NavControlPanel.tsx  # Interactive nav panel (Leaflet map, GO button)
│   │   └── src/index.ts
│   └── hardware-safety-panel/      # Foxglove extension — e-stop, teleop, system status
│       ├── src/HardwareSafetyPanel.tsx
│       └── src/index.ts
├── rviz/
│   └── couchvision.rviz
├── Makefile                         # make full-stack, make costmap, make bev-projection, etc.
└── SELF_DRIVING_STACK.md            # This file
```

---

## ROS2 Topic Reference

Complete topic map as of 2026-02-16. **Future agents: wire new hardware/nodes to these exact topic names.** The Foxglove extensions and nav2_planner depend on them.

### Sensor Input Topics (iPhone → Bridge)

Published by `bridge/ios_bridge.py`. Topic prefix is configurable (default `/iphone_charlie`).

| Topic | Message Type | Rate | Notes |
|-------|-------------|------|-------|
| `{prefix}/camera/arkit/image/compressed` | `sensor_msgs/CompressedImage` | ~11 Hz | Back camera JPEG |
| `{prefix}/camera/arkit/camera_info` | `sensor_msgs/CameraInfo` | ~30 Hz | Camera intrinsics |
| `{prefix}/lidar/depth/image` | `sensor_msgs/Image` | ~30 Hz | 32FC1 depth, 256×192 |
| `{prefix}/lidar/points` | `sensor_msgs/PointCloud2` | ~30 Hz | XYZI float32 |
| `{prefix}/imu` | `sensor_msgs/Imu` | ~100 Hz | Orientation + angular vel + linear accel |
| `{prefix}/odom` | `nav_msgs/Odometry` | ~50 Hz | ARKit VIO pose |
| `{prefix}/gps/fix` | `sensor_msgs/NavSatFix` | ~1 Hz | Lat/lon/alt |
| `{prefix}/gps/velocity` | `geometry_msgs/TwistStamped` | ~1 Hz | GPS speed/course |
| `{prefix}/heading` | `std_msgs/Float64` | ~17 Hz | Magnetic heading (degrees) |
| `{prefix}/battery` | `sensor_msgs/BatteryState` | ~1 Hz | Battery % and temperature |
| `{prefix}/thermal` | `std_msgs/Int32` | On change | 0=nominal, 1=fair, 2=serious, 3=critical |
| `{prefix}/magnetic_field` | `sensor_msgs/MagneticField` | ~100 Hz | Magnetometer |
| `{prefix}/pressure` | `sensor_msgs/FluidPressure` | ~1 Hz | Barometer |
| `{prefix}/altitude` | `std_msgs/Float64` | ~1 Hz | Barometric altitude |
| `/tf` | `tf2_msgs/TFMessage` | ~170 Hz | ARKit 6DOF pose transforms |

### Republished Sensor Topics (Nav2 Planner → SLAM/Foxglove)

Nav2 planner republishes sensor data at reduced resolution for SLAM and visualization. These are the topics that RTAB-Map and Foxglove panels consume.

| Topic | Message Type | Publisher | Subscribers | Notes |
|-------|-------------|-----------|-------------|-------|
| `/camera/image/compressed` | `sensor_msgs/CompressedImage` | nav2_planner | RTAB-Map (via republish→raw) | 512×384 JPEG |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | nav2_planner | RTAB-Map | Scaled intrinsics |
| `/camera/depth/image` | `sensor_msgs/Image` | nav2_planner | RTAB-Map, Foxglove | 512×384 32FC1 |
| `/imu` | `sensor_msgs/Imu` | nav2_planner | RTAB-Map | Republished from bag/live |
| `/odom` | `nav_msgs/Odometry` | nav2_planner | RTAB-Map | Republished from bag/live |
| `/gps/fix` | `sensor_msgs/NavSatFix` | nav2_planner | Foxglove Nav Control Panel | Republished GPS |

### Perception Output Topics

Published by `nav2_planner.py` after YOLOv8 + YOLOP inference.

| Topic | Message Type | Subscribers | Notes |
|-------|-------------|-------------|-------|
| `/perception/image_annotated/compressed` | `sensor_msgs/CompressedImage` | Foxglove (Image panel) | Camera with detection boxes + path overlay |
| `/perception/pointcloud/drivable` | `sensor_msgs/PointCloud2` | Foxglove (3D panel) | Drivable area points (green) |
| `/perception/pointcloud/lanes` | `sensor_msgs/PointCloud2` | Foxglove (3D panel) | Lane line points (red) |
| `/perception/pointcloud/detections` | `sensor_msgs/PointCloud2` | Foxglove (3D panel) | Detection bounding box points (yellow) |

### Navigation Topics

Published by `nav2_planner.py` for path planning and visualization.

| Topic | Message Type | Subscribers | Notes |
|-------|-------------|-------------|-------|
| `/costmap/occupancy_grid` | `nav_msgs/OccupancyGrid` | Nav2 planner_server, Foxglove 3D | Ego-centric costmap (20m, 0.2m res) |
| `/nav/planned_path` | `nav_msgs/Path` | Foxglove 3D panel | Nav2-planned local path |
| `/nav/google_maps_path` | `nav_msgs/Path` | Foxglove 3D panel | Full Google Maps walking route |
| `/nav/costmap_image/compressed` | `sensor_msgs/CompressedImage` | Foxglove Image panel | Color-coded costmap visualization |
| `/nav/goal_marker` | `visualization_msgs/Marker` | Foxglove 3D panel | Red cube at lookahead goal |
| `/nav/ego_marker` | `visualization_msgs/Marker` | Foxglove 3D panel | White sphere at current position |
| `/nav/status` | `std_msgs/String` | **Hardware Safety Panel**, **Nav Control Panel** | 1Hz JSON (see below) |

**`/nav/status` JSON payload:**
```json
{
  "api_key_configured": true,
  "route_resolved": true,
  "current_dest_lat": 38.03683,
  "current_dest_lon": -78.503577,
  "route_points": 465,
  "ekf_initialized": true,
  "heading_calibrated": true,
  "e_stopped": false
}
```

### Control Topics (Foxglove Panels → Backend)

**These are the topics that future hardware nodes MUST subscribe to.**

| Topic | Message Type | Publisher | Subscriber | Notes |
|-------|-------------|-----------|------------|-------|
| `/cmd_vel` | `geometry_msgs/Twist` | **Hardware Safety Panel** (teleop joystick/WASD) | **Motor controller (Phase 0)** | `linear.x` (m/s) + `angular.z` (rad/s). Panel also sends zeros at 10Hz when e-stopped. |
| `/e_stop` | `std_msgs/Bool` | **Hardware Safety Panel** | nav2_planner | `true`=stop, `false`=armed. Planner pauses perception/planning but keeps localization. **Motor controller must also subscribe.** |
| `/nav/set_destination` | `std_msgs/String` | **Nav Control Panel** | nav2_planner | JSON: `{dest_lat, dest_lon, lookahead_m, start_lat?, start_lon?, api_key?}` |
| `/clicked_point` | `geometry_msgs/PointStamped` | Foxglove 3D panel (click tool) | nav2_planner | Click-to-navigate in 3D view |

### SLAM Topics (RTAB-Map)

| Topic | Message Type | Publisher | Notes |
|-------|-------------|-----------|-------|
| `/map` | `nav_msgs/OccupancyGrid` | RTAB-Map | SLAM-generated occupancy grid |
| `/mapPath` | `nav_msgs/Path` | RTAB-Map | SLAM trajectory |
| `/cloud_map` | `sensor_msgs/PointCloud2` | RTAB-Map | SLAM point cloud |

### Hardware Integration Checklist (Phase 0)

When motors + ESC hardware arrive, the motor controller node must:

1. **Subscribe to `/cmd_vel`** (`geometry_msgs/Twist`) — convert `linear.x` + `angular.z` to differential wheel speeds
2. **Subscribe to `/e_stop`** (`std_msgs/Bool`) — hard-stop motors when `data=true`
3. **Publish `/wheel_odom`** (`nav_msgs/Odometry`) — encoder-derived odometry for EKF fusion
4. **Respect the Hardware Safety Panel**: panel defaults to e-stopped on startup, user must press ARM MOTORS before any motion

The Hardware Safety Panel (`foxglove/hardware-safety-panel/`) already publishes these topics. The Nav Control Panel (`foxglove/nav-control-panel/`) handles destination setting. No changes to the Foxglove extensions are needed for hardware bring-up.

---

## Debugging Guide

### iPhone not connecting / topics not appearing
```bash
# Check topics
ros2 topic list | grep iphone

# Check network (USB-C creates ethernet interface)
ifconfig | grep -A5 "en"
ping <iphone_ip>

# Check Zenoh router
zenohd
```

### TF transform errors
```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link iphone_base_link

# The app publishes to /tf, not /tf_static
ros2 topic echo /tf
```

### Nav2 not planning
```bash
ros2 lifecycle list /controller_server
ros2 topic hz /local_costmap/costmap
ros2 topic echo /goal_pose
ros2 topic echo /cmd_vel
```

### SLAM map quality
```bash
# Reset RTAB-Map
ros2 service call /rtabmap/reset std_srvs/srv/Empty

# Check depth
ros2 topic hz /iphone/lidar/depth/image

# Check bandwidth
ros2 topic bw /iphone/camera/back_wide/image/compressed
```

### Useful debug commands
```bash
htop
nvidia-smi                    # GPU usage on Jetson (jtop is better)
sudo jtop                     # Jetson-specific monitoring
ros2 run rqt_graph rqt_graph
ros2 topic bw /iphone/camera/back_wide/image/compressed
ros2 topic delay /iphone/imu
ros2 param dump /controller_server
```

---

## Agent Notes & Learnings

This section is for Claude Code instances and other agents to record learnings, gotchas, and useful information discovered while working on this project.

### iOS App (CouchVision) Internals

- **Architecture:** Sensor managers → Combine PassthroughSubjects → SensorCoordinator subscribes → CDR encodes → ZenohPublisher sends over TCP
- **CDR encoding:** Custom `CDREncoder` class implements XCDR1 little-endian format with encapsulation header `[0x00, 0x01, 0x00, 0x00]`
- **Timestamps:** iOS uses time-since-boot; `TimeUtils.toUnixTimestamp()` converts to Unix epoch by adding boot time offset
- **LiDAR camera frames:** When LiDAR (ARKit) is active, camera frames come from ARKit (higher quality intrinsics). The camera toggle gates whether these are published — see `SensorCoordinator.swift` lines ~298-312.
- **Coordinate conversion:** ARKit (right, up, back) → ROS (forward, left, up). Done in `LiDARManager.createPointCloud()`: `(depth, -ptX, -ptY, intensity)`
- **Background:** ARKit/LiDAR stops in background. Motion/GPS/Environment continue. Camera stops.
- **Config hot-reload:** Changing sensor config while running triggers reconfiguration without restart.
- **Frame IDs:** `iphone_base_link`, `iphone_lidar`, `iphone_camera_arkit`, `iphone_camera_back_wide`, `world`
- **ZenohPublisher:** Currently a placeholder TCP implementation, not actual Zenoh protocol. Sends `[topic_len:4][topic][data_len:4][data]` frames. Needs a bridge on the receiver side.
- **Backpressure:** `SensorCoordinator.publishGated(key:)` uses a `Set<String>` to gate sends per topic. If a previous send is still in-flight, the frame is dropped. Zero impact on fast connections.

### Jetson Orin Nano
- Access: `ssh jetson-nano` (via Tailscale)
- 8GB shared GPU/CPU memory — monitor with `jtop`
- ROS2 Jazzy built from source at `~/ros2_jazzy/`
- Python 3.10 (not 3.12 like Mac)
- Tailscale iptables: port 8765 needs explicit allow rule for Foxglove access from other tailnet peers (`sudo iptables -I ts-input 3 -s 100.64.0.0/10 -p tcp --dport 8765 -j ACCEPT`). This rule is NOT persistent across reboots — needs to be made permanent.

### Open Questions
- [x] ~~What motor controllers to use?~~ → Flipsky Dual FSESC 6.7 + Flipsky 7070 110KV motor
- [ ] Actual couch dimensions?
- [ ] Power system? (currently bench PSU; need battery selection for production)
- [ ] Do we need the Zenoh bridge to be a proper rmw_zenoh implementation, or is the TCP bridge sufficient?
- [ ] Wheel diameter/radius? (needed for odometry and differential drive calculations)
- [ ] Wheel separation? (distance between left and right wheel centers)
- [ ] Second motor — same 7070 110KV for the other side?

### Discovered Issues

- **Adding a new topic requires updating BOTH the iOS app AND the Python bridge.** The bridge (`bridge/ios_bridge.py`) manually parses CDR bytes for each message type. If you add a new topic/message on the iOS side but don't add a corresponding entry in `_topic_types` and a `_parse_*` method in the bridge, messages will be silently dropped (with a warning log). This is the most common mistake when adding new publishers. Always update: (1) iOS message struct, (2) CDREncoder, (3) sensor manager, (4) SensorCoordinator, (5) bridge `_topic_types` + `_parsers` + parser method.

### Perception Pipeline Architecture (PR #12)

The perception package was refactored to eliminate code duplication across runners:

- **`PerceptionPipeline`** (`perception_pipeline.py`): Shared detect→project→rotate loop. Call `process_frame(SyncedFrame)` → get `PerceptionResult` with drivable_pts, lane_pts, det_pts, det_groups.
- **`BagSource` / `LiveSource`** (`frame_source.py`): Both produce `SensorStreams` (frames iterator + GPS fixes + IMU samples). `BagSource` reads MCAP bags with playback pacing. `LiveSource` subscribes to live ROS2 topics with `message_filters.ApproximateTimeSynchronizer` for image+depth sync.
- **`projection.py`**: Shared geometry utilities (IMU rotation, depth camera model, mask/bbox pixel extraction).
- **`costmap.py`**: Pure costmap building logic (`build_costmap` + grid constants). Extracted from the former `costmap_runner.py`.
- **Single entry point**: `nav2_planner.py` handles both bag replay and live mode. Standalone runners (`runner.py`, `bev_projection_runner.py`, `costmap_runner.py`) were removed — their functionality is fully covered by `nav2_planner.py`.

### Docker Full-Stack (Dockerfile.nav2)

The `make full-stack` command runs Nav2 + foxglove_bridge + perception in Docker. In live mode, a second container runs the iOS bridge. Key details:
- Stack runs in background (`-d`). Use `make logs-bridge` / `make logs-nav2` / `make logs` to tail individual or all services. `make stop` to bring down.
- Uses `foxglove_bridge` (C++ ROS2 node) to auto-forward ALL ROS2 topics to Foxglove on port 8765
- Nav2 planner_server runs inside the container, receives costmaps via ROS2 topics
- **CycloneDDS**: Both containers mount `cyclonedds.xml` (localhost-unicast, no multicast). This is required for inter-container DDS communication when using `network_mode: host`. Without matching configs, topic discovery works but data doesn't flow.
- On macOS: requires Colima (`colima start --cpu 4 --memory 8`), NOT Docker Desktop
- Foxglove connect to `ws://localhost:8765` to visualize camera, pointclouds, costmap, and planned path
- **Multi-arch**: `dustynv/pytorch:r36.4.0` base on Jetson (arm64), `ros:jazzy` on amd64/Mac. CI builds amd64 only; Jetson builds locally.
- **Volume mounts**: `./weights:/perception/weights` persists TRT engines and downloaded `.pt` files across container restarts
- **Jetson Docker**: requires `nvidia` container runtime. If `permission denied` on Docker socket, run `sudo usermod -aG docker $USER` and re-login.
- **Jetson USB-C connection**: Jetson USB-C port is in device mode (tegra-xudc). iPhone connects as USB host, gets `192.168.55.100` via DHCP on `l4tbr0` bridge. Bridge listens on `192.168.55.1:7447`.

### Streaming Bag Reader

`bag_reader.py` uses a two-pass approach to avoid loading all images into memory:
1. **First pass**: reads lightweight scalar topics (GPS, IMU, odom, camera_info) into lists
2. **Second pass**: lazy iterator streams image+depth frames one at a time, syncing with nearest depth/IMU on-the-fly

This is critical for Jetson (8GB shared RAM). The old approach loaded all decoded frames into memory, causing OOM on bags >1GB. The streaming reader processes a 6.9GB bag using only ~2.1GB RAM.

### Foxglove Navigation Control Extension (PR #18)

Custom Foxglove extension at `foxglove/nav-control-panel/` for interactive destination control:
- **Leaflet map**: click=set destination, shift+click/right-click=set start, draggable markers
- **GO button**: publishes to `/nav/set_destination` (std_msgs/String with JSON payload). Flashes green on send.
- **Status bar**: shows backend destination vs panel destination (in sync / differs), EKF state, route points, API key status
- **API key**: stored in localStorage. Backend key (from env var) takes precedence — warning only shows when neither panel nor backend has a key.
- **Start location**: defaults to GPS, overridable via map or text input
- Nav2 planner subscribes to `/nav/set_destination` and re-routes dynamically without restarting the pipeline
- Nav2 planner publishes `/nav/status` (1Hz) with route state, enabling the sync indicator
- Build: `cd foxglove/nav-control-panel && pnpm install && pnpm build`
- Install: `pnpm local-install` (copies to `~/.foxglove-studio/extensions/`)
- Panel appears as "Navigation Control [local]" in Foxglove panel dropdown
- **CSP constraint**: Leaflet CSS injected inline (Electron blocks CDN scripts)
- **Grid tuning (PR #18)**: 40m/0.3res → 20m/0.2res, lane cost 80→50, unseen areas collapsed into single cost 98 (was -1 unknown + 99 FOV boundary), Nav2 `allow_unknown=false`, inflation layer 1.0m radius, lookahead 15m→8m

### Foxglove Hardware Safety Panel Extension (PR #25)

Custom Foxglove extension at `foxglove/hardware-safety-panel/` for hardware bring-up and safety:
- **E-stop button**: publishes `std_msgs/Bool` on `/e_stop`. Defaults to stopped on startup (safety-first). Publishes initial state on connect so planner is in sync.
- **ARM MOTORS toggle**: must be pressed before teleop works. Shows STOPPED (red) / ARMED (green) indicator.
- **Teleop controls**: WASD keyboard + virtual joystick → publishes `geometry_msgs/Twist` on `/cmd_vel`. Configurable max linear/angular velocity.
- **Deadman switch**: while e-stopped, continuously publishes zero `/cmd_vel` at 10Hz.
- **Velocity monitor**: live horizontal bars showing current `linear.x` and `angular.z` from `/cmd_vel` feedback.
- **System status**: battery %, thermal state from iPhone (subscribes to `/iphone/battery`, `/iphone/thermal`).
- **Nav status**: route resolved, EKF init, planner paused/active (subscribes to `/nav/status`).
- **E-stop → nav2_planner**: planner subscribes to `/e_stop`, pauses perception/planning when engaged but keeps localization running.
- Build: `cd foxglove/hardware-safety-panel && pnpm install && pnpm build`
- Install: `pnpm local-install` (copies to `~/.foxglove-studio/extensions/`)
- Panel type ID in layout: `"Hardware Safety Panel.Hardware Safety!hwsafety"`
- **Battery/thermal topic note**: panel hardcodes `/iphone/battery` and `/iphone/thermal` but live bridge uses configurable prefix (e.g. `/iphone_charlie/battery`). Panel degrades gracefully (shows "--").

### VESC Motor Controller Setup & Communication

**Hardware:** Flipsky Dual FSESC 6.7 + Flipsky 7070 110KV motor, connected via USB to Jetson Orin Nano.

**Jetson cross-compilation:** The `~/bldc/` repo contains the VESC firmware source. The bundled x86 ARM toolchain doesn't work on the aarch64 Jetson. Fix: `sudo apt install gcc-arm-none-eabi` and rename the bundled toolchain dir (`tools/gcc-arm-none-eabi-7-2018-q2-update` → `.x86bak`) so the Makefile falls through to the system compiler. Build with `make flipsky_60_mk5` (NOT `make 60_mk5` — that's the Trampa variant). Output: `build/flipsky_60_mk5/flipsky_60_mk5.bin`.

**VESC Tool on Jetson:** Installed via Flatpak (`flatpak install flathub com.vesc_project.VescTool`). The official Linux zip is x86-only. Must run as root for reliable serial access: `sudo flatpak run --device=all --filesystem=/dev com.vesc_project.VescTool`. User must be in `dialout` group (`sudo usermod -aG dialout charlie`).

**Motor detection in VESC Tool:** Use FOC mode, detection current 10A (not 5A — the 7070 is a large motor). Run RL detection first, then hall sensor detection. Set sensor mode to Hall Sensors. Turn OFF "Enable Phase Filters" (FSESC 6.7 doesn't have them). Turn ON "Slow ABS Max Current".

**Serial protocol reference:** See `~/bldc/spin_test.py` for a complete working implementation of VESC UART communication in Python (no external VESC libraries — `pyvesc` is broken on Python 3.12). Uses `pyserial` only. Protocol, command IDs, and telemetry parsing are all documented in Phase 0 above.

**Key gotchas:**
- VESC command timeout is 1000ms. Send commands every 50ms to avoid stuttering.
- `COMM_SET_RPM` uses ERPM (electrical RPM) = mechanical RPM × pole_pairs (7 for the 7070).
- Telemetry reads take ~20ms. Always re-send the motor command immediately after reading to avoid a gap.
- Current limits MUST be set in VESC Tool (Motor Settings > General > Current Limits), not in the script. The RPM PID controller decides how much current to draw internally. If your bench PSU has a 5A limit, set Motor Current Max = 5A and Battery Current Max = 5A in VESC Tool, otherwise the ESC will overcurrent the PSU and the USB device will disconnect (`Input/output error`).
- The dual ESC has two independent controllers. USB connects to the master (CAN ID 0). The slave (CAN ID 1) is reached via CAN forwarding in VESC Tool. Each side needs independent motor detection.
- `pyvesc` Python library is broken on Python 3.12 (PyCRC import fails). Use the raw protocol implementation in `spin_test.py` instead.

**Verified telemetry fields:** RPM (from hall sensor PID), duty cycle, input voltage, motor current, input current, Iq (torque-producing), Id (field-weakening), FET temperature, motor temperature, watt-hours, amp-hours, tachometer (cumulative hall counts), fault codes.

**Tachometer → wheel odometry:** `tach / (7 pole_pairs × 6 transitions) = mechanical revolutions`. Multiply by `2π × wheel_radius` for distance. The tachometer is cumulative and monotonically increasing (tachometer_abs for absolute). This gives us wheel odometry without separate encoders.

**Next steps for motor integration:**
1. Wire second motor to slave side of dual ESC, run motor detection
2. Build a ROS2 node that subscribes to `/cmd_vel` (Twist) and converts to VESC commands:
   - `v_left = linear.x - (angular.z × wheel_separation / 2)`
   - `v_right = linear.x + (angular.z × wheel_separation / 2)`
   - Convert wheel velocity to ERPM: `erpm = (v / (2π × wheel_radius)) × 60 × pole_pairs`
   - Send `COMM_SET_RPM` to master (left) and slave via CAN forwarding (right)
3. Publish `/wheel_odom` (nav_msgs/Odometry) from tachometer feedback
4. Respect `/e_stop` topic (send zero current on e-stop)
5. Wire into the existing Hardware Safety Panel (Foxglove) for teleop control

### Helpful Commands
<!-- Agents: add useful commands here -->
- `make full-stack` — live mode: starts iOS bridge + Nav2 + perception in Docker (background). Connect iPhone via USB-C, then Foxglove to `ws://localhost:8765`.
- `make full-stack BAG=<path>` — bag replay mode.
- `make full-stack DEST_LAT=38.036 DEST_LON=-78.503` — set navigation destination (defaults to UVA area).
- `make logs-bridge` — tail iOS bridge container logs only.
- `make logs-nav2` — tail Nav2 planner container logs only.
- `make logs` — tail all container logs (interleaved).
- `make stop` — `docker compose down` the full stack.
- `make test` — run perception pytest suite.
- `make benchmark` — run pytest-benchmark profiling.
- Foxglove layout: import `foxglove/couch_layout.json` for sensor + nav2 visualization.
- `make build-extension` — build Foxglove nav control extension (pnpm).
- `make install-extension` — install extension to Foxglove desktop app.

### foxglove_bridge Build (Jetson)

foxglove_bridge is built from source on the Jetson at `~/ros2_jazzy/`. It requires:

1. **rosx_introspection** — cloned to `~/ros2_jazzy/src/rosx_introspection`. Requires a patch for Jazzy: `rosbag2_cpp/typesupport_helpers.hpp` moved to `rclcpp/typesupport_helpers.hpp`, and namespace changed from `rosbag2_cpp::` to `rclcpp::` (functions: `get_typesupport_library`, `get_message_typesupport_handle`). The `get_message_typesupport_handle` takes `SharedLibrary&` not `shared_ptr`, so dereference with `*`.
2. **foxglove-sdk** — cloned to `~/ros2_jazzy/src/foxglove-sdk`. The bridge is at `ros/src/foxglove_bridge/`.
3. **System deps:** `rapidjson-dev`, `nlohmann-json3-dev`, `libasio-dev` (all via apt).
4. **Build:** `colcon build --packages-select rosx_introspection foxglove_bridge --symlink-install --cmake-args -DBUILD_TESTING=OFF`

**Does NOT build on macOS** — the foxglove SDK ships precompiled binaries for Linux only (x86_64 and aarch64). macOS/ARM64 is not supported.

### TensorRT Optimization (Jetson Orin Nano)

- **TRT engines auto-export on first CUDA run.** No manual export step needed. YOLOP: `_export_onnx()` → `_export_trt_engine()` (PyTorch → ONNX → TRT FP16). YOLOv8: `ultralytics model.export(format="engine", half=True)`.
- **Auto-export takes ~10 min per model on Orin Nano.** First `make full-stack` run is slow; subsequent runs start in seconds since engines are cached in volume-mounted `weights/` dir.
- **YOLOP full model fails TRT export** — the detection head causes "insufficient workspace" errors. Solution: export a segmentation-only wrapper that strips the detection head (only seg outputs are used anyway).
- **YOLOP fp16 via `.half()`** — fails because input tensor stays float32 but weights become float16. Not a viable shortcut.
- **torch.cuda.amp autocast** — 20% slower than fp32 for YOLOP on Jetson. Don't bother.
- **torch.compile** — requires Triton, which is not available on Jetson. Won't work.
- **TRT engine build** takes ~8 minutes on Jetson for YOLOP seg-only. Use `trtexec --memPoolSize=workspace:2048MiB` (not deprecated `--workspace` flag).
- **YOLOPDetector auto-detects TRT**: if `weights/yolop_seg.engine` exists, uses TRT backend; otherwise falls back to PyTorch. Works seamlessly on Mac (no engine file → PyTorch).
- **Postprocessing optimization**: argmax at model output resolution (80×80) then nearest-neighbor upsample to full resolution, instead of bilinear-upsampling float logits then argmax. Same output, less computation.
- **Delete `*.engine` to force re-export** — needed when TensorRT version changes (e.g. JetPack upgrade).

### Configurable Perception Pipeline

- `PipelineConfig` dataclass in `config.py` controls model selection, subsampling, device, CUDA streams.
- `from_yaml(path)` loads YAML config; `preset(name)` loads built-in presets.
- Setting `segmentation_model: "none"` skips YOLOP entirely — useful for detection-only use cases (~3x faster).
- CUDA stream concurrency (`cuda_streams: true`) runs YOLOP and YOLOv8 on separate CUDA streams. Implemented but not yet benchmarked on Jetson.
- All runners accept `--config` CLI arg. Makefile accepts `CONFIG=` variable.
- Backward compatible: no config arg = default behavior (YOLOP + YOLOv8n).

### RTAB-Map SLAM Integration (PR #20)

RTAB-Map SLAM was integrated into the Nav2 Docker stack. Key learnings:

**Root cause of WM=1 (only 1 keyframe):**
- RGB images (1920×1440) and depth images (256×192) had different resolutions
- RTAB-Map warning: `"RGB size modulo depth size is not 0. Ignoring depth mask for feature detection"`
- Without depth mask, feature detection was poor, leading to only 1 keyframe

**Fix:**
- Resize both RGB and depth to common resolution (512×384) before publishing
- Scale camera intrinsics to match: `fx_new = fx_old * (new_width / old_width)`
- Added `_make_camera_info_scaled()` method to `nav2_planner.py`

**RTAB-Map tuning for more keyframes:**
- `Kp/MaxFeatures: "500"` — more ORB features (was 200)
- `Vis/FeatureType: "6"` — must match `Kp/DetectorStrategy` (ORB)
- `Mem/RehearsalSimilarity: "0.2"` — lower = less keyframe merging (was 0.3)
- `Rtabmap/CreateIntermediateNodes: "true"` — denser pose graph

**Result:** WM=6-7 keyframes, `/map` OccupancyGrid published, `/mapPath` trajectory visible

**Visualization in Foxglove:**
- `/map` — OccupancyGrid (enable in 3D panel topics)
- `/mapPath` — SLAM trajectory
- Zoom in close — map is small (~4m initially)

### Architecture Decisions
<!-- Agents: record decisions and rationale here -->
- **No VLA for now.** Team considered NVIDIA Alpamayo but it requires 24GB+ VRAM (won't fit on Orin Nano's 8GB). Classical perception stack (YOLO + Nav2) is more debuggable and faster. VLA can be explored later on workstation as a parallel "advisor" that doesn't control the couch directly.
- **GPS waypoint navigation** uses Nav2's built-in GPS waypoint follower + `navsat_transform_node` from `robot_localization`. No custom code needed for WGS84→UTM conversion.
- **Perception on Jetson only.** YOLOv8n INT8 via TensorRT (not DeepStream — ultralytics handles export directly). Workstation (5090) reserved for experiments, not in the critical path.
- **Platform-specific torch via uv sources.** `pyproject.toml` uses `[tool.uv.sources]` with `sys_platform` + `platform_machine` markers: Jetson (linux/aarch64) → Jetson CUDA index, CI (linux/x86_64) + Mac → PyTorch CPU index. Jetson needs Python 3.10 (only version with CUDA wheels), Mac uses 3.12. Torch 2.9.1 on Jetson requires `libcudss` not available on JetPack 6 — pinned to `<=2.8.0`.
- **Auto device detection.** `YOLOv8Detector` auto-selects cuda > mps > cpu and prefers `.engine` (TensorRT) over `.pt` if the engine file exists alongside the weights.

---

## Questions to Resolve

- [x] ~~What motor controllers are available/being used?~~ → Flipsky Dual FSESC 6.7 (VESC6-based)
- [x] ~~Do we have wheel encoders?~~ → No standalone encoders, but VESC hall sensor tachometer provides wheel position feedback
- [ ] What is the actual couch wheel configuration? (wheel diameter, separation)
- [ ] What are the actual couch dimensions?
- [ ] Power system details? (battery voltage, capacity, max discharge current)
- [ ] Is the current TCP bridge sufficient or do we need proper Zenoh (rmw_zenoh)?

---

*Last updated: 2026-02-17*
*Current phase: Full-stack live mode working on Jetson + iPhone over USB-C (PR #24). Hardware Safety Panel (e-stop, teleop, system status) added (PR #25). Phase 4 SLAM in progress — RTAB-Map integrated with WM=6+ keyframes (PR #20). Phase 3 perception + Nav2 working end-to-end. Phase 2 EKF fuses IMU + GPS + ARKit odom + Google Maps routing. Phase 1 complete. Phase 0 hardware in progress — ESC + motor verified, programmatic control working.*
