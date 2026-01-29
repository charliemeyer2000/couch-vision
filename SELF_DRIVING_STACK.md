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
- Motor Controllers: TBD (ODrive, VESC, or similar)

**Software Stack:**
- ROS2 Jazzy
- Nav2 for planning and control (including GPS waypoint navigation)
- Perception: YOLOv8 on Jetson (DeepStream + TensorRT) for object detection
- SLAM: RTAB-Map (primary) or Isaac ROS Visual SLAM (stretch — tight on Orin Nano memory)
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

The app publishes **JPEG CompressedImage**, but SLAM nodes (RTAB-Map, Isaac ROS) expect raw `sensor_msgs/Image`. Options:
1. **Jetson-side:** Use `image_transport` `republish` node to decompress JPEG → raw
2. **iOS-side:** Add a raw image publishing mode (higher bandwidth but no decompression needed)
3. **RTAB-Map:** Can accept compressed images directly with `image_transport` plugin

Option 3 is simplest for RTAB-Map. For Isaac ROS, option 1 is needed.

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
│  │  Primary: RTAB-Map (works with compressed images via image_transport)  │ │
│  │  Stretch: Isaac ROS Visual SLAM (needs raw images + more GPU memory)   │ │
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

**Status:** Not Started — can proceed in parallel with software phases

#### Tasks
- [ ] Select and purchase motors, wheels, motor controllers, encoders
- [ ] Mount motors + wheels on couch frame
- [ ] Wire motor controllers to Jetson Orin Nano (GPIO/PWM or CAN/UART)
- [ ] Implement motor control node (ros2_control hardware interface)
- [ ] Test with `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- [ ] Add wheel encoders
- [ ] Verify `/wheel_odom` publishes correctly
- [ ] Measure and document wheel separation and wheel radius
- [ ] Determine power system (batteries, voltage regulators)

#### Hardware Specs (Fill in as determined)
```yaml
wheel_separation: _____ m        # distance between wheel centers
wheel_radius: _____ m            # wheel radius
max_wheel_velocity: _____ rad/s  # motor max speed
encoder_ticks_per_rev: _____     # if using encoders
```

#### Deliverable
Couch drives with keyboard control, `/wheel_odom` publishes from encoders

---

### Phase 1: Sensor Verification & Visualization 📱
**Goal:** iPhone data streaming end-to-end, visualized in Foxglove

**Status:** In Progress — streaming works, Foxglove connected, layout needed

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
- [ ] Verify topic data quality:
  - [ ] `/iphone/camera/back_wide/image/compressed` — check Hz and image quality
  - [ ] `/iphone/lidar/points` — PointCloud2 looks correct in 3D view
  - [ ] `/iphone/imu` — ~100Hz, orientation looks reasonable
  - [ ] `/iphone/gps/fix` — position shows on map panel
  - [ ] `/tf` — ARKit pose transforms visible in 3D panel
  - [ ] `/iphone/odom` — odometry trail visible
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

**Status:** In Progress — `nav/basic_nav_node.py` generates Google Maps routes and publishes to Foxglove

**Prerequisites:** Phase 1 (GPS streaming verified in Foxglove)

**Does NOT require hardware** — path planning and visualization can be tested without motors. The couch just won't move yet.

#### Existing Work: `nav/basic_nav_node.py` (PR #3, @wkaisertexas)
A standalone path generator already exists. It:
- Calls Google Maps Directions API for a driving route between two GPS points
- Snaps the route to roads via Google Roads API
- Projects to local ENU frame (origin: UVA Rotunda `38.035853,-78.503307`)
- Resamples with spline interpolation at 0.5m spacing
- Publishes to Foxglove directly via `foxglove-sdk` (NOT ROS2):
  - `/gps_path` (GeoJSON LineString), `/path_points` (PointCloud), `/location` (origin)
- Requires `GOOGLE_MAPS_API_KEY` env var (stored in `.env`, gitignored)

This handles path generation and visualization. Remaining work is integrating with ROS2 Nav2 for actual navigation (publishing `nav_msgs/Path`, feeding into the Nav2 planner/controller).

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
- [ ] Install `robot_localization`: `sudo apt install ros-jazzy-robot-localization`
- [ ] Configure `navsat_transform_node`:
  - [ ] Set datum (world origin GPS point) — either first fix or a configured point
  - [ ] Map `/iphone/gps/fix` → UTM local frame
  - [ ] Verify UTM output makes sense (position moves when phone moves)
- [ ] Configure EKF (see config below):
  - [ ] Fuse ARKit VIO odom + IMU + GPS
  - [ ] Verify `/odometry/filtered` is smooth
- [ ] Set up TF tree on Jetson:
  - [ ] `map` → `odom` (from SLAM or identity initially)
  - [ ] `odom` → `base_link` (from EKF)
  - [ ] `base_link` → `iphone_link` (static, from mount measurement)
- [ ] Install Nav2: `sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup`
- [ ] Configure Nav2 GPS waypoint follower
- [ ] Test: send two GPS points, verify `nav_msgs/Path` is published
- [ ] Visualize path in Foxglove 3D panel (overlay on map)
- [ ] Add path visualization to Foxglove layout

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

**Status:** Not Started

**Prerequisites:** Phase 1 (camera streaming verified)

**Does NOT require hardware** — detection runs on camera frames regardless of whether the couch moves.

#### Approach
- **YOLOv8n with INT8 quantization** via DeepStream + TensorRT on Jetson Orin Nano
- Expected: ~10-17 FPS end-to-end (sufficient for a slow-moving couch)
- Publishes `vision_msgs/Detection2DArray` on `/detections`
- Detection classes: pedestrian, vehicle, stop sign, traffic light, lane markings
- Later: feed detections into Nav2 costmap for obstacle avoidance

#### Alternative Approaches (if YOLOv8 doesn't fit)
- `jetson-inference` + `ros_deep_learning` — simpler setup, built-in ROS2 node, has Vehicle/Person/RoadSign
- DeepStream reference app — more complex but better optimized for multi-stream
- Fine-tuned model on Roboflow — if default COCO classes aren't sufficient

#### Tasks
- [ ] Install DeepStream on Jetson (if not already)
- [ ] Export YOLOv8n to TensorRT INT8 engine for Jetson
- [ ] Create ROS2 node: subscribe to `/iphone/camera/back_wide/image/compressed`, run inference, publish `/detections`
- [ ] Test detection on live camera feed
- [ ] Verify FPS is acceptable (~10+ FPS)
- [ ] Add detection overlay to Foxglove layout (image panel with bounding boxes)
- [ ] Evaluate detection quality for target classes (pedestrians, stop signs, vehicles)
- [ ] If COCO classes insufficient, fine-tune on custom dataset (Roboflow)
- [ ] Integrate detections into Nav2 costmap (obstacle layer from detections)

#### Deliverable
Live bounding boxes on camera feed in Foxglove. `/detections` topic publishing `Detection2DArray` at 10+ FPS.

---

### Phase 4: SLAM 🗺️
**Goal:** Build and localize in a map

**Status:** Not Started

**Prerequisites:** Phase 1 (camera + depth streaming), Phase 2 (EKF odometry)

#### Approach Decision
- [x] **Primary: RTAB-Map** — mature, works with RGB-D, supports compressed images via image_transport, well-documented
- [ ] **Stretch: Isaac ROS Visual SLAM** — GPU-accelerated but tight on Orin Nano (8GB shared memory), requires raw images

#### Tasks — RTAB-Map
- [ ] Install: `sudo apt install ros-jazzy-rtabmap-ros`
- [ ] Set up image_transport compressed subscriber (RTAB-Map supports this natively)
- [ ] Create launch file with correct topic remapping (see below)
- [ ] Test SLAM with manual driving (teleop or carry phone around)
- [ ] Verify map quality in Foxglove
- [ ] Test loop closure
- [ ] Save map

#### Tasks — Common
- [ ] Drive couch around entire operating area
- [ ] Verify map covers all needed spaces
- [ ] Save map: `ros2 run nav2_map_server map_saver_cli -f ~/maps/house`
- [ ] Test localization with saved map (restart and relocalize)

#### RTAB-Map Launch
```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/iphone/camera/back_wide/image/compressed \
  depth_topic:=/iphone/lidar/depth/image \
  camera_info_topic:=/iphone/camera/back_wide/camera_info \
  frame_id:=base_link \
  approx_sync:=true \
  qos:=2
```

Note: If RTAB-Map doesn't accept compressed directly on `rgb_topic`, use image_transport republish:
```bash
ros2 run image_transport republish compressed raw \
  --ros-args \
  -r in/compressed:=/iphone/camera/back_wide/image/compressed \
  -r out:=/iphone/camera/back_wide/image_raw
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

**Status:** Not Started

#### Tasks
- [ ] Add emergency stop button (hardware button → GPIO → `/e_stop` topic)
- [ ] Implement e-stop handler in motor controller
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
~/couch_ws/
├── src/
│   ├── couch_bringup/              # Launch files, params, rviz configs
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── launch/
│   │   │   ├── couch_bringup.launch.py
│   │   │   ├── sensors.launch.py
│   │   │   ├── navigation.launch.py
│   │   │   └── slam.launch.py
│   │   ├── config/
│   │   │   ├── nav2_params.yaml
│   │   │   ├── ekf_params.yaml
│   │   │   ├── navsat_params.yaml
│   │   │   └── slam_params.yaml
│   │   └── foxglove/
│   │       └── couch_layout.json
│   │
│   ├── couch_perception/           # Object detection + sensor processing
│   │   ├── setup.py
│   │   ├── package.xml
│   │   ├── couch_perception/
│   │   │   ├── __init__.py
│   │   │   ├── yolo_detector.py       # YOLOv8 inference node
│   │   │   ├── pointcloud_merger.py   # Phase 6
│   │   │   └── marker_publisher.py    # Couch bounding box marker
│   │   └── launch/
│   │       └── perception.launch.py
│   │
│   ├── couch_hardware/             # Motor control, hardware interfaces
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── src/
│   │   │   └── couch_hardware_interface.cpp
│   │   ├── include/
│   │   │   └── couch_hardware/
│   │   │       └── couch_hardware_interface.hpp
│   │   ├── urdf/
│   │   │   └── couch.urdf.xacro
│   │   └── config/
│   │       └── controllers.yaml
│   │
│   └── couch_msgs/                 # Custom messages (if needed)
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── msg/
│           └── CouchStatus.msg
│
└── maps/
    ├── house.yaml
    └── house.pgm
```

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
- [ ] What motor controllers to use? (ODrive, VESC, or simpler PWM?)
- [ ] Actual couch dimensions?
- [ ] Power system?
- [ ] Do we need the Zenoh bridge to be a proper rmw_zenoh implementation, or is the TCP bridge sufficient?

### Discovered Issues

- **Adding a new topic requires updating BOTH the iOS app AND the Python bridge.** The bridge (`bridge/ios_bridge.py`) manually parses CDR bytes for each message type. If you add a new topic/message on the iOS side but don't add a corresponding entry in `_topic_types` and a `_parse_*` method in the bridge, messages will be silently dropped (with a warning log). This is the most common mistake when adding new publishers. Always update: (1) iOS message struct, (2) CDREncoder, (3) sensor manager, (4) SensorCoordinator, (5) bridge `_topic_types` + `_parsers` + parser method.

### Helpful Commands
<!-- Agents: add useful commands here -->
- `make foxglove` — starts Foxglove WebSocket bridge on port 8765 (Linux/Jetson only). Connect from Foxglove desktop app at `ws://<jetson-ip>:8765`. Preferred over RViz2 for remote visualization — supports compressed images natively, has GPS map panel, runs on any OS.

### foxglove_bridge Build (Jetson)

foxglove_bridge is built from source on the Jetson at `~/ros2_jazzy/`. It requires:

1. **rosx_introspection** — cloned to `~/ros2_jazzy/src/rosx_introspection`. Requires a patch for Jazzy: `rosbag2_cpp/typesupport_helpers.hpp` moved to `rclcpp/typesupport_helpers.hpp`, and namespace changed from `rosbag2_cpp::` to `rclcpp::` (functions: `get_typesupport_library`, `get_message_typesupport_handle`). The `get_message_typesupport_handle` takes `SharedLibrary&` not `shared_ptr`, so dereference with `*`.
2. **foxglove-sdk** — cloned to `~/ros2_jazzy/src/foxglove-sdk`. The bridge is at `ros/src/foxglove_bridge/`.
3. **System deps:** `rapidjson-dev`, `nlohmann-json3-dev`, `libasio-dev` (all via apt).
4. **Build:** `colcon build --packages-select rosx_introspection foxglove_bridge --symlink-install --cmake-args -DBUILD_TESTING=OFF`

**Does NOT build on macOS** — the foxglove SDK ships precompiled binaries for Linux only (x86_64 and aarch64). macOS/ARM64 is not supported.

### Architecture Decisions
<!-- Agents: record decisions and rationale here -->
- **No VLA for now.** Team considered NVIDIA Alpamayo but it requires 24GB+ VRAM (won't fit on Orin Nano's 8GB). Classical perception stack (YOLO + Nav2) is more debuggable and faster. VLA can be explored later on workstation as a parallel "advisor" that doesn't control the couch directly.
- **GPS waypoint navigation** uses Nav2's built-in GPS waypoint follower + `navsat_transform_node` from `robot_localization`. No custom code needed for WGS84→UTM conversion.
- **Perception on Jetson only.** YOLOv8n INT8 via DeepStream/TensorRT. Workstation (5090) reserved for experiments, not in the critical path.

---

## Questions to Resolve

- [ ] What motor controllers are available/being used?
- [ ] What is the actual couch wheel configuration?
- [ ] Do we have wheel encoders?
- [ ] What are the actual couch dimensions?
- [ ] Power system details?
- [ ] Is the current TCP bridge sufficient or do we need proper Zenoh (rmw_zenoh)?

---

*Last updated: 2026-01-28*
*Current phase: Phase 1 in progress (layout + marker done, data verification next). Phase 0 hardware unblocked in parallel.*
