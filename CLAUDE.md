# CouchVision - iOS Sensor Streamer for ROS2

## Project Overview
iOS app that streams iPhone sensors (cameras, LiDAR, IMU, GPS, etc.) to ROS2 via TCP bridge over wired USB-C connection.

**Bundle ID**: `com.couchvision.CouchVision`
**Target**: iOS 16.0+, iPhone 12 Pro+ (for LiDAR)
**ROS2 Target**: Jazzy with rmw_cyclonedds_cpp

## Current Status
- [x] Project setup
- [x] Core architecture (protocols, abstractions)
- [x] TCP bridge (iOS → bridge → ROS2 via CycloneDDS)
- [x] Camera streaming (back wide priority)
- [x] LiDAR depth streaming
- [x] IMU/Motion sensors
- [x] GPS/Location
- [x] Other sensors (barometer, magnetometer, battery, thermal)
- [ ] Background execution
- [x] Configuration UI (basic)
- [x] Testing with ROS2 (Jetson live mode working)

## Architecture Decisions

### Publisher Abstraction
Using a `Publisher` protocol to abstract the communication layer. This allows swapping between:
1. zenoh-pico (C library wrapped in Swift) - first attempt
2. Native Swift Zenoh implementation - fallback
3. Other protocols if needed

### Sensor Manager Pattern
Each sensor type has its own manager class conforming to a `SensorProvider` protocol:
- `CameraManager` - AVFoundation cameras
- `LiDARManager` - ARKit depth
- `MotionManager` - CoreMotion (IMU, magnetometer, barometer)
- `LocationManager` - CoreLocation GPS
- `DeviceStatusManager` - Battery, thermal, proximity

Central `SensorCoordinator` manages all sensors and routes data to publishers.

### Message Serialization
Using CDR (Common Data Representation) for ROS2 compatibility. Custom `CDREncoder` class handles serialization.

### Data Pipeline: iOS App → Bridge → ROS2
The iOS app does NOT publish directly to ROS2. Data flows through **three layers that must all be updated when adding a new topic/message type:**

1. **iOS app** — Swift message struct (`CouchVision/ROS/Messages/`), CDR encoder overload + factory (`CDREncoder.swift`), sensor manager subject/publisher, SensorCoordinator subscription + publish method
2. **Python bridge** (`bridge/ios_bridge.py`) — import the ROS2 message type, add topic suffix → type mapping in `_topic_types`, add CDR parser method in `_parsers`, implement the `_parse_*` method
3. **rviz config** (`rviz/couchvision.rviz`) — add display if you want it visible by default

**If you skip the bridge, messages will be silently dropped** with a warning log. The bridge does topic matching by suffix (e.g. `/odom`, `/imu`) and needs an explicit parser for each message type because it manually deserializes CDR bytes into ROS2 Python message objects.

## Code Review Notes (Jan 2026)

**Fixed issues:**
- Removed unused `BaseSensorManager` class and `SensorProvider` protocol (sensor managers work fine standalone)
- Fixed `protected` keyword (doesn't exist in Swift)
- Extracted timestamp calculation to `TimeUtils.toUnixTimestamp()` (was duplicated 4x)
- Removed obvious/slop comments (kept useful ones like coordinate system conversions)
- Simplified `pixelBufferToROSImage` helper in LiDARManager (was duplicated)
- Cleaned up CDREncoder (condensed factory methods)

**Architecture decision:** Kept sensor managers as standalone classes rather than using inheritance. They share patterns but the complexity of a base class wasn't worth it for 3 sensors.

## Common Learnings / Gotchas

### iOS Sensor Access
- Multi-camera requires `AVCaptureMultiCamSession` (iPhone XS+)
- LiDAR requires ARKit with `ARWorldTrackingConfiguration.sceneDepth`
- Background location requires "Always" permission + `allowsBackgroundLocationUpdates = true`
- ARKit does NOT work in background - LiDAR only available when app is foreground

### Zenoh/Networking
- zenoh-swift official bindings don't exist (as of Jan 2025)
- zenoh-pico is pure C, may be compilable for iOS
- USB-C Ethernet adapter creates standard TCP/IP networking
- Personal Hotspot reverse tethering also works for wired connection

### ROS2 Message Formats
- Timestamps: Use `builtin_interfaces/Time` (sec + nanosec)
- Images: `sensor_msgs/CompressedImage` with JPEG encoding
- IMU: `sensor_msgs/Imu` with covariance matrices
- Point clouds: `sensor_msgs/PointCloud2` with fields x,y,z,intensity

### Background Execution
- iOS aggressively suspends apps
- Location updates keep app alive
- Silent audio playback as backup
- ARKit (LiDAR) stops in background

## Helpful Commands

```bash
# Full stack (Docker, runs bridge + Nav2 + perception)
make full-stack                    # live mode (iPhone → bridge → perception)
make full-stack BAG=bags/walk.mcap # bag replay mode

# Log management (stack runs in background)
make logs-bridge    # tail iOS bridge logs
make logs-nav2      # tail Nav2 planner logs
make logs           # tail all logs (interleaved)
make stop           # bring everything down

# Standalone bridge (without Docker, for local dev)
make bridge

# iOS
make xcode          # open Xcode project

# ROS2 debugging
make topics                   # list all ROS2 topics
make hz T=/iphone_charlie/imu # check topic frequency
make echo T=/iphone_charlie/odom # echo topic messages
make rviz                     # launch RViz2 with project config

# Utilities
make ip             # show IP addresses
make deploy         # pull latest code on Jetson via SSH
```

## File Structure
```
CouchVision/
├── App/
│   ├── CouchVisionApp.swift      # App entry point
│   ├── ContentView.swift         # Main UI
│   └── SettingsView.swift        # Configuration UI
├── Core/
│   ├── Protocols/
│   │   ├── Publisher.swift       # Publishing abstraction
│   │   └── SensorProvider.swift  # Sensor abstraction
│   └── Coordinator/
│       └── SensorCoordinator.swift
├── Sensors/
│   ├── CameraManager.swift
│   ├── LiDARManager.swift
│   ├── MotionManager.swift
│   ├── LocationManager.swift
│   └── DeviceStatusManager.swift
├── ROS/
│   ├── Messages/                 # ROS2 message structs
│   │   ├── Header.swift
│   │   ├── CompressedImage.swift
│   │   ├── Imu.swift
│   │   └── ...
│   └── CDREncoder.swift          # CDR serialization
├── Networking/
│   ├── ZenohPublisher.swift      # Zenoh implementation
│   └── ConnectionManager.swift   # Connection state
└── Resources/
    └── Info.plist
```

## Dependencies
- System frameworks only (AVFoundation, ARKit, CoreMotion, CoreLocation, Network)
- Potentially zenoh-pico (C library) if we can compile for iOS

## Testing Notes
- ROS2 Jazzy installed on dev Mac
- Can test Zenoh communication locally before deploying to Jetson
- Use iPhone 12 Pro or later for LiDAR testing
