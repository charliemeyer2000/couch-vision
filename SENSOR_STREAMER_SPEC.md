# iOS Sensor Streamer for ROS2 - Technical Specification

## Overview

Build a native Swift iOS application that streams all available iPhone sensor data to a ROS2 system running on a Jetson Orin Nano via a **wired USB-C connection**. The app must support background execution and publish data using the Zenoh protocol (ROS2 RMW).

## Target Platform
- **iOS**: 16.0+ (for full sensor access)
- **Devices**: iPhone 12 Pro or later (for LiDAR support)
- **Connection**: Wired USB-C (either direct USB networking or USB-C to Ethernet adapter)
- **ROS2 Target**: Jetson Orin Nano running ROS2 Jazzy with `rmw_zenoh_cpp`

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     iPhone (Swift App)                       │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │   Cameras   │  │   Motion    │  │    Location/Env     │  │
│  │ AVFoundation│  │ CoreMotion  │  │ CoreLocation/etc    │  │
│  └──────┬──────┘  └──────┬──────┘  └──────────┬──────────┘  │
│         │                │                     │             │
│         └────────────────┼─────────────────────┘             │
│                          ▼                                   │
│              ┌───────────────────────┐                       │
│              │   Sensor Manager      │                       │
│              │   (Central Hub)       │                       │
│              └───────────┬───────────┘                       │
│                          ▼                                   │
│              ┌───────────────────────┐                       │
│              │   ROS2 Message        │                       │
│              │   Serializer          │                       │
│              └───────────┬───────────┘                       │
│                          ▼                                   │
│              ┌───────────────────────┐                       │
│              │   Zenoh Publisher     │                       │
│              │   (zenoh-swift or     │                       │
│              │    custom impl)       │                       │
│              └───────────┬───────────┘                       │
│                          │                                   │
└──────────────────────────┼───────────────────────────────────┘
                           │ USB-C / Ethernet
                           ▼
┌─────────────────────────────────────────────────────────────┐
│              Jetson Orin Nano (ROS2 Jazzy)                  │
│              rmw_zenoh_cpp + Zenoh Router                   │
└─────────────────────────────────────────────────────────────┘
```

---

## Wired Connection Options

### Option 1: USB Ethernet Adapter (Recommended)
- Use Apple USB-C to Ethernet adapter or third-party
- iPhone gets static IP on local network segment with Jetson
- Simple TCP/IP networking, Zenoh works natively
- **Setup**: Configure iPhone network settings for Ethernet adapter

### Option 2: USB Network Tethering (Personal Hotspot Reverse)
- Enable Personal Hotspot on iPhone
- Connect Jetson via USB-C
- iPhone acts as network gateway
- Jetson gets IP from iPhone's DHCP
- **Note**: Requires "Allow Others to Join" but connection is wired

### Option 3: NCM/ECM USB Networking (Advanced)
- iPhone as USB network device (requires jailbreak or MDM profile)
- Creates virtual ethernet interface over USB
- Most reliable but requires additional setup

**Recommendation**: Start with Option 1 (USB Ethernet adapter) for simplicity.

---

## Sensors to Stream

### 1. Cameras

| Camera | Topic | Message Type | Notes |
|--------|-------|--------------|-------|
| Front (TrueDepth) | `/iphone/camera/front/image/compressed` | `sensor_msgs/CompressedImage` | 12MP, face tracking capable |
| Back Wide | `/iphone/camera/back_wide/image/compressed` | `sensor_msgs/CompressedImage` | Main camera, 12-48MP |
| Back Ultra Wide | `/iphone/camera/back_ultrawide/image/compressed` | `sensor_msgs/CompressedImage` | 0.5x zoom |
| Back Telephoto | `/iphone/camera/back_telephoto/image/compressed` | `sensor_msgs/CompressedImage` | 2-5x zoom (Pro models) |
| Camera Info (each) | `/iphone/camera/*/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics, distortion |

**Implementation Notes**:
- Use `AVCaptureSession` with `AVCaptureMultiCamSession` for simultaneous capture
- Compress to JPEG (quality configurable, default 80%)
- Include timestamps synchronized to device clock
- Support resolution selection: 1080p, 720p, 480p
- Support frame rate selection: 30fps, 15fps, 10fps

```swift
// Key frameworks
import AVFoundation
import CoreImage
```

### 2. LiDAR Depth

| Data | Topic | Message Type | Notes |
|------|-------|--------------|-------|
| Point Cloud | `/iphone/lidar/points` | `sensor_msgs/PointCloud2` | Sparse depth points |
| Depth Image | `/iphone/lidar/depth/image` | `sensor_msgs/Image` | 32FC1 format |
| Confidence | `/iphone/lidar/confidence/image` | `sensor_msgs/Image` | Confidence map |

**Implementation Notes**:
- Use `ARKit` with `ARWorldTrackingConfiguration`
- Enable `sceneDepth` for LiDAR data
- Convert `ARDepthData` to ROS PointCloud2 format
- LiDAR runs at ~60Hz but can be throttled
- Depth range: 0.2m - 5m

```swift
import ARKit
```

### 3. IMU (Inertial Measurement Unit)

| Data | Topic | Message Type | Rate |
|------|-------|--------------|------|
| Combined IMU | `/iphone/imu` | `sensor_msgs/Imu` | Up to 100Hz |
| Raw Accelerometer | `/iphone/accelerometer` | `geometry_msgs/Vector3Stamped` | Up to 100Hz |
| Raw Gyroscope | `/iphone/gyroscope` | `geometry_msgs/Vector3Stamped` | Up to 100Hz |

**Implementation Notes**:
- Use `CMMotionManager` from CoreMotion
- Set `accelerometerUpdateInterval` and `gyroUpdateInterval`
- Device motion gives sensor-fused orientation
- Include covariance matrices (use device specs or estimate)

```swift
import CoreMotion

let motionManager = CMMotionManager()
motionManager.deviceMotionUpdateInterval = 1.0 / 100.0 // 100Hz
motionManager.startDeviceMotionUpdates(using: .xArbitraryCorrectedZVertical, to: queue) { motion, error in
    // Publish IMU message
}
```

### 4. Magnetometer (Compass)

| Data | Topic | Message Type | Rate |
|------|-------|--------------|------|
| Magnetic Field | `/iphone/magnetic_field` | `sensor_msgs/MagneticField` | Up to 100Hz |
| Heading | `/iphone/heading` | `std_msgs/Float64` | ~10Hz |

**Implementation Notes**:
- Use `CMMotionManager.magnetometerData` for raw field
- Use `CLLocationManager.heading` for calibrated heading
- Magnetic field in microteslas (µT)

### 5. GPS / Location

| Data | Topic | Message Type | Notes |
|------|-------|--------------|-------|
| GPS Fix | `/iphone/gps/fix` | `sensor_msgs/NavSatFix` | Lat/Lon/Alt |
| Velocity | `/iphone/gps/velocity` | `geometry_msgs/TwistStamped` | Speed/Course |

**Implementation Notes**:
- Use `CLLocationManager` with `desiredAccuracy = kCLLocationAccuracyBest`
- Request "Always" authorization for background
- Include horizontal/vertical accuracy in covariance

```swift
import CoreLocation

let locationManager = CLLocationManager()
locationManager.desiredAccuracy = kCLLocationAccuracyBest
locationManager.allowsBackgroundLocationUpdates = true
locationManager.startUpdatingLocation()
```

### 6. Barometer (Pressure/Altitude)

| Data | Topic | Message Type | Rate |
|------|-------|--------------|------|
| Pressure | `/iphone/pressure` | `sensor_msgs/FluidPressure` | ~1Hz |
| Relative Altitude | `/iphone/altitude` | `std_msgs/Float64` | ~1Hz |

**Implementation Notes**:
- Use `CMAltimeter` from CoreMotion
- Pressure in kilopascals (kPa)
- Relative altitude change is very accurate

```swift
let altimeter = CMAltimeter()
altimeter.startRelativeAltitudeUpdates(to: queue) { data, error in
    // data.pressure (kPa), data.relativeAltitude (meters)
}
```

### 7. Proximity Sensor

| Data | Topic | Message Type | Notes |
|------|-------|--------------|-------|
| Proximity | `/iphone/proximity` | `std_msgs/Bool` | Near/Far only |

**Implementation Notes**:
- Use `UIDevice.current.isProximityMonitoringEnabled`
- Binary sensor only (near/far)
- Observe `proximityStateDidChangeNotification`

### 8. Device Status

| Data | Topic | Message Type | Rate |
|------|-------|--------------|------|
| Battery | `/iphone/battery` | `sensor_msgs/BatteryState` | ~1Hz |
| Thermal State | `/iphone/thermal` | `std_msgs/Int32` | On change |

**Implementation Notes**:
- Battery: `UIDevice.current.batteryLevel` and `batteryState`
- Thermal: `ProcessInfo.processInfo.thermalState` (nominal/fair/serious/critical)

### 9. TF (Transform) Data

| Data | Topic | Message Type | Notes |
|------|-------|--------------|-------|
| Device Pose | `/tf` | `tf2_msgs/TFMessage` | From ARKit |
| Static Transforms | `/tf_static` | `tf2_msgs/TFMessage` | Camera extrinsics |

**Implementation Notes**:
- Publish device pose from ARKit world tracking
- Static transforms for camera positions relative to device frame
- Frame IDs: `iphone_base_link`, `iphone_camera_front`, `iphone_camera_back`, `iphone_lidar`, etc.

---

## Background Execution

iOS restricts background execution. Use these strategies:

### Required Capabilities (Info.plist)
```xml
<key>UIBackgroundModes</key>
<array>
    <string>location</string>
    <string>audio</string>
    <string>external-accessory</string>
</array>
```

### Strategy 1: Location Updates (Primary)
- Request "Always" location permission
- Enable `allowsBackgroundLocationUpdates = true`
- GPS updates keep app alive indefinitely

### Strategy 2: Silent Audio Session (Backup)
- Play silent audio to prevent suspension
- Use `AVAudioSession` with `.playback` category

### Strategy 3: Background Task
- Use `BGProcessingTask` for periodic work
- Limited to ~30 seconds unless using above methods

### Implementation
```swift
// In AppDelegate or SceneDelegate
func setupBackgroundExecution() {
    // Audio session for background
    let audioSession = AVAudioSession.sharedInstance()
    try? audioSession.setCategory(.playback, mode: .default, options: .mixWithOthers)
    try? audioSession.setActive(true)

    // Location for background
    locationManager.allowsBackgroundLocationUpdates = true
    locationManager.pausesLocationUpdatesAutomatically = false
}
```

---

## Zenoh Communication Layer

### Option A: Use zenoh-swift (Recommended if available)
Check if Eclipse Zenoh has Swift bindings: https://github.com/eclipse-zenoh

```swift
// Hypothetical zenoh-swift usage
import Zenoh

let session = try Zenoh.open(config)
let publisher = session.declarePublisher(keyexpr: "iphone/imu")
publisher.put(encodedIMUMessage)
```

### Option B: Implement Zenoh Protocol Manually
Zenoh uses a custom protocol over TCP. Implement:
1. Session establishment (SCOUT, HELLO, INIT, OPEN)
2. Publishing (PUT messages)
3. Key expression handling

### Option C: HTTP/WebSocket Bridge (Simplest)
Create a lightweight bridge on the Jetson that:
1. Accepts HTTP POST or WebSocket from iPhone
2. Republishes to ROS2 via rclcpp

```
iPhone --[HTTP/WS]--> Bridge Node --[ROS2]--> Topics
```

**Recommendation**: Start with Option C for rapid development, then optimize to Option A/B.

---

## ROS2 Message Serialization

Use CDR (Common Data Representation) serialization for ROS2 compatibility.

### Key Message Formats

#### sensor_msgs/CompressedImage
```
Header header
string format          # "jpeg" or "png"
uint8[] data          # compressed image data
```

#### sensor_msgs/Imu
```
Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
```

#### sensor_msgs/NavSatFix
```
Header header
uint8 status
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type
```

### Serialization Library
- Implement minimal CDR encoder in Swift
- Or use MessagePack/JSON and convert on Jetson side

---

## Configuration UI

### Settings Screen
- **Connection**
  - Host IP (auto-detect or manual)
  - Port (default 7447)
  - ROS Domain ID (default 0)
  - Connection type (USB Ethernet / WiFi)

- **Cameras**
  - Enable/disable each camera
  - Resolution (1080p / 720p / 480p)
  - Frame rate (30 / 15 / 10 fps)
  - JPEG quality (50-100%)

- **Sensors**
  - Enable/disable each sensor
  - IMU rate (100 / 50 / 25 Hz)
  - GPS accuracy mode

- **Advanced**
  - Topic prefix (default `/iphone`)
  - QoS settings (reliable / best effort)
  - Buffer sizes

### Status Display
- Connection status (connected/disconnected)
- Data rates per sensor
- Battery usage estimate
- Thermal state warning

---

## Project Structure

```
SensorStreamer/
├── App/
│   ├── SensorStreamerApp.swift
│   ├── ContentView.swift
│   └── SettingsView.swift
├── Sensors/
│   ├── SensorManager.swift          # Central coordinator
│   ├── CameraManager.swift          # AVFoundation cameras
│   ├── LiDARManager.swift           # ARKit depth
│   ├── MotionManager.swift          # CoreMotion IMU/mag/baro
│   ├── LocationManager.swift        # CoreLocation GPS
│   └── DeviceStatusManager.swift    # Battery, thermal, proximity
├── ROS/
│   ├── ROSMessages.swift            # Message struct definitions
│   ├── CDRSerializer.swift          # CDR encoding
│   └── ZenohPublisher.swift         # Zenoh communication
├── Networking/
│   ├── ConnectionManager.swift      # TCP/UDP handling
│   └── USBNetworkDetector.swift     # Detect wired connection
├── Background/
│   └── BackgroundTaskManager.swift  # Background execution
├── Utils/
│   ├── TimeSync.swift               # Timestamp handling
│   └── Extensions.swift
└── Resources/
    └── Info.plist                   # Background modes, permissions
```

---

## Required Permissions (Info.plist)

```xml
<!-- Camera -->
<key>NSCameraUsageDescription</key>
<string>Stream camera to ROS2</string>

<!-- Location -->
<key>NSLocationWhenInUseUsageDescription</key>
<string>Stream GPS to ROS2</string>
<key>NSLocationAlwaysAndWhenInUseUsageDescription</key>
<string>Stream GPS in background to ROS2</string>

<!-- Motion -->
<key>NSMotionUsageDescription</key>
<string>Stream IMU data to ROS2</string>

<!-- Background Modes -->
<key>UIBackgroundModes</key>
<array>
    <string>location</string>
    <string>audio</string>
</array>
```

---

## Testing Checklist

- [ ] All cameras stream independently
- [ ] Multi-camera simultaneous streaming works
- [ ] LiDAR depth and point cloud valid
- [ ] IMU data at correct rate with valid covariance
- [ ] GPS fix with accuracy
- [ ] Magnetometer calibrated
- [ ] Barometer pressure readings
- [ ] Battery state updates
- [ ] Thermal state warnings
- [ ] Background execution sustained >10 minutes
- [ ] Wired connection stable
- [ ] Reconnection after cable disconnect
- [ ] Memory usage stable (no leaks)
- [ ] CPU/thermal acceptable

---

## Reference: Conduit App

The existing Conduit app (https://www.youtalk.jp/conduit/) provides similar functionality. Key differences in this implementation:

1. **Wired connection** vs WiFi only
2. **Background execution** vs foreground only
3. **All cameras** vs single camera
4. **Full sensor suite** vs limited sensors
5. **Open source** vs closed source

---

## Dependencies

### Swift Packages
- None required for core functionality (use system frameworks)
- Optional: SwiftProtobuf (if using protobuf serialization)
- Optional: Network.framework (for USB network detection)

### System Frameworks
- AVFoundation (cameras)
- ARKit (LiDAR)
- CoreMotion (IMU, magnetometer, barometer)
- CoreLocation (GPS)
- UIKit (device status)
- Network (connection management)

---

## Getting Started

1. Create new Xcode project (iOS App, Swift, SwiftUI)
2. Add required capabilities in Signing & Capabilities:
   - Background Modes (Location, Audio)
   - Camera
   - Access WiFi Information (for network detection)
3. Implement SensorManager first with mock publishers
4. Add Zenoh/networking layer
5. Implement each sensor manager
6. Add UI for configuration
7. Test background execution
8. Test with Jetson Orin Nano

---

## Open Questions for Implementation

1. **Zenoh Swift bindings**: Are official bindings available, or need custom implementation?
2. **USB networking**: Which method works best with Jetson (Ethernet adapter vs tethering)?
3. **Multi-camera limits**: How many cameras can stream simultaneously without thermal throttling?
4. **ARKit in background**: Does LiDAR work when app is backgrounded?

---

*Last updated: January 2025*
*Target: ROS2 Jazzy + rmw_zenoh_cpp*
