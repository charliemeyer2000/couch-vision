---
name: rtabmap-slam
description: RTAB-Map SLAM integration with CouchVision. Use when working on visual SLAM, loop closure, keyframe tuning, or debugging map building issues.
---

# RTAB-Map SLAM Integration

## Overview

RTAB-Map (Real-Time Appearance-Based Mapping) provides visual SLAM for CouchVision. It builds an occupancy grid map from RGB-D data and publishes the `map→odom` transform.

## Architecture

```
/camera/image/compressed (from Python)
         │
         ▼
    republish node (decompresses)
         │
         ▼
/camera/image (raw RGB 512×384)
         │
         ├──────────────────────────────┐
         │                              │
         ▼                              ▼
/camera/depth/image (32FC1)    /camera/camera_info
         │                              │
         └──────────────────────────────┘
                       │
                       ▼
                  RTAB-Map
                       │
                       ├─→ /map (OccupancyGrid)
                       ├─→ /mapPath (trajectory)
                       ├─→ /tf (map→odom)
                       └─→ /mapData, /mapGraph, etc.
```

## Key Files

| File | Purpose |
|------|---------|
| `perception/launch/nav2_planner.launch.py` | ROS2 launch file with RTAB-Map node |
| `perception/config/rtabmap_params.yaml` | RTAB-Map configuration |
| `perception/src/couch_perception/nav2_planner.py` | Python script that publishes resized images |

## Running SLAM

```bash
make full-stack BAG=bags/walk_around_university_all_data.mcap
# Connect Foxglove to ws://localhost:8765
```

## Verifying SLAM is Working

Check for `WM=N` in logs where N > 1:

```bash
docker compose -f perception/docker-compose.nav2.yml logs | grep "WM="
# Good: (local map=2, WM=6)
# Bad:  (local map=1, WM=1)
```

- **WM (Working Memory):** Number of keyframes in active memory
- **local map:** Keyframes in current local map

## Critical: RGB/Depth Resolution Matching

**Root cause of WM=1:** RGB and depth images must have compatible resolutions.

Original problem:
- RGB: 1920×1440 (from iPhone camera)
- Depth: 256×192 (from iPhone LiDAR)
- RTAB-Map error: `"RGB size modulo depth size is not 0. Ignoring depth mask"`

Fix in `nav2_planner.py`:
```python
# Resize both to common resolution
target_w, target_h = 512, 384
rgb_resized = cv2.resize(frame.image, (target_w, target_h), interpolation=cv2.INTER_AREA)
depth_resized = cv2.resize(frame.depth, (target_w, target_h), interpolation=cv2.INTER_NEAREST)
```

Also scale camera intrinsics:
```python
scale_x = target_w / intrinsics.width
scale_y = target_h / intrinsics.height
fx = intrinsics.K[0, 0] * scale_x
fy = intrinsics.K[1, 1] * scale_y
cx = intrinsics.K[0, 2] * scale_x
cy = intrinsics.K[1, 2] * scale_y
```

## Key RTAB-Map Parameters

Located in `perception/config/rtabmap_params.yaml`:

```yaml
# Feature detection (ORB)
Kp/MaxFeatures: "500"           # More features = better keyframes
Kp/DetectorStrategy: "6"        # 6 = ORB
Vis/FeatureType: "6"            # Must match detector

# Keyframe management
Mem/RehearsalSimilarity: "0.2"  # Lower = more distinct keyframes (default 0.6)
Mem/IncrementalMemory: "true"   # Build map incrementally
Vis/MinInliers: "15"            # Minimum features for valid keyframe

# Graph density
Rtabmap/CreateIntermediateNodes: "true"  # Denser pose graph

# Performance
Rtabmap/DetectionRate: "1.0"    # Process 1 frame per second
Db/Sqlite3InMemory: "true"      # Faster database operations
```

## Tuning for More Keyframes

If WM stays at 1-2:

1. **Increase features:** `Kp/MaxFeatures: "1000"`
2. **Lower rehearsal threshold:** `Mem/RehearsalSimilarity: "0.1"`
3. **Check resolution matching:** Ensure RGB and depth are same size
4. **Check feature type:** `Vis/FeatureType` must equal `Kp/DetectorStrategy`

## Visualizing SLAM in Foxglove

1. Add 3D panel
2. Enable topics:
   - `/map` — OccupancyGrid (the SLAM map)
   - `/mapPath` — Trajectory (where robot has been)
   - `/tf` — Transform tree
3. Set frame to `map`
4. Zoom in — map starts small (~4m)

## Common Issues

### "Did not receive data since 5 seconds"

RTAB-Map isn't receiving synchronized messages. Check:
- Are all three topics publishing? (`/camera/image`, `/camera/depth/image`, `/camera/camera_info`)
- Are timestamps aligned? (approx_sync handles small differences)
- Is the Python script running?

### "Rejected loop closure: Not enough features"

Old keyframes have too few features. This happens when early frames were captured before good images arrived. The system will work better as more good keyframes accumulate.

### WM stuck at small number

- Check rehearsal similarity (lower = more keyframes)
- Check that CreateIntermediateNodes is true
- Verify RGB/depth resolution matching

### Map not visible in Foxglove

- Enable `/map` topic in 3D panel
- Set display frame to `map`
- Zoom way in (map is small, ~4m initially)
- Check that `/map` is publishing: `ros2 topic hz /map`

## RTAB-Map Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/map` | OccupancyGrid | 2D occupancy map |
| `/mapPath` | Path | SLAM trajectory |
| `/mapData` | MapData | Full map data for saving |
| `/mapGraph` | MapGraph | Pose graph visualization |
| `/rtabmap/republish_node_data` | Service | Republish specific nodes |

## TF Tree with SLAM

```
map (RTAB-Map publishes map→odom)
 └── odom (static: identity)
      └── base_link (static: identity)
           └── camera (static: identity)
```

RTAB-Map corrects drift by adjusting the `map→odom` transform.
