"""Nav2 path planning on MCAP bag data with EKF localization and Google Maps routing.

Processes bag through YOLOv8+YOLOP perception, runs an EKF fusing IMU+GPS,
queries Google Maps for a route to a destination on the first GPS fix,
builds a costmap, publishes it as a ROS2 OccupancyGrid for Nav2, and plans
a local path to the next waypoint on the Google Maps route.

Visualization is handled by foxglove_bridge (launched separately), which
forwards all ROS2 topics to Foxglove Studio over WebSocket.

Requires Nav2 planner_server + foxglove_bridge to be running (see launch file).
"""

import argparse
import math
import os
import threading
import time

import cv2
import numpy as np
import requests
import googlemaps
import polyline as polyline_lib
from pyproj import Transformer
from scipy.interpolate import splprep, splev

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage, Image, Imu, PointCloud2, PointField, NavSatFix
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from builtin_interfaces.msg import Time as RosTime, Duration as RosDuration

from nav2_simple_commander.robot_navigator import BasicNavigator

from couch_perception.frame_source import BagSource
from couch_perception.ekf import EKF
from couch_perception.geo import geodetic_to_enu, ROTUNDA_LAT, ROTUNDA_LON
from couch_perception.perception_pipeline import PerceptionPipeline
from couch_perception.costmap_runner import (
    build_costmap,
    GRID_CELLS,
    GRID_ORIGIN,
    GRID_RESOLUTION,
    GRID_SIZE_M,
)
from couch_perception.costmap_visualizer import costmap_to_upscaled_image


# ── Google Maps route planning ────────────────────────────────────────────

def _get_google_maps_route(
    start_lat: float, start_lon: float,
    dest_lat: float, dest_lon: float,
    api_key: str,
) -> list[tuple[float, float]] | None:
    """Query Google Maps Directions + Roads API for a snapped route.

    Returns list of (lat, lon) waypoints or None on failure.
    """
    gmaps = googlemaps.Client(key=api_key)
    origin = f"{start_lat},{start_lon}"
    destination = f"{dest_lat},{dest_lon}"

    # 1. Get directions
    try:
        directions = gmaps.directions(origin, destination, mode="walking")
        if not directions:
            print("Google Maps: no directions found.")
            return None
        enc = directions[0]["overview_polyline"]["points"]
        latlon = polyline_lib.decode(enc)
        print(f"Google Maps: {len(latlon)} direction points")
    except Exception as e:
        print(f"Google Maps Directions API failed: {e}")
        return None

    # 2. Snap to roads
    snapped = []
    chunk_size = 100
    for i in range(0, len(latlon), chunk_size):
        chunk = latlon[i:i + chunk_size]
        path_str = "|".join([f"{lat},{lon}" for lat, lon in chunk])
        url = "https://roads.googleapis.com/v1/snapToRoads"
        try:
            r = requests.get(
                url,
                params={"path": path_str, "interpolate": "true", "key": api_key},
                timeout=20,
            )
            r.raise_for_status()
            data = r.json()
            for p in data.get("snappedPoints", []):
                loc = p["location"]
                snapped.append((loc["latitude"], loc["longitude"]))
        except Exception as e:
            print(f"Roads API snap failed: {e}")

    if not snapped:
        print("Snap returned no points, using original route.")
        snapped = latlon

    print(f"Google Maps: {len(snapped)} snapped points")
    return snapped


def _route_to_enu(
    route_gps: list[tuple[float, float]],
) -> np.ndarray:
    """Convert GPS route to ENU coordinates relative to Rotunda.

    Returns (N, 2) array of (east, north) in meters.
    """
    proj_string = (
        f"+proj=tmerc +lat_0={ROTUNDA_LAT} +lon_0={ROTUNDA_LON} "
        f"+k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
    )
    tfm = Transformer.from_crs("EPSG:4326", proj_string, always_xy=True)

    xy = np.array(
        [tfm.transform(lon, lat) for lat, lon in route_gps], dtype=np.float64
    )
    return xy


def _resample_path(xy: np.ndarray, step_size: float = 0.3, smoothing: float = 5.0) -> np.ndarray:
    """Resample an (N,2) path to uniform spacing via smoothed spline interpolation.

    Args:
        xy: (N, 2) array of waypoints.
        step_size: distance between resampled points (meters).
        smoothing: spline smoothing factor (higher = smoother, 0 = exact interpolation).
    """
    if len(xy) < 2:
        return xy

    # Remove duplicate consecutive points
    diffs = np.diff(xy, axis=0)
    dists = np.linalg.norm(diffs, axis=1)
    keep = np.concatenate(([True], dists > 1e-6))
    xy = xy[keep]
    if len(xy) < 2:
        return xy

    diffs = np.diff(xy, axis=0)
    dists = np.linalg.norm(diffs, axis=1)
    cumulative = np.concatenate(([0], np.cumsum(dists)))
    total = cumulative[-1]
    if total < step_size:
        return xy

    num_points = int(total / step_size) + 1
    new_dists = np.linspace(0, total, num_points)

    try:
        k = min(3, len(xy) - 1)
        tck, _ = splprep(xy.T, u=cumulative, s=smoothing, k=k)
        new_points = splev(new_dists, tck)
        return np.column_stack(new_points)
    except Exception:
        return xy


def _find_lookahead_goal(
    route_enu: np.ndarray,
    ego_enu: np.ndarray,
    lookahead_m: float = 15.0,
) -> tuple[float, float] | None:
    """Find an interpolated point on the route exactly lookahead_m ahead of ego.

    Returns (east, north) in ENU or None if no valid point found.
    Clamps lookahead to 20m max so the goal always falls within the costmap.
    """
    if len(route_enu) == 0:
        return None

    lookahead_m = min(lookahead_m, 20.0)

    # Find closest point on route to ego
    dists = np.linalg.norm(route_enu - ego_enu[:2], axis=1)
    closest_idx = int(np.argmin(dists))

    # Walk forward along the route until we've traveled lookahead_m,
    # then interpolate to get the exact point at that distance.
    traveled = 0.0
    for i in range(closest_idx, len(route_enu) - 1):
        seg = np.linalg.norm(route_enu[i + 1] - route_enu[i])
        if seg < 1e-9:
            continue
        if traveled + seg >= lookahead_m:
            # Interpolate within this segment
            remaining = lookahead_m - traveled
            t = remaining / seg
            pt = route_enu[i] + t * (route_enu[i + 1] - route_enu[i])
            return float(pt[0]), float(pt[1])
        traveled += seg

    # If route is shorter than lookahead, use the last point
    return float(route_enu[-1, 0]), float(route_enu[-1, 1])


# ── ROS2 message helpers ──────────────────────────────────────────────────

def _stamp_from_epoch(t: float) -> RosTime:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return RosTime(sec=sec, nanosec=nanosec)


def _header(timestamp: float, frame_id: str = "map") -> Header:
    h = Header()
    h.stamp = _stamp_from_epoch(timestamp)
    h.frame_id = frame_id
    return h


# ── OccupancyGrid ──────────────────────────────────────────────────────────

def _costmap_to_occupancy_grid(grid: np.ndarray, timestamp: float) -> OccupancyGrid:
    """Convert internal grid to OccupancyGrid.

    Internal grid: grid[r,c] — row 0 = min x (forward, -X), col 0 = min y.
    OccupancyGrid: row-major, row 0 = min y, col 0 = min x.
    Transform: transpose only (row already starts at min x).
    """
    msg = OccupancyGrid()
    msg.header = _header(timestamp)
    msg.info.resolution = GRID_RESOLUTION
    msg.info.width = GRID_CELLS
    msg.info.height = GRID_CELLS
    msg.info.origin.position.x = GRID_ORIGIN
    msg.info.origin.position.y = GRID_ORIGIN
    msg.info.origin.position.z = 0.0
    msg.info.origin.orientation.w = 1.0
    og = grid.T
    msg.data = og.flatten().tolist()
    return msg


# ── PointCloud2 ────────────────────────────────────────────────────────────

_PC2_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="r", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="g", offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name="b", offset=20, datatype=PointField.FLOAT32, count=1),
    PointField(name="a", offset=24, datatype=PointField.FLOAT32, count=1),
]
_PC2_POINT_STEP = 28


def _make_pointcloud2(
    points: np.ndarray, r: float, g: float, b: float, timestamp: float
) -> PointCloud2:
    """Pack (N,3) float32 points with constant RGBA into a PointCloud2 message."""
    msg = PointCloud2()
    msg.header = _header(timestamp)

    if len(points) == 0:
        msg.height = 1
        msg.width = 0
        msg.fields = _PC2_FIELDS
        msg.point_step = _PC2_POINT_STEP
        msg.row_step = 0
        msg.data = b""
        msg.is_dense = True
        return msg

    n = len(points)
    data = np.empty((n, 7), dtype=np.float32)
    data[:, :3] = points
    data[:, 3] = r
    data[:, 4] = g
    data[:, 5] = b
    data[:, 6] = 1.0

    msg.height = 1
    msg.width = n
    msg.fields = _PC2_FIELDS
    msg.is_bigendian = False
    msg.point_step = _PC2_POINT_STEP
    msg.row_step = _PC2_POINT_STEP * n
    msg.data = data.tobytes()
    msg.is_dense = True
    return msg


# ── Markers ────────────────────────────────────────────────────────────────

def _make_goal_marker(x: float, y: float, timestamp: float) -> Marker:
    """Red cube at the goal waypoint."""
    m = Marker()
    m.header = _header(timestamp)
    m.ns = "nav"
    m.id = 0
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 0.15
    m.pose.orientation.w = 1.0
    m.scale.x = 0.3
    m.scale.y = 0.3
    m.scale.z = 0.3
    m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9)
    m.lifetime = RosDuration(sec=1, nanosec=0)
    return m


def _make_ego_marker(timestamp: float) -> Marker:
    """Small white sphere at ego position (0,0)."""
    m = Marker()
    m.header = _header(timestamp)
    m.ns = "nav"
    m.id = 1
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = 0.0
    m.pose.position.y = 0.0
    m.pose.position.z = 0.15
    m.pose.orientation.w = 1.0
    m.scale.x = 0.2
    m.scale.y = 0.2
    m.scale.z = 0.2
    m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
    m.lifetime = RosDuration(sec=1, nanosec=0)
    return m


def _make_route_path(route_ego: np.ndarray, timestamp: float) -> Path:
    """Build a nav_msgs/Path from (N,2) ego-relative route points."""
    msg = Path()
    msg.header = _header(timestamp)
    for i in range(len(route_ego)):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = float(route_ego[i, 0])
        ps.pose.position.y = float(route_ego[i, 1])
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)
    return msg


# ── Costmap overlay image ─────────────────────────────────────────────────

def _path_to_costmap_image(
    grid: np.ndarray, path: Path | None, goal_x: float, goal_y: float
) -> CompressedImage:
    """Draw the planned path and waypoint on the costmap color image."""
    img = costmap_to_upscaled_image(grid, scale=4)
    scale = 4

    if path and path.poses:
        for ps in path.poses:
            px = ps.pose.position.x
            py = ps.pose.position.y
            col = int((py - GRID_ORIGIN) / GRID_RESOLUTION) * scale
            row = int((px - GRID_ORIGIN) / GRID_RESOLUTION) * scale
            if 0 <= row < img.shape[0] and 0 <= col < img.shape[1]:
                cv2.circle(img, (col, row), 3, (255, 180, 0), -1)

    # Goal
    gc = int((goal_y - GRID_ORIGIN) / GRID_RESOLUTION) * scale
    gr = int((goal_x - GRID_ORIGIN) / GRID_RESOLUTION) * scale
    if 0 <= gr < img.shape[0] and 0 <= gc < img.shape[1]:
        cv2.circle(img, (gc, gr), 8, (0, 0, 255), -1)
        cv2.putText(img, "GOAL", (gc + 10, gr + 5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (0, 0, 255), 1)

    # Ego
    center = GRID_CELLS * scale // 2
    cv2.drawMarker(img, (center, center), (255, 255, 255),
                   cv2.MARKER_CROSS, 12, 2)

    _, buf = cv2.imencode(".png", img)
    msg = CompressedImage()
    msg.header = _header(time.time())
    msg.format = "png"
    msg.data = buf.tobytes()
    return msg


# ── Main processing ───────────────────────────────────────────────────────

def process_bag(
    bag_path: str,
    dest_lat: float = 38.036830,
    dest_lon: float = -78.503577,
    device: str | None = None,
    conf: float = 0.3,
    max_frames: int | None = None,
    subsample_drivable: int = 4,
    subsample_lane: int = 2,
    subsample_bbox: int = 8,
    playback_rate: float = 1.0,
    lookahead_m: float = 15.0,
    config: "PipelineConfig | None" = None,
) -> None:
    """Process bag with EKF localization, Google Maps routing, and Nav2 planning."""

    api_key = os.environ.get("GOOGLE_MAPS_API_KEY")
    if not api_key:
        print("WARNING: GOOGLE_MAPS_API_KEY not set. Will use fixed goal fallback.")

    # ── Open bag source ─────────────────────────────────────────────────
    source = BagSource(bag_path, playback_rate=playback_rate, max_frames=max_frames)
    streams = source.open()
    gps_fixes = streams.gps_fixes
    imu_samples = streams.imu_samples
    print(f"  GPS fixes: {len(gps_fixes)}, IMU samples: {len(imu_samples)}")

    if gps_fixes:
        g0 = gps_fixes[0]
        print(f"  First GPS: ({g0.latitude:.6f}, {g0.longitude:.6f})")

    # ── Initialize EKF ────────────────────────────────────────────────────
    ekf = EKF(
        accel_noise=2.0,
        gyro_noise=0.05,
        initial_pos_cov=1.0,
        heading_noise=0.15,
        gps_heading_noise=0.3,
        velocity_damping=0.95,
        use_imu=True,
        force_2d=True,
        max_speed=3.0,
    )

    # Convert all GPS to ENU for the EKF
    gps_enu_all = []
    gps_cov_all = []
    for g in gps_fixes:
        enu = geodetic_to_enu(g.latitude, g.longitude, g.altitude)
        gps_enu_all.append(np.array(enu))
        cov = g.position_covariance
        gps_cov_all.append(np.diag([cov[0], cov[4], cov[8]]))

    # Initialize EKF with first GPS + first IMU orientation
    if gps_fixes and imu_samples:
        ekf.initialize(gps_enu_all[0], imu_samples[0].orientation)
        print(f"  EKF initialized at ENU: ({gps_enu_all[0][0]:.1f}, {gps_enu_all[0][1]:.1f})")

    # ── Google Maps route (triggered on first GPS) ────────────────────────
    route_enu: np.ndarray | None = None  # (N, 2) ENU path

    if api_key and gps_fixes:
        g0 = gps_fixes[0]
        print(f"Querying Google Maps: ({g0.latitude:.6f}, {g0.longitude:.6f}) → ({dest_lat}, {dest_lon})")
        route_gps = _get_google_maps_route(g0.latitude, g0.longitude, dest_lat, dest_lon, api_key)
        if route_gps:
            route_xy = _route_to_enu(route_gps)
            route_enu = _resample_path(route_xy, step_size=0.5)
            print(f"Google Maps route: {len(route_enu)} points in ENU")

    # ── ROS2 setup ────────────────────────────────────────────────────────
    rclpy.init()
    node = rclpy.create_node("nav2_planner_runner")

    qos_reliable = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    qos_best_effort = QoSProfile(depth=5)

    costmap_pub = node.create_publisher(OccupancyGrid, "/costmap/occupancy_grid", qos_reliable)
    path_pub = node.create_publisher(Path, "/nav/planned_path", qos_best_effort)
    gmaps_path_pub = node.create_publisher(Path, "/nav/google_maps_path", qos_best_effort)
    goal_marker_pub = node.create_publisher(Marker, "/nav/goal_marker", qos_best_effort)
    ego_marker_pub = node.create_publisher(Marker, "/nav/ego_marker", qos_best_effort)
    costmap_img_pub = node.create_publisher(CompressedImage, "/nav/costmap_image/compressed", qos_best_effort)
    pc_drivable_pub = node.create_publisher(PointCloud2, "/perception/pointcloud/drivable", qos_best_effort)
    pc_lanes_pub = node.create_publisher(PointCloud2, "/perception/pointcloud/lanes", qos_best_effort)
    pc_det_pub = node.create_publisher(PointCloud2, "/perception/pointcloud/detections", qos_best_effort)
    camera_pub = node.create_publisher(CompressedImage, "/camera/image/compressed", qos_best_effort)
    depth_pub = node.create_publisher(Image, "/camera/depth/image", qos_best_effort)
    imu_pub = node.create_publisher(Imu, "/imu", qos_best_effort)
    gps_pub = node.create_publisher(NavSatFix, "/gps/fix", qos_best_effort)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Wait for Nav2
    print("Waiting for Nav2 planner server...")
    navigator = BasicNavigator()
    navigator._waitForNodeToActivate("planner_server")
    print("Nav2 planner is active!")

    # Load perception
    from couch_perception.config import PipelineConfig

    if config is None:
        config = PipelineConfig(
            device=device,
            detection_confidence=conf,
            subsample_drivable=subsample_drivable,
            subsample_lane=subsample_lane,
            subsample_bbox=subsample_bbox,
        )
    pipeline = PerceptionPipeline(config=config)
    print(f"YOLOv8 loaded (device={pipeline.device})")

    # Poses
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose.orientation.w = 1.0

    start_pose = PoseStamped()
    start_pose.header.frame_id = "map"
    start_pose.pose.orientation.w = 1.0

    print(f"Destination: ({dest_lat}, {dest_lon}), lookahead: {lookahead_m}m")
    print("Topics are published on ROS2. Connect Foxglove to foxglove_bridge (ws://localhost:8765).")

    # ── Frame processing loop ─────────────────────────────────────────────
    frame_num = 0

    # EKF tracking indices
    gps_idx = 0
    imu_idx = 0
    prev_imu_t: float | None = None
    prev_gps_enu: np.ndarray | None = None

    for frame in streams.frames:
        t0 = time.perf_counter()

        # ── EKF update: advance IMU predictions and GPS updates up to this frame ──
        while imu_idx < len(imu_samples) and imu_samples[imu_idx].timestamp <= frame.timestamp:
            s = imu_samples[imu_idx]
            dt = (s.timestamp - prev_imu_t) if prev_imu_t is not None else 0.0
            prev_imu_t = s.timestamp
            if ekf.initialized:
                ekf.predict(s.accel, s.gyro, s.orientation, dt)
                ekf.update_heading(s.orientation)
            imu_idx += 1

        while gps_idx < len(gps_fixes) and gps_fixes[gps_idx].timestamp <= frame.timestamp:
            if ekf.initialized:
                ekf.update_gps(gps_enu_all[gps_idx], gps_cov_all[gps_idx])
                if prev_gps_enu is not None:
                    ekf.update_gps_heading(prev_gps_enu, gps_enu_all[gps_idx])
                prev_gps_enu = gps_enu_all[gps_idx].copy()

            # Publish GPS fix as ROS2 topic
            g = gps_fixes[gps_idx]
            gps_msg = NavSatFix()
            gps_msg.header = _header(g.timestamp, "gps")
            gps_msg.latitude = g.latitude
            gps_msg.longitude = g.longitude
            gps_msg.altitude = g.altitude
            gps_pub.publish(gps_msg)

            gps_idx += 1

        # Current EKF position in ENU
        ego_enu = ekf.x[:3].copy()

        # ── Compute goal from Google Maps route ──
        if route_enu is not None:
            goal_result = _find_lookahead_goal(route_enu, ego_enu, lookahead_m)
            if goal_result:
                goal_east, goal_north = goal_result
                # Convert to ego-relative (costmap is ego-centric)
                goal_x = goal_east - ego_enu[0]
                goal_y = goal_north - ego_enu[1]
            else:
                goal_x, goal_y = lookahead_m, 0.0
        else:
            # Fallback: fixed goal straight ahead
            goal_x, goal_y = lookahead_m, 0.0

        # Clamp goal to within costmap bounds
        half = GRID_SIZE_M / 2.0 - 1.0
        goal_x = np.clip(goal_x, -half, half)
        goal_y = np.clip(goal_y, -half, half)

        # Publish raw sensor data
        cam_msg = CompressedImage()
        cam_msg.header = _header(frame.timestamp, "camera")
        cam_msg.format = "jpeg"
        _, cam_buf = cv2.imencode(".jpg", frame.image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        cam_msg.data = cam_buf.tobytes()
        camera_pub.publish(cam_msg)

        depth_msg = Image()
        depth_msg.header = _header(frame.timestamp, "camera")
        depth_msg.height, depth_msg.width = frame.depth.shape[:2]
        depth_msg.encoding = "32FC1"
        depth_msg.is_bigendian = False
        depth_msg.step = depth_msg.width * 4
        depth_msg.data = frame.depth.tobytes()
        depth_pub.publish(depth_msg)

        if frame.orientation is not None:
            imu_msg = Imu()
            imu_msg.header = _header(frame.timestamp, "imu")
            imu_msg.orientation.x = float(frame.orientation[0])
            imu_msg.orientation.y = float(frame.orientation[1])
            imu_msg.orientation.z = float(frame.orientation[2])
            imu_msg.orientation.w = float(frame.orientation[3])
            imu_pub.publish(imu_msg)

        # Publish Google Maps route in ego-relative coords
        if route_enu is not None:
            route_ego = route_enu - ego_enu[:2]
            gmaps_path_pub.publish(_make_route_path(route_ego, frame.timestamp))

        # Run perception pipeline
        result = pipeline.process_frame(frame)

        # Publish point clouds via ROS2
        if result.drivable_pts is not None:
            pc_drivable_pub.publish(
                _make_pointcloud2(result.drivable_pts, 0.0, 1.0, 0.0, frame.timestamp)
            )
        if result.lane_pts is not None:
            pc_lanes_pub.publish(
                _make_pointcloud2(result.lane_pts, 1.0, 0.0, 0.0, frame.timestamp)
            )
        if result.det_pts is not None:
            pc_det_pub.publish(
                _make_pointcloud2(result.det_pts, 1.0, 1.0, 0.0, frame.timestamp)
            )

        # Build and publish costmap
        grid = build_costmap(
            result.drivable_pts, result.lane_pts, result.det_pts,
            det_groups=result.det_groups or None,
        )
        costmap_pub.publish(_costmap_to_occupancy_grid(grid, frame.timestamp))

        # Publish markers
        goal_marker_pub.publish(_make_goal_marker(goal_x, goal_y, frame.timestamp))
        ego_marker_pub.publish(_make_ego_marker(frame.timestamp))

        # Plan path via Nav2
        goal_pose.header.stamp = _stamp_from_epoch(frame.timestamp)
        goal_pose.pose.position.x = float(goal_x)
        goal_pose.pose.position.y = float(goal_y)
        start_pose.header.stamp = goal_pose.header.stamp

        path = None
        try:
            path = navigator.getPath(start_pose, goal_pose, planner_id="GridBased")
        except Exception as e:
            if frame_num % 10 == 0:
                print(f"\nPlanner failed: {e}")

        if path and path.poses:
            path.header = _header(frame.timestamp)
            path_pub.publish(path)

        # Publish costmap overlay image
        costmap_img_pub.publish(
            _path_to_costmap_image(grid, path, goal_x, goal_y)
        )

        dt = time.perf_counter() - t0
        frame_num += 1
        if frame_num % 5 == 0:
            n_poses = len(path.poses) if path and path.poses else 0
            print(
                f"\rFrame {frame_num}: path={n_poses} poses, "
                f"{len(result.detections)} det, {1/dt:.1f} FPS, "
                f"EKF=({ego_enu[0]:.1f},{ego_enu[1]:.1f}) yaw={math.degrees(ekf.yaw):.0f}° "
                f"goal=({goal_x:.1f},{goal_y:.1f})",
                end="", flush=True,
            )

    print(f"\nDone. Processed {frame_num} frames.")
    print("Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.lifecycleShutdown()
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Nav2 path planning with EKF localization and Google Maps routing"
    )
    parser.add_argument("--bag", required=True, help="Path to .mcap bag file")
    parser.add_argument("--config", default=None, help="Path to pipeline config YAML")
    parser.add_argument("--dest-lat", type=float, default=38.036830,
                        help="Destination latitude")
    parser.add_argument("--dest-lon", type=float, default=-78.503577,
                        help="Destination longitude")
    parser.add_argument("--device", default=None, help="Torch device")
    parser.add_argument("--conf", type=float, default=0.3)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--subsample-drivable", type=int, default=4)
    parser.add_argument("--subsample-lane", type=int, default=2)
    parser.add_argument("--subsample-bbox", type=int, default=8)
    parser.add_argument("--playback-rate", type=float, default=1.0)
    parser.add_argument("--lookahead", type=float, default=15.0,
                        help="Lookahead distance on Google Maps route (meters)")
    args = parser.parse_args()

    from couch_perception.config import PipelineConfig

    config = PipelineConfig.from_yaml(args.config) if args.config else None

    process_bag(
        bag_path=args.bag,
        dest_lat=args.dest_lat,
        dest_lon=args.dest_lon,
        device=args.device,
        conf=args.conf,
        max_frames=args.max_frames,
        subsample_drivable=args.subsample_drivable,
        subsample_lane=args.subsample_lane,
        subsample_bbox=args.subsample_bbox,
        playback_rate=args.playback_rate,
        lookahead_m=args.lookahead,
        config=config,
    )


if __name__ == "__main__":
    main()
