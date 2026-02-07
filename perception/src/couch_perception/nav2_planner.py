"""Nav2 path planning with GPS localization and phone-orientation heading.

Supports two modes:
  - Bag replay: `--bag path.mcap` reads from a recorded MCAP file
  - Live: omit `--bag` to subscribe to live ROS2 topics from the iOS bridge

Runs YOLOv8+YOLOP perception, uses GPS for position and phone orientation
for heading, queries Google Maps for a route, builds costmaps, and plans
local paths via Nav2.

Visualization via foxglove_bridge (ws://localhost:8765).
"""

import argparse
import bisect
import copy
import json
import math
import os
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor

import cv2
import numpy as np
import requests
import googlemaps
import polyline as polyline_lib
from pyproj import Transformer
from scipy.interpolate import splprep, splev
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, PointCloud2, PointField, NavSatFix
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA, String
from builtin_interfaces.msg import Time as RosTime, Duration as RosDuration

from nav2_simple_commander.robot_navigator import BasicNavigator

from couch_perception.bag_reader import CameraIntrinsics, GpsFix, ImuSample, OdomSample
from couch_perception.config import PipelineConfig
from couch_perception.frame_model import load_mount_frame_model
from couch_perception.frame_source import BagSource, LiveSource, SensorStreams
from couch_perception.geo import geodetic_to_enu, ROTUNDA_LAT, ROTUNDA_LON
from couch_perception.gpu_utils import resize_image, resize_depth
from couch_perception.perception_pipeline import PerceptionPipeline
from couch_perception.projection import (
    default_camera_to_base_rotation,
)
from couch_perception.costmap import (
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


_TMERC_PROJ = (
    f"+proj=tmerc +lat_0={ROTUNDA_LAT} +lon_0={ROTUNDA_LON} "
    f"+k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
)


def _enu_to_geodetic(x: float, y: float) -> tuple[float, float]:
    """Convert ENU (x=east, y=north) back to (lat, lon)."""
    tfm = Transformer.from_crs(_TMERC_PROJ, "EPSG:4326", always_xy=True)
    lon, lat = tfm.transform(x, y)
    return lat, lon


def _route_to_enu(route_gps: list[tuple[float, float]]) -> np.ndarray:
    """Convert GPS route to ENU coordinates relative to Rotunda. Returns (N, 2)."""
    tfm = Transformer.from_crs("EPSG:4326", _TMERC_PROJ, always_xy=True)
    return np.array(
        [tfm.transform(lon, lat) for lat, lon in route_gps], dtype=np.float64
    )


def _resample_path(xy: np.ndarray, step_size: float = 0.3, smoothing: float = 5.0) -> np.ndarray:
    """Resample an (N,2) path to uniform spacing via smoothed spline interpolation."""
    if len(xy) < 2:
        return xy
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
    route_enu: np.ndarray, ego_enu: np.ndarray, lookahead_m: float = 15.0,
) -> tuple[float, float] | None:
    """Find an interpolated point on the route exactly lookahead_m ahead of ego."""
    if len(route_enu) == 0:
        return None
    lookahead_m = min(lookahead_m, 20.0)
    dists = np.linalg.norm(route_enu - ego_enu[:2], axis=1)
    closest_idx = int(np.argmin(dists))

    traveled = 0.0
    for i in range(closest_idx, len(route_enu) - 1):
        seg = np.linalg.norm(route_enu[i + 1] - route_enu[i])
        if seg < 1e-9:
            continue
        if traveled + seg >= lookahead_m:
            remaining = lookahead_m - traveled
            t = remaining / seg
            pt = route_enu[i] + t * (route_enu[i + 1] - route_enu[i])
            return float(pt[0]), float(pt[1])
        traveled += seg

    return float(route_enu[-1, 0]), float(route_enu[-1, 1])


def _wrap_angle(angle_rad: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


_GPS_HEADING_MIN_DIST_M = 0.8
_MAX_IMU_GPS_TIME_SKEW_S = 1.0
_DEFAULT_LOCAL_FRAME_YAW_OFFSET_RAD = -1.5707963267948966
_DEFAULT_LOCAL_FRAME_YAW_OFFSET_DEG = math.degrees(_DEFAULT_LOCAL_FRAME_YAW_OFFSET_RAD)


def _heading_from_phone_orientation(
    orientation_quat: np.ndarray, base_forward_axis_in_imu: np.ndarray
) -> float | None:
    """Estimate heading from phone quaternion using the URDF mount frame.

    Returns heading in ENU convention: yaw = atan2(east, north).
    """
    if orientation_quat.shape[0] != 4:
        return None
    # iOS quaternion in this pipeline is imu->world (validated against bag data).
    rot_world_from_imu = Rotation.from_quat(orientation_quat)
    forward_world = rot_world_from_imu.apply(base_forward_axis_in_imu)
    east = float(forward_world[0])
    north = float(forward_world[1])
    norm = math.hypot(east, north)
    if norm < 1e-6:
        return None
    return math.atan2(east / norm, north / norm)


def _enu_to_body_xy(
    delta_east: float, delta_north: float, heading_enu_rad: float
) -> tuple[float, float]:
    """Rotate ENU delta into body frame (x=forward, y=left)."""
    sin_h = math.sin(heading_enu_rad)
    cos_h = math.cos(heading_enu_rad)
    x_forward = delta_east * sin_h + delta_north * cos_h
    y_left = -delta_east * cos_h + delta_north * sin_h
    return x_forward, y_left


def _rotate_xy(x: float, y: float, yaw_rad: float) -> tuple[float, float]:
    """Rotate a local-frame XY vector around +Z by yaw_rad."""
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return x * c - y * s, x * s + y * c


def _rotate_xy_points(points: np.ndarray | None, yaw_rad: float) -> np.ndarray | None:
    """Rotate Nx3 points around +Z, preserving Z."""
    if points is None or len(points) == 0:
        return points
    out = points.copy()
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    x = points[:, 0]
    y = points[:, 1]
    out[:, 0] = x * c - y * s
    out[:, 1] = x * s + y * c
    return out


def _rotate_xy_path(path_xy: np.ndarray, yaw_rad: float) -> np.ndarray:
    """Rotate an Nx2 XY path around +Z."""
    if len(path_xy) == 0:
        return path_xy
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    out = path_xy.copy()
    x = path_xy[:, 0]
    y = path_xy[:, 1]
    out[:, 0] = x * c - y * s
    out[:, 1] = x * s + y * c
    return out


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


def _costmap_to_occupancy_grid(
    grid: np.ndarray, timestamp: float, frame_id: str
) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header = _header(timestamp, frame_id)
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
    points: np.ndarray,
    r: float,
    g: float,
    b: float,
    timestamp: float,
    frame_id: str,
) -> PointCloud2:
    msg = PointCloud2()
    msg.header = _header(timestamp, frame_id)
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


def _make_goal_marker(x: float, y: float, timestamp: float, frame_id: str) -> Marker:
    m = Marker()
    m.header = _header(timestamp, frame_id)
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


def _make_ego_marker(timestamp: float, frame_id: str) -> Marker:
    m = Marker()
    m.header = _header(timestamp, frame_id)
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


def _make_route_path(
    route_ego: np.ndarray, timestamp: float, frame_id: str, max_points: int = 200
) -> Path:
    msg = Path()
    msg.header = _header(timestamp, frame_id)
    step = max(1, len(route_ego) // max_points)
    for i in range(0, len(route_ego), step):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = float(route_ego[i, 0])
        ps.pose.position.y = float(route_ego[i, 1])
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)
    return msg


def _path_to_costmap_image(
    grid: np.ndarray, path: Path | None, goal_x: float, goal_y: float
) -> CompressedImage:
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

    gc = int((goal_y - GRID_ORIGIN) / GRID_RESOLUTION) * scale
    gr = int((goal_x - GRID_ORIGIN) / GRID_RESOLUTION) * scale
    if 0 <= gr < img.shape[0] and 0 <= gc < img.shape[1]:
        cv2.circle(img, (gc, gr), 8, (0, 0, 255), -1)
        cv2.putText(img, "GOAL", (gc + 10, gr + 5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (0, 0, 255), 1)

    center = GRID_CELLS * scale // 2
    cv2.drawMarker(img, (center, center), (255, 255, 255),
                   cv2.MARKER_CROSS, 12, 2)

    _, buf = cv2.imencode(".png", img)
    msg = CompressedImage()
    msg.header = _header(time.time())
    msg.format = "png"
    msg.data = buf.tobytes()
    return msg


# ── Nav2Planner ───────────────────────────────────────────────────────────

class Nav2Planner:
    """Perception + GPS + phone heading + Google Maps + Nav2 path planning.

    Consumes SensorStreams from any source (BagSource or LiveSource) and
    publishes costmaps, planned paths, and perception results via ROS2.
    """

    def __init__(
        self,
        node: Node,
        config: PipelineConfig,
        dest_lat: float = 38.036830,
        dest_lon: float = -78.503577,
        lookahead_m: float = 8.0,
        republish_sensors: bool = True,
        camera_mount_mode: str = "handheld",
        orientation_mode: str = "gravity",
        local_frame_yaw_offset_rad: float = _DEFAULT_LOCAL_FRAME_YAW_OFFSET_RAD,
    ) -> None:
        self._node = node
        self._dest_lat = dest_lat
        self._dest_lon = dest_lon
        self._lookahead_m = lookahead_m
        self._republish_sensors = republish_sensors
        self._camera_mount_mode = camera_mount_mode
        self._orientation_mode = orientation_mode
        self._local_frame_yaw_offset_rad = float(local_frame_yaw_offset_rad)
        self._local_frame_yaw_offset_deg = math.degrees(self._local_frame_yaw_offset_rad)

        mount = load_mount_frame_model()
        self._base_forward_axis_imu = mount.base_forward_axis_in_imu
        if camera_mount_mode == "urdf":
            camera_mount = load_mount_frame_model(imu_link="camera")
            self._camera_to_base_rotation = camera_mount.rotation_base_from_imu
        elif camera_mount_mode == "handheld":
            self._camera_to_base_rotation = default_camera_to_base_rotation()
        elif camera_mount_mode == "identity":
            self._camera_to_base_rotation = np.eye(3, dtype=np.float64)
        else:
            raise ValueError(f"Unsupported camera_mount_mode: {camera_mount_mode}")
        print(
            "Mount frame loaded:",
            f"urdf={mount.urdf_path}",
            f"base_forward_in_imu={self._base_forward_axis_imu}",
            f"camera_to_base=\n{self._camera_to_base_rotation}",
            f"camera_mount_mode={self._camera_mount_mode}",
            f"orientation_mode={self._orientation_mode}",
            f"imu_height={mount.translation_base_from_imu_m[2]:.2f}m",
            f"local_frame_yaw_offset={self._local_frame_yaw_offset_deg:.1f}deg",
        )

        # Perception
        self._pipeline = PerceptionPipeline(
            config=config,
            camera_to_base_rotation=self._camera_to_base_rotation,
            orientation_mode=self._orientation_mode,
        )
        print(f"YOLOv8 loaded (device={self._pipeline.device})")

        # Google Maps route (resolved on first GPS fix)
        self._api_key = os.environ.get("GOOGLE_MAPS_API_KEY")
        if not self._api_key:
            print("WARNING: GOOGLE_MAPS_API_KEY not set. Will use fixed goal fallback.")
        self._route_enu: np.ndarray | None = None
        self._route_resolved = False

        # Nav2 — manually manage lifecycle transitions since autostart=false
        # (avoids race where lifecycle_manager times out configuring planner_server)
        print("Waiting for Nav2 planner server...")
        self._navigator = BasicNavigator()
        from lifecycle_msgs.srv import ChangeState, GetState
        from lifecycle_msgs.msg import Transition

        get_state = node.create_client(GetState, "/planner_server/get_state")
        change_state = node.create_client(ChangeState, "/planner_server/change_state")
        while not get_state.wait_for_service(timeout_sec=2.0):
            print("  planner_server not yet available...")
        change_state.wait_for_service(timeout_sec=5.0)

        def _get_state() -> int:
            f = get_state.call_async(GetState.Request())
            while not f.done():
                time.sleep(0.1)
            return f.result().current_state.id if f.result() else -1

        def _change_state(transition_id: int) -> bool:
            req = ChangeState.Request()
            req.transition = Transition(id=transition_id)
            f = change_state.call_async(req)
            while not f.done():
                time.sleep(0.1)
            return f.result().success if f.result() else False

        state = _get_state()
        # 1=unconfigured, 2=inactive, 3=active
        if state == 1:
            print("  Configuring planner_server...")
            _change_state(Transition.TRANSITION_CONFIGURE)
            while _get_state() != 2:
                time.sleep(0.5)
        state = _get_state()
        if state == 2:
            print("  Activating planner_server...")
            _change_state(Transition.TRANSITION_ACTIVATE)
            while _get_state() != 3:
                time.sleep(0.5)
        node.destroy_client(get_state)
        node.destroy_client(change_state)
        print("Nav2 planner is active!")

        # ROS2 publishers
        qos_reliable = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        qos = QoSProfile(depth=5)

        self._costmap_pub = node.create_publisher(OccupancyGrid, "/costmap/occupancy_grid", qos_reliable)
        self._path_pub = node.create_publisher(Path, "/nav/planned_path", qos)
        self._gmaps_path_pub = node.create_publisher(Path, "/nav/google_maps_path", qos)
        self._goal_marker_pub = node.create_publisher(Marker, "/nav/goal_marker", qos)
        self._ego_marker_pub = node.create_publisher(Marker, "/nav/ego_marker", qos)
        self._costmap_img_pub = node.create_publisher(CompressedImage, "/nav/costmap_image/compressed", qos)
        self._pc_drivable_pub = node.create_publisher(PointCloud2, "/perception/pointcloud/drivable", qos)
        self._pc_lanes_pub = node.create_publisher(PointCloud2, "/perception/pointcloud/lanes", qos)
        self._pc_det_pub = node.create_publisher(PointCloud2, "/perception/pointcloud/detections", qos)

        # Dynamic destination control (from Foxglove panel)
        self._status_pub = node.create_publisher(String, "/nav/status", qos)
        self._dest_sub = node.create_subscription(
            String, "/nav/set_destination", self._on_destination_command, qos,
        )
        self._status_timer = node.create_timer(1.0, self._publish_status)
        self._start_override: tuple[float, float] | None = None
        self._clicked_sub = node.create_subscription(
            PointStamped, "/clicked_point", self._on_clicked_point, qos,
        )

        if republish_sensors:
            self._camera_pub = node.create_publisher(CompressedImage, "/camera/image/compressed", qos)
            self._camera_info_pub = node.create_publisher(CameraInfo, "/camera/camera_info", qos)
            self._depth_pub = node.create_publisher(Image, "/camera/depth/image", qos)
            self._odom_pub = node.create_publisher(Odometry, "/odom", qos)
            self._imu_pub = node.create_publisher(Imu, "/imu", qos)
            self._gps_pub = node.create_publisher(NavSatFix, "/gps/fix", qos)

        # Poses
        self._goal_pose = PoseStamped()
        self._goal_pose.header.frame_id = "map"
        self._goal_pose.pose.orientation.w = 1.0
        self._start_pose = PoseStamped()
        self._start_pose.header.frame_id = "map"
        self._start_pose.pose.orientation.w = 1.0

        # Async Nav2 planning
        self._planning_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="nav2_plan")
        self._plan_future: Future | None = None
        self._latest_path: Path | None = None

        # Localization tracking indices/state (GPS position + phone heading)
        self._gps_idx = 0
        self._imu_idx = 0
        self._gps_enu_cache: list[np.ndarray] = []
        self._latest_gps_enu: np.ndarray | None = None
        self._prev_gps_enu_for_heading: np.ndarray | None = None
        self._latest_course_heading: float | None = None

        self._latest_phone_heading_raw: float | None = None
        self._latest_phone_heading_timestamp: float | None = None
        self._heading_offset_enu_from_phone: float | None = None
        self._latest_heading_enu: float | None = None

        self._frame_num = 0

    def _on_clicked_point(self, msg: PointStamped) -> None:
        """Handle clicked point from Foxglove 3D panel — set as new destination."""
        x, y = msg.point.x, msg.point.y
        lat, lon = _enu_to_geodetic(x, y)
        print(f"\nClicked point: ENU=({x:.1f}, {y:.1f}) → dest=({lat:.6f}, {lon:.6f})")
        self._dest_lat = lat
        self._dest_lon = lon
        self._route_resolved = False
        self._route_enu = None

    def _on_destination_command(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._dest_lat = data["dest_lat"]
            self._dest_lon = data["dest_lon"]
            if data.get("api_key"):
                self._api_key = data["api_key"]
            if "lookahead_m" in data:
                self._lookahead_m = data["lookahead_m"]
            if data.get("start_lat") is not None and data.get("start_lon") is not None:
                self._start_override = (data["start_lat"], data["start_lon"])
            self._route_resolved = False
            self._route_enu = None
            print(f"\nDestination updated: ({self._dest_lat}, {self._dest_lon})")
        except Exception as e:
            print(f"\nFailed to parse destination command: {e}")

    def _publish_status(self) -> None:
        status = {
            "api_key_configured": bool(self._api_key),
            "route_resolved": self._route_resolved,
            "current_dest_lat": self._dest_lat,
            "current_dest_lon": self._dest_lon,
            "route_points": len(self._route_enu) if self._route_enu is not None else 0,
            "ekf_initialized": self._latest_gps_enu is not None,  # Backward-compatible field.
            "heading_calibrated": self._heading_offset_enu_from_phone is not None,
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    def _make_camera_info(
        self,
        intrinsics: CameraIntrinsics,
        timestamp: float,
        target_size: tuple[int, int] | None = None,
    ) -> CameraInfo:
        """Convert CameraIntrinsics to ROS CameraInfo, optionally scaled to target resolution."""
        if target_size:
            target_w, target_h = target_size
            scale_x = target_w / intrinsics.width
            scale_y = target_h / intrinsics.height
        else:
            target_w, target_h = intrinsics.width, intrinsics.height
            scale_x = scale_y = 1.0

        fx = intrinsics.K[0, 0] * scale_x
        fy = intrinsics.K[1, 1] * scale_y
        cx = intrinsics.K[0, 2] * scale_x
        cy = intrinsics.K[1, 2] * scale_y

        msg = CameraInfo()
        msg.header.stamp.sec = int(timestamp)
        msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
        msg.header.frame_id = "camera"
        msg.width = target_w
        msg.height = target_h
        msg.distortion_model = intrinsics.distortion_model
        msg.d = intrinsics.D.flatten().tolist()
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def _publish_odom_nearest(self, timestamp: float, odom_samples: list[OdomSample]) -> None:
        """Publish the odom sample nearest to the given timestamp."""
        if not odom_samples:
            return
        idx = bisect.bisect_left(odom_samples, timestamp, key=lambda o: o.timestamp)
        if idx == 0:
            sample = odom_samples[0]
        elif idx == len(odom_samples):
            sample = odom_samples[-1]
        elif timestamp - odom_samples[idx - 1].timestamp <= odom_samples[idx].timestamp - timestamp:
            sample = odom_samples[idx - 1]
        else:
            sample = odom_samples[idx]
        msg = Odometry()
        msg.header.stamp.sec = int(sample.timestamp)
        msg.header.stamp.nanosec = int((sample.timestamp - int(sample.timestamp)) * 1e9)
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = float(sample.position[0])
        msg.pose.pose.position.y = float(sample.position[1])
        msg.pose.pose.position.z = float(sample.position[2])
        msg.pose.pose.orientation.x = float(sample.orientation[0])
        msg.pose.pose.orientation.y = float(sample.orientation[1])
        msg.pose.pose.orientation.z = float(sample.orientation[2])
        msg.pose.pose.orientation.w = float(sample.orientation[3])
        self._odom_pub.publish(msg)

    def run(self, streams: SensorStreams) -> None:
        """Process frames from any source. Blocks until source is exhausted or interrupted."""
        print(f"Destination: ({self._dest_lat}, {self._dest_lon}), lookahead: {self._lookahead_m}m")
        print("Connect Foxglove to ws://localhost:8765")

        for frame in streams.frames:
            self._process_frame(frame, streams.gps_fixes, streams.imu_samples, streams.odom_samples)

        print(f"\nDone. Processed {self._frame_num} frames.")

    def _ensure_gps_enu(self, gps_fixes: list[GpsFix]) -> None:
        """Convert any new GPS fixes to ENU (incremental)."""
        while len(self._gps_enu_cache) < len(gps_fixes):
            g = gps_fixes[len(self._gps_enu_cache)]
            enu = geodetic_to_enu(g.latitude, g.longitude, g.altitude)
            self._gps_enu_cache.append(np.array(enu))

    def _try_resolve_route(self, gps_fixes: list[GpsFix]) -> None:
        """Query Google Maps route on first GPS fix or when destination changes."""
        if self._route_resolved or not self._api_key or not gps_fixes:
            return
        self._route_resolved = True
        if self._start_override:
            start_lat, start_lon = self._start_override
            self._start_override = None
        else:
            start_lat, start_lon = gps_fixes[0].latitude, gps_fixes[0].longitude
        print(f"Querying Google Maps: ({start_lat:.6f}, {start_lon:.6f}) → ({self._dest_lat}, {self._dest_lon})")
        route_gps = _get_google_maps_route(start_lat, start_lon, self._dest_lat, self._dest_lon, self._api_key)
        if route_gps:
            route_xy = _route_to_enu(route_gps)
            self._route_enu = _resample_path(route_xy, step_size=0.5)
            print(f"Google Maps route: {len(self._route_enu)} points in ENU")

    def _process_frame(
        self,
        frame: "SyncedFrame",
        gps_fixes: list[GpsFix],
        imu_samples: list[ImuSample],
        odom_samples: list[OdomSample],
    ) -> None:
        t0 = time.perf_counter()

        n_imu = len(imu_samples)
        n_gps = len(gps_fixes)

        self._try_resolve_route(gps_fixes)
        self._ensure_gps_enu(gps_fixes)

        # Advance heading estimate from phone orientation up to this frame.
        while self._imu_idx < n_imu and imu_samples[self._imu_idx].timestamp <= frame.timestamp:
            s = imu_samples[self._imu_idx]
            phone_heading = _heading_from_phone_orientation(
                s.orientation, self._base_forward_axis_imu
            )
            if phone_heading is not None:
                self._latest_phone_heading_raw = phone_heading
                self._latest_phone_heading_timestamp = s.timestamp
                if self._heading_offset_enu_from_phone is not None:
                    self._latest_heading_enu = _wrap_angle(
                        phone_heading + self._heading_offset_enu_from_phone
                    )
            self._imu_idx += 1

        # Advance GPS position and course heading up to this frame.
        while self._gps_idx < n_gps and gps_fixes[self._gps_idx].timestamp <= frame.timestamp:
            g = gps_fixes[self._gps_idx]
            enu = self._gps_enu_cache[self._gps_idx]
            self._latest_gps_enu = enu.copy()

            if self._prev_gps_enu_for_heading is not None:
                de = float(enu[0] - self._prev_gps_enu_for_heading[0])
                dn = float(enu[1] - self._prev_gps_enu_for_heading[1])
                dist = math.hypot(de, dn)
                if dist >= _GPS_HEADING_MIN_DIST_M:
                    self._latest_course_heading = math.atan2(de, dn)

                    if (
                        self._latest_phone_heading_raw is not None
                        and self._latest_phone_heading_timestamp is not None
                        and abs(g.timestamp - self._latest_phone_heading_timestamp) <= _MAX_IMU_GPS_TIME_SKEW_S
                    ):
                        target_offset = _wrap_angle(
                            self._latest_course_heading - self._latest_phone_heading_raw
                        )
                        if self._heading_offset_enu_from_phone is None:
                            self._heading_offset_enu_from_phone = target_offset
                            print(
                                f"  Heading calibrated: phone→ENU offset="
                                f"{math.degrees(target_offset):.1f}°"
                            )
                        else:
                            # Slow offset adaptation for magnetometer/local disturbances.
                            delta = _wrap_angle(target_offset - self._heading_offset_enu_from_phone)
                            self._heading_offset_enu_from_phone = _wrap_angle(
                                self._heading_offset_enu_from_phone + 0.1 * delta
                            )
                        self._latest_heading_enu = _wrap_angle(
                            self._latest_phone_heading_raw + self._heading_offset_enu_from_phone
                        )

            if self._republish_sensors:
                gps_msg = NavSatFix()
                gps_msg.header = _header(g.timestamp, "gps")
                gps_msg.latitude = g.latitude
                gps_msg.longitude = g.longitude
                gps_msg.altitude = g.altitude
                self._gps_pub.publish(gps_msg)

            self._prev_gps_enu_for_heading = enu.copy()
            self._gps_idx += 1

        # Localization state used by routing/planning.
        if self._latest_gps_enu is not None:
            ego_enu = self._latest_gps_enu.copy()
        elif self._gps_enu_cache:
            ego_enu = self._gps_enu_cache[0].copy()
        else:
            ego_enu = np.zeros(3, dtype=np.float64)

        heading_enu = (
            self._latest_heading_enu
            if self._latest_heading_enu is not None
            else self._latest_course_heading
        )
        if heading_enu is None:
            heading_enu = 0.0

        # Compute goal from route
        if self._route_enu is not None and self._latest_gps_enu is not None:
            goal_result = _find_lookahead_goal(self._route_enu, ego_enu, self._lookahead_m)
            if goal_result:
                goal_east, goal_north = goal_result
                goal_x, goal_y = _enu_to_body_xy(
                    goal_east - float(ego_enu[0]),
                    goal_north - float(ego_enu[1]),
                    heading_enu,
                )
            else:
                goal_x, goal_y = self._lookahead_m, 0.0
        else:
            goal_x, goal_y = self._lookahead_m, 0.0

        half = GRID_SIZE_M / 2.0 - 1.0
        goal_x, goal_y = _rotate_xy(goal_x, goal_y, self._local_frame_yaw_offset_rad)
        goal_x = np.clip(goal_x, -half, half)
        goal_y = np.clip(goal_y, -half, half)

        if self._republish_sensors and self._frame_num % 2 == 0:
            # Resize RGB and depth to common resolution for SLAM
            target_size = (512, 384)
            rgb_resized = resize_image(frame.image, target_size, cv2.INTER_AREA)
            depth_resized = resize_depth(frame.depth, target_size)

            cam_msg = CompressedImage()
            cam_msg.header = _header(frame.timestamp, "camera")
            cam_msg.format = "jpeg"
            _, cam_buf = cv2.imencode(".jpg", rgb_resized, [cv2.IMWRITE_JPEG_QUALITY, 80])
            cam_msg.data = cam_buf.tobytes()
            self._camera_pub.publish(cam_msg)

            self._camera_info_pub.publish(
                self._make_camera_info(frame.intrinsics, frame.timestamp, target_size)
            )

            depth_msg = Image()
            depth_msg.header = _header(frame.timestamp, "camera")
            depth_msg.height, depth_msg.width = target_size[1], target_size[0]
            depth_msg.encoding = "32FC1"
            depth_msg.is_bigendian = False
            depth_msg.step = target_size[0] * 4
            depth_msg.data = depth_resized.tobytes()
            self._depth_pub.publish(depth_msg)

            self._publish_odom_nearest(frame.timestamp, odom_samples)

            if frame.orientation is not None:
                imu_msg = Imu()
                imu_msg.header = _header(frame.timestamp, "imu")
                imu_msg.orientation.x = float(frame.orientation[0])
                imu_msg.orientation.y = float(frame.orientation[1])
                imu_msg.orientation.z = float(frame.orientation[2])
                imu_msg.orientation.w = float(frame.orientation[3])
                self._imu_pub.publish(imu_msg)

        # Publish Google Maps route in ego-relative coords
        if self._route_enu is not None:
            if self._latest_gps_enu is not None:
                route_delta_e = self._route_enu[:, 0] - float(ego_enu[0])
                route_delta_n = self._route_enu[:, 1] - float(ego_enu[1])
                sin_h = math.sin(heading_enu)
                cos_h = math.cos(heading_enu)
                route_ego = np.empty_like(self._route_enu)
                route_ego[:, 0] = route_delta_e * sin_h + route_delta_n * cos_h
                route_ego[:, 1] = -route_delta_e * cos_h + route_delta_n * sin_h
            else:
                route_ego = self._route_enu - self._route_enu[0]
            route_ego = _rotate_xy_path(route_ego, self._local_frame_yaw_offset_rad)
            local_frame_id = (
                "camera"
                if self._republish_sensors
                else (frame.intrinsics.frame_id or "camera")
            )
            self._gmaps_path_pub.publish(
                _make_route_path(route_ego, frame.timestamp, local_frame_id)
            )

        # Perception
        result = self._pipeline.process_frame(frame)
        drivable_pts = _rotate_xy_points(result.drivable_pts, self._local_frame_yaw_offset_rad)
        lane_pts = _rotate_xy_points(result.lane_pts, self._local_frame_yaw_offset_rad)
        det_pts = _rotate_xy_points(result.det_pts, self._local_frame_yaw_offset_rad)
        local_frame_id = (
            "camera"
            if self._republish_sensors
            else (frame.intrinsics.frame_id or "camera")
        )

        if drivable_pts is not None:
            self._pc_drivable_pub.publish(
                _make_pointcloud2(
                    drivable_pts, 0.0, 1.0, 0.0, frame.timestamp, local_frame_id
                )
            )
        if lane_pts is not None:
            self._pc_lanes_pub.publish(
                _make_pointcloud2(
                    lane_pts, 1.0, 0.0, 0.0, frame.timestamp, local_frame_id
                )
            )
        if det_pts is not None:
            self._pc_det_pub.publish(
                _make_pointcloud2(
                    det_pts, 1.0, 1.0, 0.0, frame.timestamp, local_frame_id
                )
            )

        # Costmap
        grid = build_costmap(drivable_pts, lane_pts, det_pts)
        self._costmap_pub.publish(
            _costmap_to_occupancy_grid(grid, frame.timestamp, local_frame_id)
        )

        # Markers
        self._goal_marker_pub.publish(
            _make_goal_marker(goal_x, goal_y, frame.timestamp, local_frame_id)
        )
        self._ego_marker_pub.publish(_make_ego_marker(frame.timestamp, local_frame_id))

        # Collect previous planning result (non-blocking)
        if self._plan_future is not None and self._plan_future.done():
            try:
                path = self._plan_future.result()
                if path and path.poses:
                    path.header = _header(frame.timestamp)
                    self._path_pub.publish(path)
                    self._latest_path = path
            except Exception as e:
                if self._frame_num % 10 == 0:
                    print(f"\nPlanner failed: {e}")
            self._plan_future = None

        # Submit new Nav2 planning in background
        if self._plan_future is None:
            goal_pose = copy.deepcopy(self._goal_pose)
            goal_pose.header.stamp = _stamp_from_epoch(frame.timestamp)
            goal_pose.pose.position.x = float(goal_x)
            goal_pose.pose.position.y = float(goal_y)
            start_pose = copy.deepcopy(self._start_pose)
            start_pose.header.stamp = goal_pose.header.stamp
            self._plan_future = self._planning_executor.submit(
                self._navigator.getPath, start_pose, goal_pose, planner_id="GridBased",
            )

        # Throttled costmap image (expensive PNG encode) — every 3rd frame
        if self._frame_num % 3 == 0:
            self._costmap_img_pub.publish(
                _path_to_costmap_image(grid, self._latest_path, goal_x, goal_y)
            )

        dt = time.perf_counter() - t0
        self._frame_num += 1
        if self._frame_num % 5 == 0:
            n_poses = len(self._latest_path.poses) if self._latest_path and self._latest_path.poses else 0
            print(
                f"\rFrame {self._frame_num}: path={n_poses} poses, "
                f"{len(result.detections)} det, {1/dt:.1f} FPS, "
                f"GPS=({ego_enu[0]:.1f},{ego_enu[1]:.1f}) heading={math.degrees(heading_enu):.0f}° "
                f"goal=({goal_x:.1f},{goal_y:.1f})",
                end="", flush=True,
            )

    def shutdown(self) -> None:
        """Clean up Nav2 and ROS2 resources."""
        self._status_timer.cancel()
        self._planning_executor.shutdown(wait=False)
        self._pipeline.shutdown()
        self._navigator.destroyNode()


# ── CLI entry point ───────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Nav2 path planning with GPS + phone heading + perception (bag replay or live)"
    )
    parser.add_argument("--bag", default=None,
                        help="Path to .mcap bag file (omit for live mode)")
    parser.add_argument("--topic-prefix", default="/iphone",
                        help="ROS2 topic prefix for live mode")
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
    parser.add_argument("--lookahead", type=float, default=8.0,
                        help="Lookahead distance on Google Maps route (meters)")
    parser.add_argument(
        "--camera-mount-mode",
        choices=["handheld", "urdf", "identity"],
        default="handheld",
        help="Camera->base transform mode: handheld, urdf (strict), or identity (no frame delta)",
    )
    parser.add_argument(
        "--orientation-mode",
        choices=["gravity", "full", "none"],
        default="gravity",
        help="IMU compensation for projection: gravity (roll/pitch), full, or none",
    )
    parser.add_argument(
        "--local-frame-yaw-offset-rad",
        type=float,
        default=_DEFAULT_LOCAL_FRAME_YAW_OFFSET_RAD,
        help="Additional yaw rotation applied to local BEV frame (radians)",
    )
    args = parser.parse_args()

    config = PipelineConfig.from_yaml(args.config) if args.config else PipelineConfig(
        device=args.device,
        detection_confidence=args.conf,
        subsample_drivable=args.subsample_drivable,
        subsample_lane=args.subsample_lane,
        subsample_bbox=args.subsample_bbox,
    )

    live_mode = args.bag is None
    mode_label = "LIVE" if live_mode else f"BAG: {args.bag}"
    print(f"Nav2 Planner — {mode_label}")

    rclpy.init()
    node = rclpy.create_node("nav2_planner")
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    planner = Nav2Planner(
        node=node,
        config=config,
        dest_lat=args.dest_lat,
        dest_lon=args.dest_lon,
        lookahead_m=args.lookahead,
        republish_sensors=not live_mode,
        camera_mount_mode=args.camera_mount_mode,
        orientation_mode=args.orientation_mode,
        local_frame_yaw_offset_rad=args.local_frame_yaw_offset_rad,
    )

    if live_mode:
        source = LiveSource(node, topic_prefix=args.topic_prefix)
    else:
        source = BagSource(args.bag, playback_rate=args.playback_rate, max_frames=args.max_frames)

    streams = source.open()

    try:
        planner.run(streams)
        if not live_mode:
            print("Press Ctrl+C to stop.")
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        if live_mode:
            source.stop()
        planner.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
