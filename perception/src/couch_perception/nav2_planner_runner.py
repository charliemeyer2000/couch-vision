"""Nav2 path planning on MCAP bag data with foxglove_bridge visualization.

Processes bag through YOLOv8+YOLOP perception, builds a costmap, publishes it
as a ROS2 OccupancyGrid for Nav2, uses the Simple Commander API to plan a path
to a waypoint ahead, and publishes all results as ROS2 topics.

Visualization is handled by foxglove_bridge (launched separately), which
forwards all ROS2 topics to Foxglove Studio over WebSocket.

Requires Nav2 planner_server + foxglove_bridge to be running (see launch file).
"""

import argparse
import math
import struct
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage, Image, Imu, PointCloud2, PointField
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from builtin_interfaces.msg import Time as RosTime, Duration as RosDuration

from nav2_simple_commander.robot_navigator import BasicNavigator

from couch_perception.bag_reader import read_synced_frames
from couch_perception.camera_model import make_camera_model, CameraModel
from couch_perception.yolov8_detector import YOLOv8Detector
from couch_perception.yolop_detector import YOLOPDetector
from couch_perception.costmap_runner import (
    build_costmap,
    GRID_CELLS,
    GRID_ORIGIN,
    GRID_RESOLUTION,
    GRID_SIZE_M,
)
from couch_perception.costmap_visualizer import costmap_to_upscaled_image
from couch_perception.bev_projection_runner import (
    _apply_imu_rotation,
    _extract_mask_pixels,
    _extract_bbox_pixels,
    _extract_bbox_pixels_grouped,
    _build_depth_camera_model,
)


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

    Internal grid: grid[r,c] — row 0 = max forward (+x), col 0 = min lateral (min y).
    OccupancyGrid: row-major, row 0 = min y, col 0 = min x.
    Transform: flip rows (so row 0 = min x) then transpose (swap row/col axes).
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
    og = grid[::-1].T
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


# ── Costmap overlay image ─────────────────────────────────────────────────

def _path_to_costmap_image(
    grid: np.ndarray, path: Path | None, goal_x: float, goal_y: float
) -> CompressedImage:
    """Draw the planned path and waypoint on the costmap color image, return as CompressedImage."""
    img = costmap_to_upscaled_image(grid, scale=4)
    scale = 4

    if path and path.poses:
        for ps in path.poses:
            px = ps.pose.position.x
            py = ps.pose.position.y
            col = int((py - GRID_ORIGIN) / GRID_RESOLUTION) * scale
            row = int((GRID_CELLS - 1 - (px - GRID_ORIGIN) / GRID_RESOLUTION)) * scale
            if 0 <= row < img.shape[0] and 0 <= col < img.shape[1]:
                cv2.circle(img, (col, row), 3, (255, 180, 0), -1)

    # Goal
    gc = int((goal_y - GRID_ORIGIN) / GRID_RESOLUTION) * scale
    gr = int((GRID_CELLS - 1 - (goal_x - GRID_ORIGIN) / GRID_RESOLUTION)) * scale
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
    device: str | None = None,
    conf: float = 0.3,
    max_frames: int | None = None,
    subsample_drivable: int = 4,
    subsample_lane: int = 2,
    subsample_bbox: int = 8,
    playback_rate: float = 1.0,
    goal_x: float = 15.0,
    goal_y: float = 0.0,
) -> None:
    """Process bag, build costmaps, plan paths via Nav2, publish all as ROS2 topics."""

    rclpy.init()
    node = rclpy.create_node("nav2_planner_runner")

    # QoS profiles
    qos_reliable = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    qos_best_effort = QoSProfile(depth=5)

    # Publishers — all ROS2 topics
    costmap_pub = node.create_publisher(OccupancyGrid, "/costmap/occupancy_grid", qos_reliable)
    path_pub = node.create_publisher(Path, "/nav/planned_path", qos_best_effort)
    goal_marker_pub = node.create_publisher(Marker, "/nav/goal_marker", qos_best_effort)
    ego_marker_pub = node.create_publisher(Marker, "/nav/ego_marker", qos_best_effort)
    costmap_img_pub = node.create_publisher(CompressedImage, "/nav/costmap_image/compressed", qos_best_effort)
    pc_drivable_pub = node.create_publisher(PointCloud2, "/perception/pointcloud/drivable", qos_best_effort)
    pc_lanes_pub = node.create_publisher(PointCloud2, "/perception/pointcloud/lanes", qos_best_effort)
    pc_det_pub = node.create_publisher(PointCloud2, "/perception/pointcloud/detections", qos_best_effort)
    camera_pub = node.create_publisher(CompressedImage, "/camera/image/compressed", qos_best_effort)
    depth_pub = node.create_publisher(Image, "/camera/depth/image", qos_best_effort)
    imu_pub = node.create_publisher(Imu, "/imu", qos_best_effort)

    # Spin in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Wait for Nav2
    print("Waiting for Nav2 planner server...")
    navigator = BasicNavigator()
    navigator._waitForNodeToActivate("planner_server")
    print("Nav2 planner is active!")

    # Load perception
    yolo = YOLOv8Detector(conf_threshold=conf, device=device)
    print(f"YOLOv8 loaded (device={yolo.device})")
    print("Loading YOLOP...")
    yolop = YOLOPDetector(device=device)

    # Poses
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    goal_pose.pose.orientation.w = 1.0

    start_pose = PoseStamped()
    start_pose.header.frame_id = "map"
    start_pose.pose.orientation.w = 1.0

    print(f"Goal waypoint: ({goal_x}, {goal_y}) — {math.hypot(goal_x, goal_y):.1f}m ahead")
    print("Topics are published on ROS2. Connect Foxglove to foxglove_bridge (ws://localhost:8765).")

    frames = read_synced_frames(bag_path)
    cam_model: CameraModel | None = None
    depth_cam_model: CameraModel | None = None

    frame_num = 0
    prev_wall_time: float | None = None
    prev_bag_time: float | None = None

    for frame in frames:
        if max_frames and frame_num >= max_frames:
            break

        if cam_model is None:
            cam_model = make_camera_model(frame.intrinsics)
            depth_cam_model = _build_depth_camera_model(
                cam_model, frame.depth.shape[:2]
            )

        # Pace playback
        if prev_bag_time is not None and playback_rate > 0:
            bag_dt = frame.timestamp - prev_bag_time
            wall_dt = time.monotonic() - prev_wall_time
            sleep_time = (bag_dt / playback_rate) - wall_dt
            if sleep_time > 0:
                time.sleep(sleep_time)
        prev_bag_time = frame.timestamp
        prev_wall_time = time.monotonic()

        t0 = time.perf_counter()

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

        # Run perception
        detections = yolo.detect(frame.image)
        yolop_result = yolop.detect(frame.image)

        # Project drivable area
        drivable_pts = None
        if yolop_result and yolop_result.drivable_mask is not None:
            pixels, depths = _extract_mask_pixels(
                yolop_result.drivable_mask, frame.depth, subsample=subsample_drivable
            )
            if len(pixels) > 0:
                pts_3d = depth_cam_model.project_pixels_to_3d(pixels, depths)
                drivable_pts = _apply_imu_rotation(pts_3d, frame.orientation)
                pc_drivable_pub.publish(
                    _make_pointcloud2(drivable_pts, 0.0, 1.0, 0.0, frame.timestamp)
                )

        # Project lanes
        lane_pts = None
        if yolop_result and yolop_result.lane_mask is not None:
            pixels, depths = _extract_mask_pixels(
                yolop_result.lane_mask, frame.depth, subsample=subsample_lane
            )
            if len(pixels) > 0:
                pts_3d = depth_cam_model.project_pixels_to_3d(pixels, depths)
                lane_pts = _apply_imu_rotation(pts_3d, frame.orientation)
                pc_lanes_pub.publish(
                    _make_pointcloud2(lane_pts, 1.0, 0.0, 0.0, frame.timestamp)
                )

        # Project detections (per-detection groups for filled costmap obstacles)
        det_pts = None
        det_groups = None
        if detections:
            groups = _extract_bbox_pixels_grouped(
                detections, frame.depth, frame.image.shape[:2], subsample=subsample_bbox
            )
            all_world_groups = []
            all_world_pts = []
            for pixels, depths in groups:
                if len(pixels) == 0:
                    continue
                pts_3d = depth_cam_model.project_pixels_to_3d(pixels, depths)
                world_pts = _apply_imu_rotation(pts_3d, frame.orientation)
                all_world_groups.append(world_pts)
                all_world_pts.append(world_pts)
            if all_world_pts:
                det_pts = np.vstack(all_world_pts)
                det_groups = all_world_groups
                pc_det_pub.publish(
                    _make_pointcloud2(det_pts, 1.0, 1.0, 0.0, frame.timestamp)
                )

        # Build and publish costmap
        grid = build_costmap(drivable_pts, lane_pts, det_pts, det_groups=det_groups)
        costmap_pub.publish(_costmap_to_occupancy_grid(grid, frame.timestamp))

        # Publish markers
        goal_marker_pub.publish(_make_goal_marker(goal_x, goal_y, frame.timestamp))
        ego_marker_pub.publish(_make_ego_marker(frame.timestamp))

        # Plan path via Nav2
        goal_pose.header.stamp = _stamp_from_epoch(frame.timestamp)
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

        # Publish costmap overlay image (with or without path)
        costmap_img_pub.publish(
            _path_to_costmap_image(grid, path, goal_x, goal_y)
        )

        dt = time.perf_counter() - t0
        frame_num += 1
        if frame_num % 5 == 0:
            n_det = len(detections)
            n_poses = len(path.poses) if path and path.poses else 0
            print(
                f"\rFrame {frame_num}: path={n_poses} poses, "
                f"{n_det} det, {1/dt:.1f} FPS",
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
        description="Nav2 path planning on MCAP bag data with ROS2 + foxglove_bridge"
    )
    parser.add_argument("--bag", required=True, help="Path to .mcap bag file")
    parser.add_argument("--device", default=None, help="Torch device")
    parser.add_argument("--conf", type=float, default=0.3)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--subsample-drivable", type=int, default=4)
    parser.add_argument("--subsample-lane", type=int, default=2)
    parser.add_argument("--subsample-bbox", type=int, default=8)
    parser.add_argument("--playback-rate", type=float, default=1.0)
    parser.add_argument("--goal-x", type=float, default=15.0,
                        help="Goal X position in meters (forward)")
    parser.add_argument("--goal-y", type=float, default=0.0,
                        help="Goal Y position in meters (lateral)")
    args = parser.parse_args()

    process_bag(
        bag_path=args.bag,
        device=args.device,
        conf=args.conf,
        max_frames=args.max_frames,
        subsample_drivable=args.subsample_drivable,
        subsample_lane=args.subsample_lane,
        subsample_bbox=args.subsample_bbox,
        playback_rate=args.playback_rate,
        goal_x=args.goal_x,
        goal_y=args.goal_y,
    )


if __name__ == "__main__":
    main()
