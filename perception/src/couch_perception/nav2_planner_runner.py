"""Nav2 path planning on MCAP bag data with Foxglove visualization.

Processes bag through YOLOv8+YOLOP perception, builds a costmap, publishes it
as a ROS2 OccupancyGrid for Nav2, uses the Simple Commander API to plan a path
to a waypoint 5m ahead, then publishes the planned path to Foxglove.

Requires Nav2 planner_server + global_costmap to be running (see launch file).
"""

import argparse
import datetime
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
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion as RosQuaternion
from std_msgs.msg import Header
from builtin_interfaces.msg import Time as RosTime

import foxglove
from foxglove.schemas import (
    CompressedImage,
    Grid,
    LinePrimitive,
    LinePrimitiveLineType,
    PackedElementField,
    PackedElementFieldNumericType,
    PointCloud,
    Pose as FgPose,
    Quaternion as FgQuaternion,
    Color,
    CubePrimitive,
    Duration,
    Point3,
    SceneEntity,
    SceneUpdate,
    Timestamp,
    Vector2,
    Vector3,
)
from foxglove.websocket import ServerListener, Client, ChannelView

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
    _make_foxglove_grid,
)
from couch_perception.costmap_visualizer import costmap_to_upscaled_image
from couch_perception.bev_projection_runner import (
    _apply_imu_rotation,
    _extract_mask_pixels,
    _extract_bbox_pixels,
    _build_depth_camera_model,
    _pack_points_rgba_vectorized,
    _make_pointcloud,
)


class _FgListener(ServerListener):
    def __init__(self) -> None:
        self.subscribers: dict[int, set[str]] = {}

    def on_subscribe(self, client: Client, channel: ChannelView) -> None:
        self.subscribers.setdefault(client.id, set()).add(channel.topic)

    def on_unsubscribe(self, client: Client, channel: ChannelView) -> None:
        if client.id in self.subscribers:
            self.subscribers[client.id].discard(channel.topic)
            if not self.subscribers[client.id]:
                del self.subscribers[client.id]


def _stamp_from_epoch(t: float) -> RosTime:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return RosTime(sec=sec, nanosec=nanosec)


def _costmap_to_occupancy_grid(grid: np.ndarray, timestamp: float) -> OccupancyGrid:
    """Convert our costmap array to a ROS2 OccupancyGrid message.

    Nav2 OccupancyGrid uses int8 values: 0=free, 100=lethal, -1=unknown.
    Our grid already uses these conventions.
    """
    msg = OccupancyGrid()
    msg.header = Header()
    msg.header.stamp = _stamp_from_epoch(timestamp)
    msg.header.frame_id = "map"

    msg.info.resolution = GRID_RESOLUTION
    msg.info.width = GRID_CELLS
    msg.info.height = GRID_CELLS
    msg.info.origin.position.x = GRID_ORIGIN
    msg.info.origin.position.y = GRID_ORIGIN
    msg.info.origin.position.z = 0.0
    msg.info.origin.orientation.w = 1.0

    # OccupancyGrid.data is int8[] — flatten row-major
    msg.data = grid.flatten().tolist()
    return msg


def _path_to_foxglove_scene(
    path: Path, timestamp: float
) -> SceneUpdate:
    """Convert a nav_msgs/Path to a Foxglove SceneUpdate with a line strip."""
    dt = datetime.datetime.fromtimestamp(timestamp, tz=datetime.timezone.utc)
    ts = Timestamp.from_datetime(dt)

    points = []
    for pose_stamped in path.poses:
        p = pose_stamped.pose.position
        points.append(Point3(x=p.x, y=p.y, z=p.z + 0.05))

    line = LinePrimitive(
        type=LinePrimitiveLineType.LineStrip,
        pose=FgPose(
            position=Vector3(x=0.0, y=0.0, z=0.0),
            orientation=FgQuaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        thickness=0.08,
        scale_invariant=False,
        points=points,
        color=Color(r=0.0, g=0.5, b=1.0, a=1.0),
    )

    entity = SceneEntity(
        timestamp=ts,
        frame_id="map",
        id="planned_path",
        lifetime=Duration(sec=0, nsec=500_000_000),
        lines=[line],
    )

    return SceneUpdate(entities=[entity])


def _waypoint_marker(
    x: float, y: float, timestamp: float
) -> SceneUpdate:
    """Create a Foxglove marker for the goal waypoint."""
    dt = datetime.datetime.fromtimestamp(timestamp, tz=datetime.timezone.utc)
    ts = Timestamp.from_datetime(dt)

    entity = SceneEntity(
        timestamp=ts,
        frame_id="map",
        id="goal_waypoint",
        lifetime=Duration(sec=0, nsec=500_000_000),
        cubes=[
            CubePrimitive(
                pose=FgPose(
                    position=Vector3(x=x, y=y, z=0.1),
                    orientation=FgQuaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                size=Vector3(x=0.3, y=0.3, z=0.3),
                color=Color(r=1.0, g=0.2, b=0.2, a=0.9),
            ),
        ],
    )
    return SceneUpdate(entities=[entity])


def _path_to_foxglove_image(
    grid: np.ndarray, path: Path, goal_x: float, goal_y: float
) -> np.ndarray:
    """Draw the planned path and waypoint on top of the costmap color image."""
    img = costmap_to_upscaled_image(grid, scale=4)
    scale = 4

    # Draw path
    for ps in path.poses:
        px = ps.pose.position.x
        py = ps.pose.position.y
        col = int((py - GRID_ORIGIN) / GRID_RESOLUTION) * scale
        row = int((GRID_CELLS - 1 - (px - GRID_ORIGIN) / GRID_RESOLUTION)) * scale
        if 0 <= row < img.shape[0] and 0 <= col < img.shape[1]:
            cv2.circle(img, (col, row), 3, (255, 180, 0), -1)  # cyan path

    # Draw goal
    gc = int((goal_y - GRID_ORIGIN) / GRID_RESOLUTION) * scale
    gr = int((GRID_CELLS - 1 - (goal_x - GRID_ORIGIN) / GRID_RESOLUTION)) * scale
    if 0 <= gr < img.shape[0] and 0 <= gc < img.shape[1]:
        cv2.circle(img, (gc, gr), 8, (0, 0, 255), -1)  # red goal
        cv2.putText(img, "GOAL", (gc + 10, gr + 5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (0, 0, 255), 1)

    # Draw ego position (center)
    center = GRID_CELLS * scale // 2
    cv2.drawMarker(img, (center, center), (255, 255, 255),
                   cv2.MARKER_CROSS, 12, 2)

    return img


def _make_compressed_image(
    image: np.ndarray, timestamp: float, frame_id: str = "costmap"
) -> CompressedImage:
    _, buf = cv2.imencode(".png", image)
    dt = datetime.datetime.fromtimestamp(timestamp, tz=datetime.timezone.utc)
    return CompressedImage(
        timestamp=Timestamp.from_datetime(dt),
        frame_id=frame_id,
        data=buf.tobytes(),
        format="png",
    )


def process_bag(
    bag_path: str,
    device: str | None = None,
    conf: float = 0.3,
    max_frames: int | None = None,
    subsample_drivable: int = 4,
    subsample_lane: int = 2,
    subsample_bbox: int = 8,
    playback_rate: float = 1.0,
    goal_x: float = 5.0,
    goal_y: float = 0.0,
) -> None:
    """Process bag, build costmaps, plan paths via Nav2, publish to Foxglove."""

    # Initialize ROS2 and Nav2
    rclpy.init()
    node = rclpy.create_node("costmap_publisher")

    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    costmap_pub = node.create_publisher(OccupancyGrid, "/costmap/occupancy_grid", qos)

    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Wait for Nav2 planner to come up
    print("Waiting for Nav2 planner server...")
    navigator = BasicNavigator()
    # Wait for the compute_path_to_pose action server directly.
    # We don't use AMCL or bt_navigator, just the planner.
    navigator._waitForNodeToActivate("planner_server")
    print("Nav2 planner is active!")

    # Load perception models
    yolo = YOLOv8Detector(conf_threshold=conf, device=device)
    print(f"YOLOv8 loaded (device={yolo.device})")
    print("Loading YOLOP...")
    yolop = YOLOPDetector(device=device)

    # Start Foxglove
    fg_listener = _FgListener()
    fg_server = foxglove.start_server(server_listener=fg_listener)
    print("Foxglove server started on ws://localhost:8765")

    # Build goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    start_pose = PoseStamped()
    start_pose.header.frame_id = "map"
    start_pose.pose.position.x = 0.0
    start_pose.pose.position.y = 0.0
    start_pose.pose.position.z = 0.0
    start_pose.pose.orientation.w = 1.0

    print(f"Goal waypoint: ({goal_x}, {goal_y}) — {math.hypot(goal_x, goal_y):.1f}m ahead")

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

        # Run perception
        detections = yolo.detect(frame.image)
        yolop_result = yolop.detect(frame.image)

        # Project points
        drivable_pts = None
        if yolop_result and yolop_result.drivable_mask is not None:
            pixels, depths = _extract_mask_pixels(
                yolop_result.drivable_mask, frame.depth, subsample=subsample_drivable
            )
            if len(pixels) > 0:
                pts_3d = depth_cam_model.project_pixels_to_3d(pixels, depths)
                drivable_pts = _apply_imu_rotation(pts_3d, frame.orientation)
                data = _pack_points_rgba_vectorized(drivable_pts, 0.0, 1.0, 0.0)
                foxglove.log("/perception/pointcloud/drivable",
                             _make_pointcloud(data, frame.timestamp))

        lane_pts = None
        if yolop_result and yolop_result.lane_mask is not None:
            pixels, depths = _extract_mask_pixels(
                yolop_result.lane_mask, frame.depth, subsample=subsample_lane
            )
            if len(pixels) > 0:
                pts_3d = depth_cam_model.project_pixels_to_3d(pixels, depths)
                lane_pts = _apply_imu_rotation(pts_3d, frame.orientation)
                data = _pack_points_rgba_vectorized(lane_pts, 1.0, 0.0, 0.0)
                foxglove.log("/perception/pointcloud/lanes",
                             _make_pointcloud(data, frame.timestamp))

        det_pts = None
        if detections:
            pixels, depths = _extract_bbox_pixels(
                detections, frame.depth, frame.image.shape[:2], subsample=subsample_bbox
            )
            if len(pixels) > 0:
                pts_3d = depth_cam_model.project_pixels_to_3d(pixels, depths)
                det_pts = _apply_imu_rotation(pts_3d, frame.orientation)
                data = _pack_points_rgba_vectorized(det_pts, 1.0, 1.0, 0.0)
                foxglove.log("/perception/pointcloud/detections",
                             _make_pointcloud(data, frame.timestamp))

        # Build costmap and publish to ROS2
        grid = build_costmap(drivable_pts, lane_pts, det_pts)
        ros_msg = _costmap_to_occupancy_grid(grid, frame.timestamp)
        costmap_pub.publish(ros_msg)

        # Publish costmap to Foxglove
        foxglove.log("/costmap/occupancy_grid",
                     _make_foxglove_grid(grid, frame.timestamp))

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
            # Publish path as Foxglove scene (3D line)
            foxglove.log("/nav/planned_path",
                         _path_to_foxglove_scene(path, frame.timestamp))

            # Publish path overlay on costmap image
            path_img = _path_to_foxglove_image(grid, path, goal_x, goal_y)
            foxglove.log("/nav/costmap_with_path",
                         _make_compressed_image(path_img, frame.timestamp))

            # Publish waypoint marker
            foxglove.log("/nav/goal_marker",
                         _waypoint_marker(goal_x, goal_y, frame.timestamp))

            if frame_num % 5 == 0:
                print(
                    f"\rFrame {frame_num}: path={len(path.poses)} poses, "
                    f"{len(detections)} det, {1/(time.perf_counter()-t0):.1f} FPS",
                    end="", flush=True,
                )
        else:
            # Still publish costmap image without path
            img = costmap_to_upscaled_image(grid, scale=4)
            foxglove.log("/nav/costmap_with_path",
                         _make_compressed_image(img, frame.timestamp))
            if frame_num % 5 == 0:
                print(
                    f"\rFrame {frame_num}: NO PATH, {len(detections)} det, "
                    f"{1/(time.perf_counter()-t0):.1f} FPS",
                    end="", flush=True,
                )

        frame_num += 1

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
        fg_server.stop()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Nav2 path planning on MCAP bag data with Foxglove visualization"
    )
    parser.add_argument("--bag", required=True, help="Path to .mcap bag file")
    parser.add_argument("--device", default=None, help="Torch device")
    parser.add_argument("--conf", type=float, default=0.3)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--subsample-drivable", type=int, default=4)
    parser.add_argument("--subsample-lane", type=int, default=2)
    parser.add_argument("--subsample-bbox", type=int, default=8)
    parser.add_argument("--playback-rate", type=float, default=1.0)
    parser.add_argument("--goal-x", type=float, default=5.0,
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
