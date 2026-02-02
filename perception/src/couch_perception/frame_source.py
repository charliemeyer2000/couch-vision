"""Frame sources: abstractions for reading sensor data from bags or live ROS2 topics.

Provides BagSource (offline replay) and LiveSource (live ROS2 subscriptions),
both yielding SensorStreams with the same interface.
"""

from __future__ import annotations

import queue
import threading
import time
from collections.abc import Iterator
from dataclasses import dataclass

import cv2
import numpy as np

from couch_perception.bag_reader import (
    CameraIntrinsics,
    GpsFix,
    ImuSample,
    OdomSample,
    SyncedFrame,
    read_gps_imu_and_odom,
    read_synced_frames,
)


@dataclass
class SensorStreams:
    """All sensor streams from a data source."""

    frames: Iterator[SyncedFrame]
    gps_fixes: list[GpsFix]
    imu_samples: list[ImuSample]
    odom_samples: list[OdomSample]


class BagSource:
    """Read frames from an MCAP bag with optional playback pacing.

    Args:
        bag_path: Path to the .mcap bag file.
        playback_rate: Speed multiplier (1.0 = real-time, 0 = as fast as possible).
        max_frames: Stop after this many frames (None = all).
    """

    def __init__(
        self,
        bag_path: str,
        playback_rate: float = 1.0,
        max_frames: int | None = None,
    ) -> None:
        self.bag_path = bag_path
        self.playback_rate = playback_rate
        self.max_frames = max_frames

    def open(self) -> SensorStreams:
        """Open the bag and return all sensor streams."""
        gps_fixes, imu_samples, odom_samples = read_gps_imu_and_odom(self.bag_path)
        frames = self._paced_frames()
        return SensorStreams(
            frames=frames,
            gps_fixes=gps_fixes,
            imu_samples=imu_samples,
            odom_samples=odom_samples,
        )

    def _paced_frames(self) -> Iterator[SyncedFrame]:
        """Yield frames with playback pacing applied."""
        prev_bag_time: float | None = None
        prev_wall_time: float | None = None
        count = 0

        for frame in read_synced_frames(self.bag_path):
            if self.max_frames is not None and count >= self.max_frames:
                return

            if prev_bag_time is not None and self.playback_rate > 0:
                bag_dt = frame.timestamp - prev_bag_time
                wall_dt = time.monotonic() - prev_wall_time
                sleep_time = (bag_dt / self.playback_rate) - wall_dt
                if sleep_time > 0:
                    time.sleep(sleep_time)

            prev_bag_time = frame.timestamp
            prev_wall_time = time.monotonic()
            count += 1
            yield frame


class LiveSource:
    """Subscribe to live ROS2 topics and yield SensorStreams.

    Pairs compressed images with depth maps using approximate time synchronization.
    GPS and IMU are buffered into growing lists that the consumer reads by index.

    Args:
        node: An active rclpy Node (must be spinning in a background thread).
        topic_prefix: ROS2 topic prefix (e.g. "/iphone" or "/iphone_charlie").
        image_suffix: Topic suffix for compressed camera images.
        depth_suffix: Topic suffix for depth images.
        camera_info_suffix: Topic suffix for camera intrinsics.
        imu_suffix: Topic suffix for IMU data.
        gps_suffix: Topic suffix for GPS fixes.
        max_queue: Max buffered synced frames (older frames dropped on overflow).
    """

    def __init__(
        self,
        node: "rclpy.node.Node",
        topic_prefix: str = "/iphone",
        image_suffix: str = "camera/arkit/image/compressed",
        depth_suffix: str = "lidar/depth/image",
        camera_info_suffix: str = "camera/arkit/camera_info",
        imu_suffix: str = "imu",
        gps_suffix: str = "gps/fix",
        odom_suffix: str = "odom",
        max_queue: int = 2,
    ) -> None:
        self._node = node
        self._prefix = topic_prefix.rstrip("/")
        self._image_suffix = image_suffix
        self._depth_suffix = depth_suffix
        self._camera_info_suffix = camera_info_suffix
        self._imu_suffix = imu_suffix
        self._gps_suffix = gps_suffix
        self._odom_suffix = odom_suffix

        self._intrinsics: CameraIntrinsics | None = None
        self._intrinsics_event = threading.Event()
        self._frame_queue: queue.Queue[SyncedFrame | None] = queue.Queue(
            maxsize=max_queue
        )
        self._gps_fixes: list[GpsFix] = []
        self._imu_samples: list[ImuSample] = []
        self._odom_samples: list[OdomSample] = []
        self._latest_orientation: np.ndarray | None = None

        self._setup_subscribers()

    def _topic(self, suffix: str) -> str:
        return f"{self._prefix}/{suffix}"

    def _setup_subscribers(self) -> None:
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, NavSatFix

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._node.create_subscription(
            CameraInfo,
            self._topic(self._camera_info_suffix),
            self._on_camera_info,
            sensor_qos,
        )
        self._node.create_subscription(
            Imu, self._topic(self._imu_suffix), self._on_imu, sensor_qos
        )
        self._node.create_subscription(
            NavSatFix, self._topic(self._gps_suffix), self._on_gps, sensor_qos
        )
        self._node.create_subscription(
            Odometry, self._topic(self._odom_suffix), self._on_odom, sensor_qos
        )

        import message_filters

        image_sub = message_filters.Subscriber(
            self._node,
            CompressedImage,
            self._topic(self._image_suffix),
            qos_profile=sensor_qos,
        )
        depth_sub = message_filters.Subscriber(
            self._node,
            Image,
            self._topic(self._depth_suffix),
            qos_profile=sensor_qos,
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub], queue_size=5, slop=0.05
        )
        self._sync.registerCallback(self._on_synced)

    def _on_camera_info(self, msg: "CameraInfo") -> None:
        if self._intrinsics is not None:
            return
        self._intrinsics = CameraIntrinsics(
            width=msg.width,
            height=msg.height,
            K=np.array(msg.k, dtype=np.float64).reshape(3, 3),
            D=np.array(msg.d, dtype=np.float64),
            distortion_model=msg.distortion_model,
            frame_id=msg.header.frame_id,
        )
        self._intrinsics_event.set()

    def _on_imu(self, msg: "Imu") -> None:
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        o = msg.orientation
        orientation = np.array([o.x, o.y, o.z, o.w], dtype=np.float64)
        self._latest_orientation = orientation
        self._imu_samples.append(
            ImuSample(
                timestamp=ts,
                orientation=orientation,
                accel=np.array(
                    [
                        msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z,
                    ],
                    dtype=np.float64,
                ),
                gyro=np.array(
                    [
                        msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z,
                    ],
                    dtype=np.float64,
                ),
            )
        )

    def _on_gps(self, msg: "NavSatFix") -> None:
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._gps_fixes.append(
            GpsFix(
                timestamp=ts,
                latitude=msg.latitude,
                longitude=msg.longitude,
                altitude=msg.altitude,
                position_covariance=list(msg.position_covariance),
            )
        )

    def _on_odom(self, msg: "Odometry") -> None:
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._odom_samples.append(
            OdomSample(
                timestamp=ts,
                position=np.array([p.x, p.y, p.z], dtype=np.float64),
                orientation=np.array([q.x, q.y, q.z, q.w], dtype=np.float64),
                pose_covariance=np.array(
                    msg.pose.covariance, dtype=np.float64
                ).reshape(6, 6),
            )
        )

    def _on_synced(
        self, image_msg: "CompressedImage", depth_msg: "Image"
    ) -> None:
        if self._intrinsics is None:
            return

        buf = np.frombuffer(bytes(image_msg.data), dtype=np.uint8)
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if bgr is None:
            return

        if depth_msg.encoding == "32FC1":
            depth = np.frombuffer(bytes(depth_msg.data), dtype=np.float32).reshape(
                depth_msg.height, depth_msg.width
            )
        elif depth_msg.encoding == "16UC1":
            depth = (
                np.frombuffer(bytes(depth_msg.data), dtype=np.uint16)
                .reshape(depth_msg.height, depth_msg.width)
                .astype(np.float32)
                / 1000.0
            )
        else:
            return

        ts = (
            image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        )

        frame = SyncedFrame(
            timestamp=ts,
            image=bgr,
            depth=depth,
            intrinsics=self._intrinsics,
            orientation=self._latest_orientation,
        )

        while True:
            try:
                self._frame_queue.put_nowait(frame)
                return
            except queue.Full:
                try:
                    self._frame_queue.get_nowait()
                except queue.Empty:
                    pass

    def open(self) -> SensorStreams:
        """Block until camera intrinsics received, then return SensorStreams.

        The returned gps_fixes and imu_samples lists grow as new messages arrive.
        """
        print(
            f"LiveSource: waiting for camera_info on {self._topic(self._camera_info_suffix)}..."
        )
        self._intrinsics_event.wait()
        print(
            f"LiveSource: intrinsics received ({self._intrinsics.width}x{self._intrinsics.height})"
        )
        return SensorStreams(
            frames=self._frame_iterator(),
            gps_fixes=self._gps_fixes,
            imu_samples=self._imu_samples,
            odom_samples=self._odom_samples,
        )

    def _frame_iterator(self) -> Iterator[SyncedFrame]:
        """Blocking iterator that yields frames as they arrive."""
        while True:
            frame = self._frame_queue.get()
            if frame is None:
                return
            yield frame

    def stop(self) -> None:
        """Signal the frame iterator to stop."""
        while not self._frame_queue.empty():
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                break
        self._frame_queue.put_nowait(None)
