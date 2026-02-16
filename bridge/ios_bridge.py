#!/usr/bin/env python3
"""iOS Sensor Bridge for ROS2.

Receives CDR-encoded sensor data from CouchVision iOS app over TCP
and republishes to ROS2 topics.

Protocol: [topic_len:4][topic][data_len:4][data]
All integers are little-endian uint32.
"""

from __future__ import annotations

import argparse
import queue
import socket
import struct
import threading
from collections.abc import Callable
from typing import TYPE_CHECKING, Any

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import (
    BatteryState,
    CameraInfo,
    CompressedImage,
    FluidPressure,
    Image,
    Imu,
    MagneticField,
    NavSatFix,
    PointCloud2,
    PointField,
)
from std_msgs.msg import Bool, ColorRGBA, Float64, Int32
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker

if TYPE_CHECKING:
    from rclpy.publisher import Publisher


class CdrReader:
    """XCDR1 CDR reader."""

    def __init__(self, cdr_data: bytes) -> None:
        self.data = cdr_data[4:]  # Skip 4-byte encapsulation header
        self.offset = 0

    def _align(self, boundary: int) -> None:
        remainder = self.offset % boundary
        if remainder != 0:
            self.offset += boundary - remainder

    def uint8(self) -> int:
        val = self.data[self.offset]
        self.offset += 1
        return val

    def int8(self) -> int:
        val = struct.unpack("<b", self.data[self.offset : self.offset + 1])[0]
        self.offset += 1
        return val

    def bool(self) -> bool:
        return bool(self.uint8())

    def uint16(self) -> int:
        self._align(2)
        val = struct.unpack("<H", self.data[self.offset : self.offset + 2])[0]
        self.offset += 2
        return val

    def int32(self) -> int:
        self._align(4)
        val = struct.unpack("<i", self.data[self.offset : self.offset + 4])[0]
        self.offset += 4
        return val

    def uint32(self) -> int:
        self._align(4)
        val = struct.unpack("<I", self.data[self.offset : self.offset + 4])[0]
        self.offset += 4
        return val

    def float32(self) -> float:
        self._align(4)
        val = struct.unpack("<f", self.data[self.offset : self.offset + 4])[0]
        self.offset += 4
        return val

    def float64(self) -> float:
        self._align(8)
        val = struct.unpack("<d", self.data[self.offset : self.offset + 8])[0]
        self.offset += 8
        return val

    def string(self) -> str:
        length = self.uint32()
        val = self.data[self.offset : self.offset + length - 1].decode("utf-8")
        self.offset += length
        return val

    def bytes_seq(self) -> bytes:
        length = self.uint32()
        val = self.data[self.offset : self.offset + length]
        self.offset += length
        return val

    def float64_array(self, count: int) -> list[float]:
        return [self.float64() for _ in range(count)]

    def float32_array(self, count: int) -> list[float]:
        return [self.float32() for _ in range(count)]


class IOSBridge(Node):  # type: ignore[misc]
    """ROS2 node that bridges iOS sensor data to ROS2 topics."""

    def __init__(self, port: int = 7447) -> None:
        super().__init__("ios_bridge")
        self.port = port
        self.server_socket: socket.socket | None = None
        self._tcp_clients: dict[str, socket.socket] = {}
        self._tcp_clients_lock = threading.Lock()
        self.running = False
        self._stop_event = threading.Event()

        self._sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._reliable_topics = {"/tf"}

        self._topic_publishers: dict[str, Publisher[Any]] = {}
        self._publish_queue: queue.Queue[tuple[str, bytes]] = queue.Queue()
        self.create_timer(0.001, self._drain_publish_queue)

        self._topic_types: dict[str, type[Any]] = {
            "/image/compressed": CompressedImage,
            "/camera_info": CameraInfo,
            "/depth/image": Image,
            "/points": PointCloud2,
            "/imu": Imu,
            "/accelerometer": Vector3Stamped,
            "/gyroscope": Vector3Stamped,
            "/tf": TFMessage,
            "/gps/fix": NavSatFix,
            "/gps/velocity": TwistStamped,
            "/heading": Float64,
            "/magnetic_field": MagneticField,
            "/pressure": FluidPressure,
            "/altitude": Float64,
            "/battery": BatteryState,
            "/thermal": Int32,
            "/proximity": Bool,
            "/odom": Odometry,
        }

        self._parsers: dict[type[Any], Callable[[bytes], Any | None]] = {
            CompressedImage: self._parse_compressed_image,
            Image: self._parse_image,
            Imu: self._parse_imu,
            PointCloud2: self._parse_pointcloud2,
            Vector3Stamped: self._parse_vector3_stamped,
            CameraInfo: self._parse_camera_info,
            TFMessage: self._parse_tf_message,
            NavSatFix: self._parse_navsatfix,
            TwistStamped: self._parse_twist_stamped,
            Float64: self._parse_float64,
            MagneticField: self._parse_magnetic_field,
            FluidPressure: self._parse_fluid_pressure,
            BatteryState: self._parse_battery_state,
            Int32: self._parse_int32,
            Bool: self._parse_bool,
            Odometry: self._parse_odometry,
        }

        # Couch bounding box marker (placeholder dimensions — adjust in Phase 0)
        self._marker_pub = self.create_publisher(Marker, "/couch/marker", self._reliable_qos)
        self.create_timer(1.0, self._publish_couch_marker)

        self.get_logger().info(f"iOS Bridge initialized, will listen on port {port}")

    def _drain_publish_queue(self) -> None:
        """Process queued messages on the main thread (timer callback)."""
        while not self._publish_queue.empty():
            try:
                topic, cdr_data = self._publish_queue.get_nowait()
            except queue.Empty:
                break
            publisher = self._get_or_create_publisher(topic)
            if publisher is None:
                continue
            self._do_publish(publisher, topic, cdr_data)

    def _get_or_create_publisher(self, topic: str) -> Publisher[Any] | None:
        """Get or create a publisher for the given topic. Must be called from main thread."""
        if topic in self._topic_publishers:
            return self._topic_publishers[topic]

        msg_type: type[Any] | None = None
        for pattern, mtype in self._topic_types.items():
            if topic == pattern or topic.endswith("/" + pattern.lstrip("/")):
                msg_type = mtype
                break

        if msg_type is None:
            self.get_logger().warn(f"Unknown topic type for: {topic}, skipping")
            return None

        qos = (
            self._reliable_qos
            if any(
                topic == rt or topic.endswith("/" + rt.lstrip("/")) for rt in self._reliable_topics
            )
            else self._sensor_qos
        )
        self.get_logger().info(f"Creating publisher for {topic} ({msg_type.__name__})")
        self._topic_publishers[topic] = self.create_publisher(msg_type, topic, qos)
        return self._topic_publishers[topic]

    @staticmethod
    def _get_interfaces() -> set[tuple[str, str]]:
        """Return set of (interface_name, ipv4_address) tuples."""
        import psutil

        interfaces: set[tuple[str, str]] = set()
        for name, addrs in psutil.net_if_addrs().items():
            for addr in addrs:
                if addr.family == socket.AF_INET and addr.address != "127.0.0.1":
                    interfaces.add((name, addr.address))
        return interfaces

    def _log_interfaces(self, interfaces: set[tuple[str, str]]) -> None:
        """Log available network interfaces."""
        if not interfaces:
            self.get_logger().warn("No network interfaces found")
            return
        self.get_logger().info("Available interfaces (connect your iPhone to any of these):")
        for name, ip in sorted(interfaces):
            self.get_logger().info(f"  {name}: {ip}:{self.port}")

    def _interface_monitor_loop(self) -> None:
        """Periodically check for new/removed network interfaces."""
        known = self._get_interfaces()
        self._log_interfaces(known)

        while not self._stop_event.wait(5.0):
            current = self._get_interfaces()
            added = current - known
            removed = known - current
            for name, ip in sorted(added):
                self.get_logger().info(
                    f"New network interface detected: {name} ({ip}) — connect on {ip}:{self.port}"
                )
            for name, ip in sorted(removed):
                self.get_logger().info(f"Network interface removed: {name} ({ip})")
            known = current

    def start_server(self) -> None:
        """Start TCP server to accept iOS connections."""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(("0.0.0.0", self.port))
        self.server_socket.listen(5)
        self.running = True

        self.get_logger().info(f"Listening on 0.0.0.0:{self.port}")

        monitor_thread = threading.Thread(target=self._interface_monitor_loop, daemon=True)
        monitor_thread.start()

        self.get_logger().info("Waiting for iOS devices to connect...")

        while self.running:
            try:
                self.server_socket.settimeout(1.0)
                try:
                    client, addr = self.server_socket.accept()
                    addr_key = f"{addr[0]}:{addr[1]}"
                    with self._tcp_clients_lock:
                        self._tcp_clients[addr_key] = client
                    self.get_logger().info(
                        f"iOS device connected from {addr} "
                        f"({len(self._tcp_clients)} client(s) connected)"
                    )
                    thread = threading.Thread(
                        target=self._client_thread,
                        args=(client, addr_key),
                        daemon=True,
                    )
                    thread.start()
                except TimeoutError:
                    continue
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Server error: {e}")
                break

    def _client_thread(self, client: socket.socket, addr_key: str) -> None:
        """Thread entry point for a single client connection."""
        try:
            self.handle_client(client)
        finally:
            with self._tcp_clients_lock:
                self._tcp_clients.pop(addr_key, None)
            self.get_logger().info(
                f"Client {addr_key} removed ({len(self._tcp_clients)} client(s) connected)"
            )

    def handle_client(self, client: socket.socket) -> None:
        """Handle incoming data from iOS client."""
        client.settimeout(5.0)

        buffer = b""
        msg_count = 0

        while self.running:
            try:
                data = client.recv(65536)
                if not data:
                    self.get_logger().info("iOS device disconnected")
                    break

                buffer += data

                while len(buffer) >= 8:
                    topic_len = struct.unpack("<I", buffer[:4])[0]

                    if len(buffer) < 4 + topic_len + 4:
                        break

                    topic = buffer[4 : 4 + topic_len].decode("utf-8")
                    data_len = struct.unpack("<I", buffer[4 + topic_len : 8 + topic_len])[0]

                    total_len = 4 + topic_len + 4 + data_len
                    if len(buffer) < total_len:
                        break

                    cdr_data = buffer[8 + topic_len : total_len]
                    buffer = buffer[total_len:]

                    self._publish_message(topic, cdr_data)
                    msg_count += 1

                    if msg_count % 100 == 0:
                        self.get_logger().info(f"Processed {msg_count} messages")

            except TimeoutError:
                continue
            except Exception as e:
                self.get_logger().error(f"Error handling client: {e}")
                break

        client.close()

    def _publish_message(self, topic: str, cdr_data: bytes) -> None:
        """Queue a message for publishing on the main thread."""
        self._publish_queue.put((topic, cdr_data))

    def _do_publish(self, publisher: Publisher[Any], topic: str, cdr_data: bytes) -> None:
        """Parse and publish a CDR message. Must be called from main thread."""
        msg_type = type(publisher.msg_type())
        parser = self._parsers.get(msg_type)
        if parser is None:
            self.get_logger().warn(f"No parser for {msg_type.__name__}")
            return

        try:
            msg = parser(cdr_data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse {topic}: {e}")
            return

        if msg is None:
            return

        publisher.publish(msg)

        if msg_type == CompressedImage:
            self._publish_decompressed(topic, msg)

    def _publish_decompressed(self, compressed_topic: str, msg: CompressedImage) -> None:
        """Decompress JPEG and publish as raw Image."""
        raw_topic = compressed_topic.replace("/image/compressed", "/image")
        if raw_topic not in self._topic_publishers:
            self._topic_publishers[raw_topic] = self.create_publisher(
                Image, raw_topic, self._sensor_qos
            )

        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if bgr is None:
            return

        # Downscale to max 640px wide so rviz can keep up
        h, w = bgr.shape[:2]
        max_w = 640
        if w > max_w:
            scale = max_w / w
            bgr = cv2.resize(bgr, (max_w, int(h * scale)), interpolation=cv2.INTER_AREA)

        raw = Image()
        raw.header = msg.header
        raw.height, raw.width = bgr.shape[:2]
        raw.encoding = "bgr8"
        raw.is_bigendian = 0
        raw.step = raw.width * 3
        raw.data = bgr.tobytes()
        self._topic_publishers[raw_topic].publish(raw)

    # -- CDR helpers --

    def _read_header(self, r: CdrReader, msg: Any) -> None:
        msg.header.stamp.sec = r.int32()
        msg.header.stamp.nanosec = r.uint32()
        msg.header.frame_id = r.string()

    def _read_vector3(self, r: CdrReader, v: Any) -> None:
        v.x = r.float64()
        v.y = r.float64()
        v.z = r.float64()

    def _read_quaternion(self, r: CdrReader, q: Any) -> None:
        q.x = r.float64()
        q.y = r.float64()
        q.z = r.float64()
        q.w = r.float64()

    # -- Parsers --

    def _parse_compressed_image(self, cdr_data: bytes) -> CompressedImage:
        r = CdrReader(cdr_data)
        msg = CompressedImage()
        self._read_header(r, msg)
        msg.format = r.string()
        msg.data = list(r.bytes_seq())
        return msg

    def _parse_image(self, cdr_data: bytes) -> Image:
        r = CdrReader(cdr_data)
        msg = Image()
        self._read_header(r, msg)
        msg.height = r.uint32()
        msg.width = r.uint32()
        msg.encoding = r.string()
        msg.is_bigendian = r.uint8()
        msg.step = r.uint32()
        msg.data = list(r.bytes_seq())
        return msg

    def _parse_imu(self, cdr_data: bytes) -> Imu:
        r = CdrReader(cdr_data)
        msg = Imu()
        self._read_header(r, msg)
        self._read_quaternion(r, msg.orientation)
        msg.orientation_covariance = r.float64_array(9)
        self._read_vector3(r, msg.angular_velocity)
        msg.angular_velocity_covariance = r.float64_array(9)
        self._read_vector3(r, msg.linear_acceleration)
        msg.linear_acceleration_covariance = r.float64_array(9)
        return msg

    def _parse_pointcloud2(self, cdr_data: bytes) -> PointCloud2:
        r = CdrReader(cdr_data)
        msg = PointCloud2()
        self._read_header(r, msg)
        msg.height = r.uint32()
        msg.width = r.uint32()
        num_fields = r.uint32()
        fields = []
        for _ in range(num_fields):
            f = PointField()
            f.name = r.string()
            f.offset = r.uint32()
            f.datatype = r.uint8()
            f.count = r.uint32()
            fields.append(f)
        msg.fields = fields
        msg.is_bigendian = bool(r.uint8())
        msg.point_step = r.uint32()
        msg.row_step = r.uint32()
        msg.data = list(r.bytes_seq())
        msg.is_dense = bool(r.uint8())
        return msg

    def _parse_vector3_stamped(self, cdr_data: bytes) -> Vector3Stamped:
        r = CdrReader(cdr_data)
        msg = Vector3Stamped()
        self._read_header(r, msg)
        self._read_vector3(r, msg.vector)
        return msg

    def _parse_camera_info(self, cdr_data: bytes) -> CameraInfo:
        r = CdrReader(cdr_data)
        msg = CameraInfo()
        self._read_header(r, msg)
        msg.height = r.uint32()
        msg.width = r.uint32()
        msg.distortion_model = r.string()
        d_len = r.uint32()
        msg.d = r.float64_array(d_len)
        msg.k = r.float64_array(9)
        msg.r = r.float64_array(9)
        msg.p = r.float64_array(12)
        msg.binning_x = r.uint32()
        msg.binning_y = r.uint32()
        return msg

    def _parse_tf_message(self, cdr_data: bytes) -> TFMessage:
        r = CdrReader(cdr_data)
        msg = TFMessage()
        num_transforms = r.uint32()
        for _ in range(num_transforms):
            tf = TransformStamped()
            self._read_header(r, tf)
            tf.child_frame_id = r.string()
            self._read_vector3(r, tf.transform.translation)
            self._read_quaternion(r, tf.transform.rotation)
            msg.transforms.append(tf)
        return msg

    def _parse_navsatfix(self, cdr_data: bytes) -> NavSatFix:
        r = CdrReader(cdr_data)
        msg = NavSatFix()
        self._read_header(r, msg)
        msg.status.status = r.int8()
        msg.status.service = r.uint16()
        msg.latitude = r.float64()
        msg.longitude = r.float64()
        msg.altitude = r.float64()
        msg.position_covariance = r.float64_array(9)
        msg.position_covariance_type = r.uint8()
        return msg

    def _parse_twist_stamped(self, cdr_data: bytes) -> TwistStamped:
        r = CdrReader(cdr_data)
        msg = TwistStamped()
        self._read_header(r, msg)
        self._read_vector3(r, msg.twist.linear)
        self._read_vector3(r, msg.twist.angular)
        return msg

    def _parse_float64(self, cdr_data: bytes) -> Float64:
        r = CdrReader(cdr_data)
        msg = Float64()
        msg.data = r.float64()
        return msg

    def _parse_magnetic_field(self, cdr_data: bytes) -> MagneticField:
        r = CdrReader(cdr_data)
        msg = MagneticField()
        self._read_header(r, msg)
        self._read_vector3(r, msg.magnetic_field)
        msg.magnetic_field_covariance = r.float64_array(9)
        return msg

    def _parse_fluid_pressure(self, cdr_data: bytes) -> FluidPressure:
        r = CdrReader(cdr_data)
        msg = FluidPressure()
        self._read_header(r, msg)
        msg.fluid_pressure = r.float64()
        msg.variance = r.float64()
        return msg

    def _parse_battery_state(self, cdr_data: bytes) -> BatteryState:
        r = CdrReader(cdr_data)
        msg = BatteryState()
        self._read_header(r, msg)
        msg.voltage = r.float32()
        msg.temperature = r.float32()
        msg.current = r.float32()
        msg.charge = r.float32()
        msg.capacity = r.float32()
        msg.design_capacity = r.float32()
        msg.percentage = r.float32()
        msg.power_supply_status = r.uint8()
        msg.power_supply_health = r.uint8()
        msg.power_supply_technology = r.uint8()
        msg.present = r.bool()
        return msg

    def _parse_int32(self, cdr_data: bytes) -> Int32:
        r = CdrReader(cdr_data)
        msg = Int32()
        msg.data = r.int32()
        return msg

    def _parse_odometry(self, cdr_data: bytes) -> Odometry:
        r = CdrReader(cdr_data)
        msg = Odometry()
        self._read_header(r, msg)
        msg.child_frame_id = r.string()
        self._read_vector3(r, msg.pose.pose.position)
        self._read_quaternion(r, msg.pose.pose.orientation)
        msg.pose.covariance = r.float64_array(36)
        self._read_vector3(r, msg.twist.twist.linear)
        self._read_vector3(r, msg.twist.twist.angular)
        msg.twist.covariance = r.float64_array(36)
        return msg

    def _parse_bool(self, cdr_data: bytes) -> Bool:
        r = CdrReader(cdr_data)
        msg = Bool()
        msg.data = r.bool()
        return msg

    def _publish_couch_marker(self) -> None:
        """Publish a box marker representing the couch footprint."""
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "couch"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.z = 0.4
        m.pose.orientation.w = 1.0
        m.scale.x = 1.5  # length (meters) — adjust when measured
        m.scale.y = 0.7  # width
        m.scale.z = 0.8  # height
        m.color = ColorRGBA(r=0.4, g=0.3, b=0.2, a=0.6)
        m.lifetime.sec = 2
        self._marker_pub.publish(m)

    def stop(self) -> None:
        """Stop the bridge."""
        self.running = False
        self._stop_event.set()
        with self._tcp_clients_lock:
            for client in self._tcp_clients.values():
                client.close()
            self._tcp_clients.clear()
        if self.server_socket:
            self.server_socket.close()


def main() -> None:
    """Entry point for the iOS bridge."""
    parser = argparse.ArgumentParser(description="iOS Sensor Bridge for ROS2")
    parser.add_argument("--port", type=int, default=7447, help="TCP port to listen on")
    args = parser.parse_args()

    rclpy.init()
    bridge = IOSBridge(port=args.port)

    server_thread = threading.Thread(target=bridge.start_server, daemon=True)
    server_thread.start()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
