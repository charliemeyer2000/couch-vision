#!/usr/bin/env python3
"""iOS Sensor Bridge for ROS2.

Receives CDR-encoded sensor data from CouchVision iOS app over TCP
and republishes to ROS2 topics.

Protocol: [topic_len:4][topic][data_len:4][data]
All integers are little-endian uint32.
"""

from __future__ import annotations

import argparse
import socket
import struct
import threading
from typing import TYPE_CHECKING, Any

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped, Vector3Stamped
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
)
from std_msgs.msg import Bool, Float64, Int32
from tf2_msgs.msg import TFMessage

if TYPE_CHECKING:
    from rclpy.publisher import Publisher


class IOSBridge(Node):  # type: ignore[misc]
    """ROS2 node that bridges iOS sensor data to ROS2 topics."""

    def __init__(self, port: int = 7447) -> None:
        super().__init__("ios_bridge")
        self.port = port
        self.server_socket: socket.socket | None = None
        self.client_socket: socket.socket | None = None
        self.running = False

        # QoS for sensor data (best effort, keep last)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers - created dynamically based on received topics
        self._topic_publishers: dict[str, Publisher[Any]] = {}
        self._sensor_qos = sensor_qos

        # Topic type mapping based on topic name patterns
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
        }

        self.get_logger().info(f"iOS Bridge initialized, will listen on port {port}")

    def get_publisher(self, topic: str) -> Publisher[Any] | None:
        """Get or create a publisher for the given topic."""
        if topic not in self._topic_publishers:
            # Determine message type from topic name
            msg_type: type[Any] | None = None
            for pattern, mtype in self._topic_types.items():
                if topic == pattern or topic.endswith(pattern):
                    msg_type = mtype
                    break

            if msg_type is None:
                self.get_logger().warn(f"Unknown topic type for: {topic}, skipping")
                return None

            self.get_logger().info(f"Creating publisher for {topic} ({msg_type.__name__})")
            self._topic_publishers[topic] = self.create_publisher(msg_type, topic, self._sensor_qos)

        return self._topic_publishers[topic]

    def start_server(self) -> None:
        """Start TCP server to accept iOS connections."""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(("0.0.0.0", self.port))
        self.server_socket.listen(1)
        self.running = True

        self.get_logger().info(f"Listening on 0.0.0.0:{self.port}")
        self.get_logger().info("Waiting for iOS device to connect...")

        while self.running:
            try:
                self.server_socket.settimeout(1.0)
                try:
                    client, addr = self.server_socket.accept()
                    self.get_logger().info(f"iOS device connected from {addr}")
                    self.handle_client(client)
                except TimeoutError:
                    continue
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Server error: {e}")
                break

    def handle_client(self, client: socket.socket) -> None:
        """Handle incoming data from iOS client."""
        self.client_socket = client
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

                # Process complete messages from buffer
                while len(buffer) >= 8:  # Minimum: 4 (topic_len) + 4 (data_len)
                    # Read topic length
                    topic_len = struct.unpack("<I", buffer[:4])[0]

                    # Check if we have complete topic + data_len
                    if len(buffer) < 4 + topic_len + 4:
                        break

                    # Read topic
                    topic = buffer[4 : 4 + topic_len].decode("utf-8")

                    # Read data length
                    data_len = struct.unpack("<I", buffer[4 + topic_len : 8 + topic_len])[0]

                    # Check if we have complete message
                    total_len = 4 + topic_len + 4 + data_len
                    if len(buffer) < total_len:
                        break

                    # Extract CDR data
                    cdr_data = buffer[8 + topic_len : total_len]

                    # Remove processed message from buffer
                    buffer = buffer[total_len:]

                    # Publish to ROS2
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
        self.client_socket = None

    def _publish_message(self, topic: str, cdr_data: bytes) -> None:
        """Publish CDR-encoded message to ROS2."""
        publisher = self.get_publisher(topic)
        if publisher is None:
            return

        try:
            # Get the message type from the publisher
            msg_type = type(publisher.msg_type())

            # Parse based on message type
            msg: Any = None
            if msg_type == CompressedImage:
                msg = self._parse_compressed_image(cdr_data)
            elif msg_type == Image:
                msg = self._parse_image(cdr_data)
            elif msg_type == Imu:
                msg = self._parse_imu(cdr_data)
            elif msg_type == PointCloud2:
                msg = self._parse_pointcloud2(cdr_data)
            elif msg_type == Vector3Stamped:
                msg = self._parse_vector3_stamped(cdr_data)
            elif msg_type == CameraInfo:
                msg = self._parse_camera_info(cdr_data)
            elif msg_type == TFMessage:
                msg = self._parse_tf_message(cdr_data)
            elif msg_type == NavSatFix:
                msg = self._parse_navsatfix(cdr_data)
            elif msg_type == TwistStamped:
                msg = self._parse_twist_stamped(cdr_data)
            elif msg_type == Float64:
                msg = self._parse_float64(cdr_data)
            elif msg_type == MagneticField:
                msg = self._parse_magnetic_field(cdr_data)
            elif msg_type == FluidPressure:
                msg = self._parse_fluid_pressure(cdr_data)
            elif msg_type == BatteryState:
                msg = self._parse_battery_state(cdr_data)
            elif msg_type == Int32:
                msg = self._parse_int32(cdr_data)
            elif msg_type == Bool:
                msg = self._parse_bool(cdr_data)
            else:
                self.get_logger().warn(f"No parser for {msg_type.__name__}")
                return

            if msg:
                publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Failed to publish {topic}: {e}")

    def _parse_compressed_image(self, cdr_data: bytes) -> CompressedImage | None:
        """Parse CDR-encoded CompressedImage."""
        try:
            # Skip 4-byte CDR header
            data = cdr_data[4:]
            offset = 0

            msg = CompressedImage()

            # Header.stamp.sec (int32, aligned to 4)
            msg.header.stamp.sec = struct.unpack("<i", data[offset : offset + 4])[0]
            offset += 4

            # Header.stamp.nanosec (uint32)
            msg.header.stamp.nanosec = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4

            # Header.frame_id (string: uint32 length + chars + null)
            str_len = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.header.frame_id = data[offset : offset + str_len - 1].decode("utf-8")
            offset += str_len

            # Align to 4 bytes
            offset = (offset + 3) & ~3

            # format (string)
            str_len = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.format = data[offset : offset + str_len - 1].decode("utf-8")
            offset += str_len

            # Align to 4 bytes
            offset = (offset + 3) & ~3

            # data (sequence<uint8>: uint32 length + bytes)
            data_len = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.data = list(data[offset : offset + data_len])

            return msg
        except Exception as e:
            self.get_logger().error(f"Failed to parse CompressedImage: {e}")
            return None

    def _parse_image(self, cdr_data: bytes) -> Image | None:
        """Parse CDR-encoded Image."""
        try:
            data = cdr_data[4:]  # Skip CDR header
            offset = 0

            msg = Image()

            # Header
            msg.header.stamp.sec = struct.unpack("<i", data[offset : offset + 4])[0]
            offset += 4
            msg.header.stamp.nanosec = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            str_len = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.header.frame_id = data[offset : offset + str_len - 1].decode("utf-8")
            offset += str_len
            offset = (offset + 3) & ~3

            # height, width
            msg.height = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.width = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4

            # encoding
            str_len = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.encoding = data[offset : offset + str_len - 1].decode("utf-8")
            offset += str_len
            offset = (offset + 3) & ~3

            # is_bigendian, step
            msg.is_bigendian = data[offset]
            offset += 1
            offset = (offset + 3) & ~3
            msg.step = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4

            # data
            data_len = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.data = list(data[offset : offset + data_len])

            return msg
        except Exception as e:
            self.get_logger().error(f"Failed to parse Image: {e}")
            return None

    def _parse_imu(self, cdr_data: bytes) -> Imu | None:
        """Parse CDR-encoded Imu message."""
        try:
            data = cdr_data[4:]  # Skip CDR header
            offset = 0

            msg = Imu()

            # Header
            msg.header.stamp.sec = struct.unpack("<i", data[offset : offset + 4])[0]
            offset += 4
            msg.header.stamp.nanosec = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            str_len = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.header.frame_id = data[offset : offset + str_len - 1].decode("utf-8")
            offset += str_len
            offset = (offset + 7) & ~7  # Align to 8 for doubles

            # Orientation quaternion (x, y, z, w)
            msg.orientation.x = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.orientation.y = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.orientation.z = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.orientation.w = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8

            # Orientation covariance (9 doubles)
            for i in range(9):
                msg.orientation_covariance[i] = struct.unpack("<d", data[offset : offset + 8])[0]
                offset += 8

            # Angular velocity
            msg.angular_velocity.x = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.angular_velocity.y = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.angular_velocity.z = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8

            # Angular velocity covariance
            for i in range(9):
                msg.angular_velocity_covariance[i] = struct.unpack("<d", data[offset : offset + 8])[
                    0
                ]
                offset += 8

            # Linear acceleration
            msg.linear_acceleration.x = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.linear_acceleration.y = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.linear_acceleration.z = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8

            # Linear acceleration covariance
            for i in range(9):
                msg.linear_acceleration_covariance[i] = struct.unpack(
                    "<d", data[offset : offset + 8]
                )[0]
                offset += 8

            return msg
        except Exception as e:
            self.get_logger().error(f"Failed to parse Imu: {e}")
            return None

    def _parse_pointcloud2(self, cdr_data: bytes) -> PointCloud2 | None:
        """Parse CDR-encoded PointCloud2."""
        # PointCloud2 is complex - for now just log that we received it
        _ = cdr_data  # unused for now
        self.get_logger().info("Received PointCloud2 (parsing not yet implemented)")
        return None

    def _parse_vector3_stamped(self, cdr_data: bytes) -> Vector3Stamped | None:
        """Parse CDR-encoded Vector3Stamped."""
        try:
            data = cdr_data[4:]  # Skip CDR header
            offset = 0

            msg = Vector3Stamped()

            # Header
            msg.header.stamp.sec = struct.unpack("<i", data[offset : offset + 4])[0]
            offset += 4
            msg.header.stamp.nanosec = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            str_len = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.header.frame_id = data[offset : offset + str_len - 1].decode("utf-8")
            offset += str_len
            offset = (offset + 7) & ~7  # Align to 8 for doubles

            # Vector3
            msg.vector.x = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.vector.y = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.vector.z = struct.unpack("<d", data[offset : offset + 8])[0]

            return msg
        except Exception as e:
            self.get_logger().error(f"Failed to parse Vector3Stamped: {e}")
            return None

    def _parse_camera_info(self, cdr_data: bytes) -> CameraInfo | None:
        """Parse CDR-encoded CameraInfo."""
        try:
            data = cdr_data[4:]  # Skip CDR header
            offset = 0

            msg = CameraInfo()

            # Header
            msg.header.stamp.sec = struct.unpack("<i", data[offset : offset + 4])[0]
            offset += 4
            msg.header.stamp.nanosec = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            str_len = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.header.frame_id = data[offset : offset + str_len - 1].decode("utf-8")
            offset += str_len
            offset = (offset + 3) & ~3  # Align to 4

            # height, width
            msg.height = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.width = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4

            # distortion_model (string)
            str_len = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.distortion_model = data[offset : offset + str_len - 1].decode("utf-8")
            offset += str_len
            offset = (offset + 7) & ~7  # Align to 8 for doubles

            # D (sequence of doubles)
            d_len = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            offset = (offset + 7) & ~7  # Align to 8
            msg.d = [
                struct.unpack("<d", data[offset + i * 8 : offset + (i + 1) * 8])[0]
                for i in range(d_len)
            ]
            offset += d_len * 8

            # K (9 doubles)
            msg.k = [
                struct.unpack("<d", data[offset + i * 8 : offset + (i + 1) * 8])[0]
                for i in range(9)
            ]
            offset += 72

            # R (9 doubles)
            msg.r = [
                struct.unpack("<d", data[offset + i * 8 : offset + (i + 1) * 8])[0]
                for i in range(9)
            ]
            offset += 72

            # P (12 doubles)
            msg.p = [
                struct.unpack("<d", data[offset + i * 8 : offset + (i + 1) * 8])[0]
                for i in range(12)
            ]
            offset += 96

            # binning_x, binning_y
            msg.binning_x = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.binning_y = struct.unpack("<I", data[offset : offset + 4])[0]

            return msg
        except Exception as e:
            self.get_logger().error(f"Failed to parse CameraInfo: {e}")
            return None

    def _parse_tf_message(self, cdr_data: bytes) -> TFMessage | None:
        """Parse CDR-encoded TFMessage."""
        try:
            data = cdr_data[4:]  # Skip CDR header
            offset = 0

            msg = TFMessage()

            # transforms (sequence)
            num_transforms = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4

            for _ in range(num_transforms):
                tf = TransformStamped()

                # Header
                tf.header.stamp.sec = struct.unpack("<i", data[offset : offset + 4])[0]
                offset += 4
                tf.header.stamp.nanosec = struct.unpack("<I", data[offset : offset + 4])[0]
                offset += 4
                str_len = struct.unpack("<I", data[offset : offset + 4])[0]
                offset += 4
                tf.header.frame_id = data[offset : offset + str_len - 1].decode("utf-8")
                offset += str_len
                offset = (offset + 3) & ~3

                # child_frame_id
                str_len = struct.unpack("<I", data[offset : offset + 4])[0]
                offset += 4
                tf.child_frame_id = data[offset : offset + str_len - 1].decode("utf-8")
                offset += str_len
                offset = (offset + 7) & ~7  # Align to 8 for doubles

                # Transform: translation (Vector3)
                tf.transform.translation.x = struct.unpack("<d", data[offset : offset + 8])[0]
                offset += 8
                tf.transform.translation.y = struct.unpack("<d", data[offset : offset + 8])[0]
                offset += 8
                tf.transform.translation.z = struct.unpack("<d", data[offset : offset + 8])[0]
                offset += 8

                # Transform: rotation (Quaternion)
                tf.transform.rotation.x = struct.unpack("<d", data[offset : offset + 8])[0]
                offset += 8
                tf.transform.rotation.y = struct.unpack("<d", data[offset : offset + 8])[0]
                offset += 8
                tf.transform.rotation.z = struct.unpack("<d", data[offset : offset + 8])[0]
                offset += 8
                tf.transform.rotation.w = struct.unpack("<d", data[offset : offset + 8])[0]
                offset += 8

                msg.transforms.append(tf)

            return msg
        except Exception as e:
            self.get_logger().error(f"Failed to parse TFMessage: {e}")
            return None

    def _parse_navsatfix(self, cdr_data: bytes) -> NavSatFix | None:
        """Parse CDR-encoded NavSatFix."""
        try:
            data = cdr_data[4:]  # Skip CDR header
            offset = 0

            msg = NavSatFix()

            # Header
            msg.header.stamp.sec = struct.unpack("<i", data[offset : offset + 4])[0]
            offset += 4
            msg.header.stamp.nanosec = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            str_len = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.header.frame_id = data[offset : offset + str_len - 1].decode("utf-8")
            offset += str_len
            offset = (offset + 7) & ~7  # Align to 8

            # NavSatStatus
            msg.status.status = struct.unpack("<b", data[offset : offset + 1])[0]
            offset += 1
            offset = (offset + 1) & ~1  # Align to 2
            msg.status.service = struct.unpack("<H", data[offset : offset + 2])[0]
            offset += 2
            offset = (offset + 7) & ~7  # Align to 8

            # latitude, longitude, altitude (doubles)
            msg.latitude = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.longitude = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.altitude = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8

            # position_covariance (9 doubles)
            for i in range(9):
                msg.position_covariance[i] = struct.unpack("<d", data[offset : offset + 8])[0]
                offset += 8

            # position_covariance_type
            msg.position_covariance_type = data[offset]

            return msg
        except Exception as e:
            self.get_logger().error(f"Failed to parse NavSatFix: {e}")
            return None

    def _parse_twist_stamped(self, cdr_data: bytes) -> TwistStamped | None:
        """Parse CDR-encoded TwistStamped."""
        try:
            data = cdr_data[4:]  # Skip CDR header
            offset = 0

            msg = TwistStamped()

            # Header
            msg.header.stamp.sec = struct.unpack("<i", data[offset : offset + 4])[0]
            offset += 4
            msg.header.stamp.nanosec = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            str_len = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.header.frame_id = data[offset : offset + str_len - 1].decode("utf-8")
            offset += str_len
            offset = (offset + 7) & ~7  # Align to 8 for doubles

            # Twist: linear (Vector3)
            msg.twist.linear.x = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.twist.linear.y = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.twist.linear.z = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8

            # Twist: angular (Vector3)
            msg.twist.angular.x = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.twist.angular.y = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.twist.angular.z = struct.unpack("<d", data[offset : offset + 8])[0]

            return msg
        except Exception as e:
            self.get_logger().error(f"Failed to parse TwistStamped: {e}")
            return None

    def _parse_float64(self, cdr_data: bytes) -> Float64 | None:
        """Parse CDR-encoded Float64."""
        try:
            data = cdr_data[4:]  # Skip CDR header
            msg = Float64()
            msg.data = struct.unpack("<d", data[:8])[0]
            return msg
        except Exception as e:
            self.get_logger().error(f"Failed to parse Float64: {e}")
            return None

    def _parse_magnetic_field(self, cdr_data: bytes) -> MagneticField | None:
        """Parse CDR-encoded MagneticField."""
        try:
            data = cdr_data[4:]  # Skip CDR header
            offset = 0

            msg = MagneticField()

            # Header
            msg.header.stamp.sec = struct.unpack("<i", data[offset : offset + 4])[0]
            offset += 4
            msg.header.stamp.nanosec = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            str_len = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.header.frame_id = data[offset : offset + str_len - 1].decode("utf-8")
            offset += str_len
            offset = (offset + 7) & ~7  # Align to 8 for doubles

            # magnetic_field (Vector3)
            msg.magnetic_field.x = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.magnetic_field.y = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.magnetic_field.z = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8

            # magnetic_field_covariance (9 doubles)
            for i in range(9):
                msg.magnetic_field_covariance[i] = struct.unpack("<d", data[offset : offset + 8])[0]
                offset += 8

            return msg
        except Exception as e:
            self.get_logger().error(f"Failed to parse MagneticField: {e}")
            return None

    def _parse_fluid_pressure(self, cdr_data: bytes) -> FluidPressure | None:
        """Parse CDR-encoded FluidPressure."""
        try:
            data = cdr_data[4:]  # Skip CDR header
            offset = 0

            msg = FluidPressure()

            # Header
            msg.header.stamp.sec = struct.unpack("<i", data[offset : offset + 4])[0]
            offset += 4
            msg.header.stamp.nanosec = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            str_len = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.header.frame_id = data[offset : offset + str_len - 1].decode("utf-8")
            offset += str_len
            offset = (offset + 7) & ~7  # Align to 8 for doubles

            # fluid_pressure, variance (doubles)
            msg.fluid_pressure = struct.unpack("<d", data[offset : offset + 8])[0]
            offset += 8
            msg.variance = struct.unpack("<d", data[offset : offset + 8])[0]

            return msg
        except Exception as e:
            self.get_logger().error(f"Failed to parse FluidPressure: {e}")
            return None

    def _parse_battery_state(self, cdr_data: bytes) -> BatteryState | None:
        """Parse CDR-encoded BatteryState."""
        try:
            data = cdr_data[4:]  # Skip CDR header
            offset = 0

            msg = BatteryState()

            # Header
            msg.header.stamp.sec = struct.unpack("<i", data[offset : offset + 4])[0]
            offset += 4
            msg.header.stamp.nanosec = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            str_len = struct.unpack("<I", data[offset : offset + 4])[0]
            offset += 4
            msg.header.frame_id = data[offset : offset + str_len - 1].decode("utf-8")
            offset += str_len
            offset = (offset + 3) & ~3  # Align to 4 for floats

            # voltage, temperature, current, charge, capacity, design_capacity (floats)
            msg.voltage = struct.unpack("<f", data[offset : offset + 4])[0]
            offset += 4
            msg.temperature = struct.unpack("<f", data[offset : offset + 4])[0]
            offset += 4
            msg.current = struct.unpack("<f", data[offset : offset + 4])[0]
            offset += 4
            msg.charge = struct.unpack("<f", data[offset : offset + 4])[0]
            offset += 4
            msg.capacity = struct.unpack("<f", data[offset : offset + 4])[0]
            offset += 4
            msg.design_capacity = struct.unpack("<f", data[offset : offset + 4])[0]
            offset += 4

            # percentage (float)
            msg.percentage = struct.unpack("<f", data[offset : offset + 4])[0]
            offset += 4

            # power_supply_status, health, technology (uint8)
            msg.power_supply_status = data[offset]
            offset += 1
            msg.power_supply_health = data[offset]
            offset += 1
            msg.power_supply_technology = data[offset]
            offset += 1

            # present (bool as uint8)
            msg.present = bool(data[offset])

            return msg
        except Exception as e:
            self.get_logger().error(f"Failed to parse BatteryState: {e}")
            return None

    def _parse_int32(self, cdr_data: bytes) -> Int32 | None:
        """Parse CDR-encoded Int32."""
        try:
            data = cdr_data[4:]  # Skip CDR header
            msg = Int32()
            msg.data = struct.unpack("<i", data[:4])[0]
            return msg
        except Exception as e:
            self.get_logger().error(f"Failed to parse Int32: {e}")
            return None

    def _parse_bool(self, cdr_data: bytes) -> Bool | None:
        """Parse CDR-encoded Bool."""
        try:
            data = cdr_data[4:]  # Skip CDR header
            msg = Bool()
            msg.data = bool(data[0])
            return msg
        except Exception as e:
            self.get_logger().error(f"Failed to parse Bool: {e}")
            return None

    def stop(self) -> None:
        """Stop the bridge."""
        self.running = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()


def main() -> None:
    """Entry point for the iOS bridge."""
    parser = argparse.ArgumentParser(description="iOS Sensor Bridge for ROS2")
    parser.add_argument("--port", type=int, default=7447, help="TCP port to listen on")
    args = parser.parse_args()

    rclpy.init()
    bridge = IOSBridge(port=args.port)

    # Run server in separate thread
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
