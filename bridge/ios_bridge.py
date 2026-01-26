#!/usr/bin/env python3
"""
iOS Sensor Bridge for ROS2

Receives CDR-encoded sensor data from CouchVision iOS app over TCP
and republishes to ROS2 topics.

Protocol: [topic_len:4][topic][data_len:4][data]
All integers are little-endian uint32.
"""

import socket
import struct
import threading
import argparse
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS2 message types
from sensor_msgs.msg import CompressedImage, Image, Imu, PointCloud2
from geometry_msgs.msg import Vector3Stamped


class iOSBridge(Node):
    def __init__(self, port: int = 7447):
        super().__init__('ios_bridge')
        self.port = port
        self.server_socket: Optional[socket.socket] = None
        self.client_socket: Optional[socket.socket] = None
        self.running = False

        # QoS for sensor data (best effort, keep last)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers - created dynamically based on received topics
        self.publishers = {}
        self.sensor_qos = sensor_qos

        # Topic type mapping based on topic name patterns
        self.topic_types = {
            '/image/compressed': CompressedImage,
            '/depth/image': Image,
            '/points': PointCloud2,
            '/imu': Imu,
            '/accelerometer': Vector3Stamped,
            '/gyroscope': Vector3Stamped,
        }

        self.get_logger().info(f'iOS Bridge initialized, will listen on port {port}')

    def get_publisher(self, topic: str):
        """Get or create a publisher for the given topic."""
        if topic not in self.publishers:
            # Determine message type from topic name
            msg_type = None
            for pattern, mtype in self.topic_types.items():
                if pattern in topic:
                    msg_type = mtype
                    break

            if msg_type is None:
                self.get_logger().warn(f'Unknown topic type for: {topic}, skipping')
                return None

            self.get_logger().info(f'Creating publisher for {topic} ({msg_type.__name__})')
            self.publishers[topic] = self.create_publisher(msg_type, topic, self.sensor_qos)

        return self.publishers[topic]

    def start_server(self):
        """Start TCP server to accept iOS connections."""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', self.port))
        self.server_socket.listen(1)
        self.running = True

        self.get_logger().info(f'Listening on 0.0.0.0:{self.port}')
        self.get_logger().info('Waiting for iOS device to connect...')

        while self.running:
            try:
                self.server_socket.settimeout(1.0)
                try:
                    client, addr = self.server_socket.accept()
                    self.get_logger().info(f'iOS device connected from {addr}')
                    self.handle_client(client)
                except socket.timeout:
                    continue
            except Exception as e:
                if self.running:
                    self.get_logger().error(f'Server error: {e}')
                break

    def handle_client(self, client: socket.socket):
        """Handle incoming data from iOS client."""
        self.client_socket = client
        client.settimeout(5.0)

        buffer = b''
        msg_count = 0

        while self.running:
            try:
                data = client.recv(65536)
                if not data:
                    self.get_logger().info('iOS device disconnected')
                    break

                buffer += data

                # Process complete messages from buffer
                while len(buffer) >= 8:  # Minimum: 4 (topic_len) + 4 (data_len)
                    # Read topic length
                    topic_len = struct.unpack('<I', buffer[:4])[0]

                    # Check if we have complete topic + data_len
                    if len(buffer) < 4 + topic_len + 4:
                        break

                    # Read topic
                    topic = buffer[4:4+topic_len].decode('utf-8')

                    # Read data length
                    data_len = struct.unpack('<I', buffer[4+topic_len:8+topic_len])[0]

                    # Check if we have complete message
                    total_len = 4 + topic_len + 4 + data_len
                    if len(buffer) < total_len:
                        break

                    # Extract CDR data
                    cdr_data = buffer[8+topic_len:total_len]

                    # Remove processed message from buffer
                    buffer = buffer[total_len:]

                    # Publish to ROS2
                    self.publish_message(topic, cdr_data)
                    msg_count += 1

                    if msg_count % 100 == 0:
                        self.get_logger().info(f'Processed {msg_count} messages')

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'Error handling client: {e}')
                break

        client.close()
        self.client_socket = None

    def publish_message(self, topic: str, cdr_data: bytes):
        """Publish CDR-encoded message to ROS2."""
        publisher = self.get_publisher(topic)
        if publisher is None:
            return

        try:
            # The cdr_data includes the 4-byte CDR encapsulation header
            # ROS2 rmw expects raw CDR without this header for some implementations
            # but with rcl, we can publish the serialized message directly

            # For now, we'll deserialize and republish
            # This is less efficient but more compatible
            msg_type = type(publisher.msg_type())

            # Create message and deserialize
            # Note: This requires the CDR data to match the message type exactly
            msg = msg_type()

            # For CompressedImage, we need to parse the CDR manually
            # since ROS2 Python doesn't expose direct CDR deserialization
            if msg_type == CompressedImage:
                msg = self.parse_compressed_image(cdr_data)
            elif msg_type == Image:
                msg = self.parse_image(cdr_data)
            elif msg_type == Imu:
                msg = self.parse_imu(cdr_data)
            elif msg_type == PointCloud2:
                msg = self.parse_pointcloud2(cdr_data)
            else:
                self.get_logger().warn(f'No parser for {msg_type.__name__}')
                return

            if msg:
                publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Failed to publish {topic}: {e}')

    def parse_compressed_image(self, cdr_data: bytes) -> Optional[CompressedImage]:
        """Parse CDR-encoded CompressedImage."""
        try:
            # Skip 4-byte CDR header
            data = cdr_data[4:]
            offset = 0

            msg = CompressedImage()

            # Header.stamp.sec (int32, aligned to 4)
            msg.header.stamp.sec = struct.unpack('<i', data[offset:offset+4])[0]
            offset += 4

            # Header.stamp.nanosec (uint32)
            msg.header.stamp.nanosec = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            # Header.frame_id (string: uint32 length + chars + null)
            str_len = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            msg.header.frame_id = data[offset:offset+str_len-1].decode('utf-8')
            offset += str_len

            # Align to 4 bytes
            offset = (offset + 3) & ~3

            # format (string)
            str_len = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            msg.format = data[offset:offset+str_len-1].decode('utf-8')
            offset += str_len

            # Align to 4 bytes
            offset = (offset + 3) & ~3

            # data (sequence<uint8>: uint32 length + bytes)
            data_len = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            msg.data = list(data[offset:offset+data_len])

            return msg
        except Exception as e:
            self.get_logger().error(f'Failed to parse CompressedImage: {e}')
            return None

    def parse_image(self, cdr_data: bytes) -> Optional[Image]:
        """Parse CDR-encoded Image."""
        try:
            data = cdr_data[4:]  # Skip CDR header
            offset = 0

            msg = Image()

            # Header
            msg.header.stamp.sec = struct.unpack('<i', data[offset:offset+4])[0]; offset += 4
            msg.header.stamp.nanosec = struct.unpack('<I', data[offset:offset+4])[0]; offset += 4
            str_len = struct.unpack('<I', data[offset:offset+4])[0]; offset += 4
            msg.header.frame_id = data[offset:offset+str_len-1].decode('utf-8'); offset += str_len
            offset = (offset + 3) & ~3

            # height, width
            msg.height = struct.unpack('<I', data[offset:offset+4])[0]; offset += 4
            msg.width = struct.unpack('<I', data[offset:offset+4])[0]; offset += 4

            # encoding
            str_len = struct.unpack('<I', data[offset:offset+4])[0]; offset += 4
            msg.encoding = data[offset:offset+str_len-1].decode('utf-8'); offset += str_len
            offset = (offset + 3) & ~3

            # is_bigendian, step
            msg.is_bigendian = data[offset]; offset += 1
            offset = (offset + 3) & ~3
            msg.step = struct.unpack('<I', data[offset:offset+4])[0]; offset += 4

            # data
            data_len = struct.unpack('<I', data[offset:offset+4])[0]; offset += 4
            msg.data = list(data[offset:offset+data_len])

            return msg
        except Exception as e:
            self.get_logger().error(f'Failed to parse Image: {e}')
            return None

    def parse_imu(self, cdr_data: bytes) -> Optional[Imu]:
        """Parse CDR-encoded Imu message."""
        try:
            data = cdr_data[4:]  # Skip CDR header
            offset = 0

            msg = Imu()

            # Header
            msg.header.stamp.sec = struct.unpack('<i', data[offset:offset+4])[0]; offset += 4
            msg.header.stamp.nanosec = struct.unpack('<I', data[offset:offset+4])[0]; offset += 4
            str_len = struct.unpack('<I', data[offset:offset+4])[0]; offset += 4
            msg.header.frame_id = data[offset:offset+str_len-1].decode('utf-8'); offset += str_len
            offset = (offset + 7) & ~7  # Align to 8 for doubles

            # Orientation quaternion (x, y, z, w)
            msg.orientation.x = struct.unpack('<d', data[offset:offset+8])[0]; offset += 8
            msg.orientation.y = struct.unpack('<d', data[offset:offset+8])[0]; offset += 8
            msg.orientation.z = struct.unpack('<d', data[offset:offset+8])[0]; offset += 8
            msg.orientation.w = struct.unpack('<d', data[offset:offset+8])[0]; offset += 8

            # Orientation covariance (9 doubles)
            for i in range(9):
                msg.orientation_covariance[i] = struct.unpack('<d', data[offset:offset+8])[0]; offset += 8

            # Angular velocity
            msg.angular_velocity.x = struct.unpack('<d', data[offset:offset+8])[0]; offset += 8
            msg.angular_velocity.y = struct.unpack('<d', data[offset:offset+8])[0]; offset += 8
            msg.angular_velocity.z = struct.unpack('<d', data[offset:offset+8])[0]; offset += 8

            # Angular velocity covariance
            for i in range(9):
                msg.angular_velocity_covariance[i] = struct.unpack('<d', data[offset:offset+8])[0]; offset += 8

            # Linear acceleration
            msg.linear_acceleration.x = struct.unpack('<d', data[offset:offset+8])[0]; offset += 8
            msg.linear_acceleration.y = struct.unpack('<d', data[offset:offset+8])[0]; offset += 8
            msg.linear_acceleration.z = struct.unpack('<d', data[offset:offset+8])[0]; offset += 8

            # Linear acceleration covariance
            for i in range(9):
                msg.linear_acceleration_covariance[i] = struct.unpack('<d', data[offset:offset+8])[0]; offset += 8

            return msg
        except Exception as e:
            self.get_logger().error(f'Failed to parse Imu: {e}')
            return None

    def parse_pointcloud2(self, cdr_data: bytes) -> Optional[PointCloud2]:
        """Parse CDR-encoded PointCloud2."""
        # PointCloud2 is complex - for now just log that we received it
        self.get_logger().info('Received PointCloud2 (parsing not yet implemented)')
        return None

    def stop(self):
        """Stop the bridge."""
        self.running = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()


def main():
    parser = argparse.ArgumentParser(description='iOS Sensor Bridge for ROS2')
    parser.add_argument('--port', type=int, default=7447, help='TCP port to listen on')
    args = parser.parse_args()

    rclpy.init()
    bridge = iOSBridge(port=args.port)

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


if __name__ == '__main__':
    main()
