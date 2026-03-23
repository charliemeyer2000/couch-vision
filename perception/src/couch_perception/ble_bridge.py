"""BLE GATT server bridge — receives teleop commands over Bluetooth Low Energy.

Runs natively on the Jetson (not in Docker) so it has direct BlueZ D-Bus
access.  Publishes received cmd_vel and e_stop to ROS2 via CycloneDDS,
which the VESC driver container sees (network_mode=host in live mode).

GATT service layout:
  CMD_VEL   (write-no-response)  8 bytes: 2x float32 LE (linear_x, angular_z)
  E_STOP    (write)              1 byte:  0x00=release, 0x01=stop
  HEARTBEAT (notify)             1 byte:  wrapping counter, 500ms interval
"""

from __future__ import annotations

import asyncio
import logging
import struct
import threading
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from bless import (
    BlessGATTCharacteristic,
    BlessServer,
    GATTAttributePermissions,
    GATTCharacteristicProperties,
)

logger = logging.getLogger(__name__)

# ── GATT UUIDs ───────────────────────────────────────────────────────────────

SERVICE_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567890"
CMD_VEL_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567891"
E_STOP_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567892"
HEARTBEAT_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567893"

DEVICE_NAME = "CouchVision-BLE"


# ── ROS2 Node ────────────────────────────────────────────────────────────────


class BLEBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("ble_bridge")
        qos = QoSProfile(depth=5)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos)
        self._e_stop_pub = self.create_publisher(Bool, "/e_stop", qos)
        self._cmd_count = 0
        self._estop_count = 0

    def publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_vel_pub.publish(msg)
        self._cmd_count += 1
        if self._cmd_count % 100 == 1:
            self.get_logger().info(
                f"BLE cmd_vel #{self._cmd_count}: lx={linear_x:.3f} az={angular_z:.3f}"
            )

    def publish_e_stop(self, stop: bool) -> None:
        msg = Bool()
        msg.data = stop
        self._e_stop_pub.publish(msg)
        self._estop_count += 1
        self.get_logger().info(f"BLE e_stop: {'STOP' if stop else 'RELEASE'}")


# ── BLE GATT Server ──────────────────────────────────────────────────────────


_ros_node: BLEBridgeNode | None = None


def _on_write(characteristic: BlessGATTCharacteristic, value: Any, **kwargs: Any) -> None:
    """Handle BLE GATT write requests."""
    uuid = str(characteristic.uuid).lower()
    data = bytes(value) if not isinstance(value, bytes) else value

    if uuid == CMD_VEL_UUID and len(data) == 8 and _ros_node is not None:
        linear_x, angular_z = struct.unpack("<ff", data)
        _ros_node.publish_cmd_vel(linear_x, angular_z)
    elif uuid == E_STOP_UUID and len(data) >= 1 and _ros_node is not None:
        _ros_node.publish_e_stop(data[0] != 0)
    else:
        logger.warning(f"Unknown write: uuid={uuid} len={len(data)}")


async def _run_ble_server() -> None:
    """Start the BLE GATT server and send heartbeat notifications."""
    gatt = {
        SERVICE_UUID: {
            CMD_VEL_UUID: {
                "Properties": (
                    GATTCharacteristicProperties.write
                    | GATTCharacteristicProperties.write_without_response
                ),
                "Permissions": GATTAttributePermissions.writeable,
                "Value": bytearray(8),
            },
            E_STOP_UUID: {
                "Properties": GATTCharacteristicProperties.write,
                "Permissions": GATTAttributePermissions.writeable,
                "Value": bytearray(1),
            },
            HEARTBEAT_UUID: {
                "Properties": (
                    GATTCharacteristicProperties.read
                    | GATTCharacteristicProperties.notify
                ),
                "Permissions": GATTAttributePermissions.readable,
                "Value": bytearray(1),
            },
        },
    }

    server = BlessServer(name=DEVICE_NAME)
    server.write_request_func = _on_write

    await server.add_gatt(gatt)
    await server.start()

    logger.info(f"BLE GATT server '{DEVICE_NAME}' advertising service {SERVICE_UUID}")

    heartbeat_seq = 0
    try:
        while True:
            await asyncio.sleep(0.5)
            heartbeat_seq = (heartbeat_seq + 1) % 256
            char = server.get_characteristic(HEARTBEAT_UUID)
            if char is not None:
                char.value = bytearray([heartbeat_seq])
                server.update_value(SERVICE_UUID, HEARTBEAT_UUID)
    except asyncio.CancelledError:
        pass
    finally:
        await server.stop()
        logger.info("BLE GATT server stopped")


# ── Entry point ──────────────────────────────────────────────────────────────


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )
    global _ros_node

    rclpy.init()
    _ros_node = BLEBridgeNode()

    # Spin ROS2 in a background thread so the asyncio loop can run BLE
    spin_thread = threading.Thread(target=rclpy.spin, args=(_ros_node,), daemon=True)
    spin_thread.start()

    logger.info("BLE bridge starting — ROS2 node active, launching GATT server...")

    try:
        asyncio.run(_run_ble_server())
    except KeyboardInterrupt:
        pass
    finally:
        _ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
