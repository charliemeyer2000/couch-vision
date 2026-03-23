"""BLE client bridge — connects to the Mac BLE peripheral and publishes to ROS2.

The Mac runs a BLE GATT server (bless on macOS). This script runs on the Jetson
as a BLE central (bleak) and subscribes to notifications for cmd_vel and e_stop.

When a notification arrives, it publishes the corresponding ROS2 message.

GATT service layout (on the Mac peripheral):
  CMD_VEL   (notify)  8 bytes: 2x float32 LE (linear_x, angular_z)
  E_STOP    (notify)  1 byte:  0x00=release, 0x01=stop
  HEARTBEAT (notify)  1 byte:  wrapping counter, 500ms interval
"""

from __future__ import annotations

import asyncio
import logging
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from bleak import BleakClient, BleakScanner

logger = logging.getLogger(__name__)

# Must match ble_relay.py on Mac
SERVICE_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567890"
CMD_VEL_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567891"
E_STOP_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567892"
HEARTBEAT_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567893"

DEVICE_NAME = "CouchVision-BLE"
SCAN_TIMEOUT_S = 10.0
RECONNECT_DELAY_S = 2.0
_SERVICE_UUID_LOWER = SERVICE_UUID.lower()


# -- ROS2 Node -----------------------------------------------------------------


class BLEBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("ble_bridge")
        qos = QoSProfile(depth=5)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos)
        self._e_stop_pub = self.create_publisher(Bool, "/e_stop", qos)
        self._cmd_count = 0

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
        self.get_logger().info(f"BLE e_stop: {'STOP' if stop else 'RELEASE'}")


# -- BLE Client ----------------------------------------------------------------


async def _connect_and_subscribe(node: BLEBridgeNode) -> None:
    """Scan for the Mac BLE peripheral, connect, and subscribe to notifications."""

    def on_cmd_vel(_char_handle: int, data: bytearray) -> None:
        if len(data) == 8:
            linear_x, angular_z = struct.unpack("<ff", data)
            node.publish_cmd_vel(linear_x, angular_z)

    def on_e_stop(_char_handle: int, data: bytearray) -> None:
        if len(data) >= 1:
            node.publish_e_stop(data[0] != 0)

    def on_heartbeat(_char_handle: int, data: bytearray) -> None:
        logger.debug(f"Heartbeat: seq={data[0]}")

    def _scan_filter(d, ad) -> bool:
        return (
            _SERVICE_UUID_LOWER in {s.lower() for s in (ad.service_uuids or [])}
            or (d.name or "").startswith(DEVICE_NAME)
        )

    while True:
        try:
            logger.info("Scanning for CouchVision BLE peripheral...")
            device = await BleakScanner.find_device_by_filter(
                _scan_filter, timeout=SCAN_TIMEOUT_S,
            )
            if device is None:
                logger.warning(
                    f"No device found (looked for UUID {SERVICE_UUID} or name "
                    f"'{DEVICE_NAME}*'), retrying in {RECONNECT_DELAY_S}s..."
                )
                await asyncio.sleep(RECONNECT_DELAY_S)
                continue

            logger.info(f"Found peripheral: {device.name} ({device.address})")

            disconnected = asyncio.Event()
            async with BleakClient(
                device, disconnected_callback=lambda _c: disconnected.set()
            ) as client:
                svcs = client.services
                if not svcs.get_service(SERVICE_UUID):
                    logger.warning(
                        f"Device {device.name} connected but missing service "
                        f"{SERVICE_UUID}, retrying..."
                    )
                    await asyncio.sleep(RECONNECT_DELAY_S)
                    continue

                logger.info(
                    f"Connected to {device.name} — subscribing to notifications"
                )

                await client.start_notify(CMD_VEL_UUID, on_cmd_vel)
                await client.start_notify(E_STOP_UUID, on_e_stop)
                await client.start_notify(HEARTBEAT_UUID, on_heartbeat)

                logger.info("BLE bridge active — forwarding to ROS2")
                await disconnected.wait()
                logger.warning("BLE peripheral disconnected")

        except Exception as e:
            logger.error(f"BLE connection error: {e}")

        logger.info(f"Reconnecting in {RECONNECT_DELAY_S}s...")
        await asyncio.sleep(RECONNECT_DELAY_S)


# -- Entry point ---------------------------------------------------------------


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )
    rclpy.init()
    node = BLEBridgeNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    logger.info("BLE bridge starting — ROS2 node active, scanning for Mac peripheral...")

    try:
        asyncio.run(_connect_and_subscribe(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
