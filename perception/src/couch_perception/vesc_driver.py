"""VESC motor driver — Flipsky Dual FSESC 6.7 over USB serial.

Dual ESC: master (CAN ID 0) direct over USB, slave (CAN ID 1) via CAN forwarding.
Single motor for now; angular.z from /cmd_vel is ignored until second motor is wired.

Protocol reference: SELF_DRIVING_STACK.md § "VESC Communication Protocol"
Working reference: ~/bldc/spin_test.py on the Jetson (pyserial only, no pyvesc).
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import struct
import threading
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

if TYPE_CHECKING:
    import serial as serial_mod

try:
    import serial
except ImportError:
    serial = None  # type: ignore[assignment]

# STM32 Virtual COM Port (Flipsky FSESC)
_VESC_USB_VID = 0x0483
_VESC_USB_PID = 0x5740


def _find_vesc_port() -> str | None:
    """Auto-detect VESC serial port by USB vendor/product ID, then by probe."""
    if serial is None:
        return None
    # Fast path: match STM32 VCP by USB IDs
    try:
        from serial.tools.list_ports import comports

        for port in comports():
            if port.vid == _VESC_USB_VID and port.pid == _VESC_USB_PID:
                return port.device
    except Exception:
        pass
    # Fallback: probe each /dev/ttyACM* with COMM_GET_VALUES
    for path in sorted(glob.glob("/dev/ttyACM*")):
        try:
            with serial.Serial(path, 115200, timeout=0.1) as s:
                s.write(_build_packet(bytes([4])))  # COMM_GET_VALUES
                resp = s.read(256)
                if resp and _parse_response(resp) is not None:
                    return path
        except (serial.SerialException, OSError):
            continue
    return None


# ── VESC protocol constants ──────────────────────────────────────────────────

COMM_GET_VALUES = 4
COMM_SET_CURRENT = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM = 8

FAULT_NAMES = {
    0: "NONE",
    1: "OVER_VOLTAGE",
    2: "UNDER_VOLTAGE",
    3: "DRV",
    4: "ABS_OVER_CURRENT",
    5: "OVER_TEMP_FET",
    6: "OVER_TEMP_MOTOR",
}


# ── Packet framing ───────────────────────────────────────────────────────────


def _crc16(data: bytes) -> int:
    """CRC-CCITT (poly 0x1021, init 0x0000)."""
    crc = 0x0000
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def _build_packet(payload: bytes) -> bytes:
    """Short packet: [0x02][len:1][payload][crc16:2][0x03]."""
    crc = _crc16(payload)
    return (
        bytes([0x02, len(payload)]) + payload + crc.to_bytes(2, "big") + bytes([0x03])
    )


def _parse_response(buf: bytes) -> bytes | None:
    """Extract payload from VESC response, or None on invalid/corrupt frame."""
    if len(buf) < 5:
        return None
    # Short packet (0x02)
    if buf[0] == 0x02:
        length = buf[1]
        if len(buf) < length + 5:
            return None
        payload = buf[2 : 2 + length]
        crc_recv = struct.unpack(">H", buf[2 + length : 4 + length])[0]
        return payload if _crc16(payload) == crc_recv else None
    # Long packet (0x03)
    if buf[0] == 0x03:
        length = struct.unpack(">H", buf[1:3])[0]
        if len(buf) < length + 6:
            return None
        payload = buf[3 : 3 + length]
        crc_recv = struct.unpack(">H", buf[3 + length : 5 + length])[0]
        return payload if _crc16(payload) == crc_recv else None
    return None


# ── Command builders ─────────────────────────────────────────────────────────


def _cmd_get_values() -> bytes:
    return _build_packet(bytes([COMM_GET_VALUES]))


def _cmd_set_rpm(erpm: int) -> bytes:
    return _build_packet(bytes([COMM_SET_RPM]) + struct.pack(">i", erpm))


def _cmd_set_current(amps: float) -> bytes:
    return _build_packet(
        bytes([COMM_SET_CURRENT]) + struct.pack(">i", int(amps * 1000))
    )


# ── Telemetry ────────────────────────────────────────────────────────────────


@dataclass
class VescTelemetry:
    temp_fet: float = 0.0
    temp_motor: float = 0.0
    current_motor: float = 0.0
    current_input: float = 0.0
    duty_cycle: float = 0.0
    erpm: int = 0
    voltage_input: float = 0.0
    tachometer: int = 0
    tachometer_abs: int = 0
    fault_code: int = 0


def _parse_get_values(payload: bytes) -> VescTelemetry | None:
    """Parse COMM_GET_VALUES response. Layout from SELF_DRIVING_STACK.md."""
    if len(payload) < 54 or payload[0] != COMM_GET_VALUES:
        return None
    d = payload[1:]
    return VescTelemetry(
        temp_fet=struct.unpack(">h", d[0:2])[0] / 10.0,
        temp_motor=struct.unpack(">h", d[2:4])[0] / 10.0,
        current_motor=struct.unpack(">i", d[4:8])[0] / 100.0,
        current_input=struct.unpack(">i", d[8:12])[0] / 100.0,
        duty_cycle=struct.unpack(">h", d[20:22])[0] / 1000.0,
        erpm=struct.unpack(">i", d[22:26])[0],
        voltage_input=struct.unpack(">h", d[26:28])[0] / 10.0,
        tachometer=struct.unpack(">i", d[44:48])[0],
        tachometer_abs=struct.unpack(">i", d[48:52])[0],
        fault_code=d[52],
    )


# ── Kinematics ───────────────────────────────────────────────────────────────


@dataclass
class VescConfig:
    mode: str = "manual"
    target_rpm: int = 0
    max_rpm: int = 500
    wheel_radius: float = 0.1
    wheel_separation: float = 0.5
    pole_pairs: int = 7


def _twist_to_erpm(
    linear_x: float, angular_z: float, cfg: VescConfig
) -> tuple[int, int]:
    """Differential drive: Twist → (left_erpm, right_erpm)."""
    v_left = linear_x - (angular_z * cfg.wheel_separation / 2.0)
    v_right = linear_x + (angular_z * cfg.wheel_separation / 2.0)
    max_erpm = cfg.max_rpm * cfg.pole_pairs

    def to_erpm(v: float) -> int:
        mech_rpm = v / (2.0 * math.pi * cfg.wheel_radius) * 60.0
        return max(-max_erpm, min(max_erpm, int(mech_rpm * cfg.pole_pairs)))

    return to_erpm(v_left), to_erpm(v_right)


def _tachometer_to_distance(tach_delta: int, cfg: VescConfig) -> float:
    """Hall sensor counts → meters. 6 transitions per electrical revolution."""
    wheel_revs = tach_delta / (cfg.pole_pairs * 6.0)
    return wheel_revs * 2.0 * math.pi * cfg.wheel_radius


def _stamp_from_epoch(t: float) -> RosTime:
    sec = int(t)
    return RosTime(sec=sec, nanosec=int((t - sec) * 1e9))


# ── ROS2 Node ────────────────────────────────────────────────────────────────


class VescDriver(Node):
    def __init__(self, port: str = "auto", baud: int = 115200) -> None:
        super().__init__("vesc_driver")

        self._port = port
        self._baud = baud
        self._serial: serial_mod.Serial | None = None
        self._serial_lock = threading.Lock()
        self._connect()

        self._config = VescConfig()
        self._e_stopped: bool = True
        self._last_cmd_vel: Twist | None = None
        self._last_cmd_time: float = 0.0
        self._last_tachometer: int | None = None
        self._last_odom_time: float | None = None
        self._latest_telemetry: VescTelemetry | None = None
        self._error_count: int = 0
        self._reconnect_time: float = 0.0
        self._angular_warned: bool = False
        self._last_erpm_sent: int = 0

        qos = QoSProfile(depth=5)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, qos)
        self.create_subscription(Bool, "/e_stop", self._on_e_stop, qos)
        self.create_subscription(String, "/motor/config", self._on_config, qos)
        self._odom_pub = self.create_publisher(Odometry, "/wheel_odom", qos)
        self._status_pub = self.create_publisher(String, "/motor/status", qos)

        self.create_timer(0.05, self._command_loop)
        self.create_timer(0.1, self._telemetry_loop)
        self.create_timer(1.0, self._publish_status)

        state = "connected" if self._serial else "scanning (will retry every 5s)"
        self.get_logger().info(f"VESC driver started — port={port} {state}")

    # ── Serial I/O ────────────────────────────────────────────────────────

    def _connect(self) -> None:
        if serial is None:
            return
        port = self._port
        if port == "auto":
            port_found = _find_vesc_port()
            if port_found is None:
                return
            port = port_found
        try:
            self._serial = serial.Serial(port, self._baud, timeout=0.05)
            self.get_logger().info(f"Connected to VESC on {port}")
        except (serial.SerialException, OSError):
            self._serial = None

    def _try_reconnect(self) -> None:
        now = time.monotonic()
        if now - self._reconnect_time < 5.0:
            return
        self._reconnect_time = now
        self._connect()

    def _send(self, packet: bytes) -> None:
        """Send a command packet. Lock must NOT be held by caller."""
        with self._serial_lock:
            if self._serial is None:
                return
            try:
                self._serial.write(packet)
            except (serial.SerialException, OSError):
                self._serial = None

    def _transact(self, packet: bytes, size: int = 256) -> bytes:
        """Send a command and read the response atomically.

        Holds the serial lock for the entire exchange to prevent
        interleaved writes from the command loop.
        """
        with self._serial_lock:
            if self._serial is None:
                return b""
            try:
                self._serial.write(packet)
                return self._serial.read(size)
            except (serial.SerialException, OSError):
                self._serial = None
                return b""

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_vel = msg
        self._last_cmd_time = time.monotonic()

    def _on_e_stop(self, msg: Bool) -> None:
        if msg.data != self._e_stopped:
            self._e_stopped = msg.data
            self.get_logger().info(f"E-STOP {'ENGAGED' if msg.data else 'RELEASED'}")
            if msg.data:
                self._send(_cmd_set_current(0.0))

    def _on_config(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if "mode" in data and data["mode"] in ("manual", "nav2"):
            self._config.mode = data["mode"]
        if "target_rpm" in data:
            self._config.target_rpm = int(data["target_rpm"])
        if "max_rpm" in data:
            self._config.max_rpm = max(0, int(data["max_rpm"]))
        if "wheel_radius" in data:
            self._config.wheel_radius = float(data["wheel_radius"])
        if "wheel_separation" in data:
            self._config.wheel_separation = float(data["wheel_separation"])
        if "pole_pairs" in data:
            self._config.pole_pairs = int(data["pole_pairs"])

    # ── Command loop (50ms) ───────────────────────────────────────────────

    def _command_loop(self) -> None:
        if self._serial is None:
            self._try_reconnect()
            return

        if self._e_stopped:
            self._send(_cmd_set_current(0.0))
            self._last_erpm_sent = 0
            return

        if self._config.mode == "manual":
            max_erpm = self._config.max_rpm * self._config.pole_pairs
            target_erpm = self._config.target_rpm * self._config.pole_pairs
            target_erpm = max(-max_erpm, min(max_erpm, target_erpm))
            self._send(_cmd_set_rpm(target_erpm))
            self._last_erpm_sent = target_erpm

        elif self._config.mode == "nav2":
            if (
                self._last_cmd_vel is None
                or time.monotonic() - self._last_cmd_time > 0.5
            ):
                self._send(_cmd_set_rpm(0))
                self._last_erpm_sent = 0
                return
            if abs(self._last_cmd_vel.angular.z) > 0.01 and not self._angular_warned:
                self.get_logger().warning("angular.z ignored — single motor")
                self._angular_warned = True
            left_erpm, _ = _twist_to_erpm(
                self._last_cmd_vel.linear.x, 0.0, self._config
            )
            self._send(_cmd_set_rpm(left_erpm))
            self._last_erpm_sent = left_erpm

    # ── Telemetry loop (100ms) ────────────────────────────────────────────

    def _telemetry_loop(self) -> None:
        if self._serial is None:
            return

        # Atomic send+read prevents command loop from interleaving writes
        raw = self._transact(_cmd_get_values())
        if not raw:
            return

        payload = _parse_response(raw)
        if payload is None:
            self._error_count += 1
            return

        telemetry = _parse_get_values(payload)
        if telemetry is None:
            self._error_count += 1
            return

        self._latest_telemetry = telemetry
        self._publish_odom(telemetry)

        # Re-send motor command to avoid VESC 1000ms timeout
        if not self._e_stopped and self._last_erpm_sent != 0:
            self._send(_cmd_set_rpm(self._last_erpm_sent))

    def _publish_odom(self, telemetry: VescTelemetry) -> None:
        now = time.time()
        tach = telemetry.tachometer

        if self._last_tachometer is not None and self._last_odom_time is not None:
            dt = now - self._last_odom_time
            if dt > 0:
                distance = _tachometer_to_distance(
                    tach - self._last_tachometer, self._config
                )
                msg = Odometry()
                msg.header.stamp = _stamp_from_epoch(now)
                msg.header.frame_id = "odom"
                msg.child_frame_id = "base_link"
                msg.twist.twist.linear.x = distance / dt
                msg.twist.covariance[0] = 0.01  # linear.x
                msg.twist.covariance[35] = 99.0  # angular.z (no steering data yet)
                self._odom_pub.publish(msg)

        self._last_tachometer = tach
        self._last_odom_time = now

    # ── Status (1Hz) ──────────────────────────────────────────────────────

    def _publish_status(self) -> None:
        t = self._latest_telemetry
        zero = VescTelemetry()
        v = t or zero
        status = {
            "connected": self._serial is not None,
            "mode": self._config.mode,
            "e_stopped": self._e_stopped,
            "rpm": v.erpm // self._config.pole_pairs,
            "erpm": v.erpm,
            "temp_fet": v.temp_fet,
            "temp_motor": v.temp_motor,
            "current_motor": v.current_motor,
            "current_input": v.current_input,
            "voltage_input": v.voltage_input,
            "duty_cycle": v.duty_cycle,
            "tachometer": v.tachometer,
            "fault_code": v.fault_code,
            "fault_name": FAULT_NAMES.get(v.fault_code, "UNKNOWN"),
            "target_rpm": self._config.target_rpm,
            "max_rpm": self._config.max_rpm,
            "cmd_vel_age_ms": (
                int((time.monotonic() - self._last_cmd_time) * 1000)
                if self._last_cmd_vel is not None
                else None
            ),
            "errors": self._error_count,
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    # ── Lifecycle ─────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        self._send(_cmd_set_current(0.0))
        with self._serial_lock:
            if self._serial is not None:
                self._serial.close()


# ── Entry point ──────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(description="VESC motor driver for ROS2")
    parser.add_argument("--port", default="auto", help="Serial port or 'auto' to scan")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    rclpy.init()
    node = VescDriver(port=args.port, baud=args.baud)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
