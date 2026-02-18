"""VESC motor driver node for differential drive control.

Communicates with a Flipsky Dual FSESC 6.7 over USB serial (/dev/ttyACM0).
The dual ESC has two independent controllers on one PCB:
  - Master (CAN ID 0): reached directly over USB
  - Slave (CAN ID 1): reached via CAN forwarding over the same USB

Subscribes to /cmd_vel, /e_stop, /motor/config.
Publishes /wheel_odom, /motor/status.
"""

import argparse
import json
import math
import struct
import threading
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from builtin_interfaces.msg import Time as RosTime

try:
    import serial
except ImportError:
    serial = None  # type: ignore[assignment]


# ── VESC command IDs ─────────────────────────────────────────────────────────

COMM_GET_VALUES = 4
COMM_SET_DUTY = 5
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


# ── CRC & packet layer ──────────────────────────────────────────────────────

def _crc16(data: bytes) -> int:
    """CRC-CCITT (poly 0x1021, init 0x0000) over payload bytes."""
    crc = 0x0000
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def _build_packet(payload: bytes) -> bytes:
    """Build VESC short packet: [0x02][len:1][payload][crc16:2][0x03]."""
    crc = _crc16(payload)
    return bytes([0x02, len(payload)]) + payload + crc.to_bytes(2, "big") + bytes([0x03])


def _parse_response(buf: bytes) -> bytes | None:
    """Extract payload from a VESC response. Returns None on invalid frame."""
    if len(buf) < 5:
        return None
    if buf[0] == 0x02:
        length = buf[1]
        if len(buf) < length + 5:
            return None
        payload = buf[2 : 2 + length]
        crc_recv = struct.unpack(">H", buf[2 + length : 4 + length])[0]
        if _crc16(payload) != crc_recv:
            return None
        return payload
    if buf[0] == 0x03:
        length = struct.unpack(">H", buf[1:3])[0]
        if len(buf) < length + 6:
            return None
        payload = buf[3 : 3 + length]
        crc_recv = struct.unpack(">H", buf[3 + length : 5 + length])[0]
        if _crc16(payload) != crc_recv:
            return None
        return payload
    return None


# ── Command builders ─────────────────────────────────────────────────────────

def _cmd_get_values() -> bytes:
    return _build_packet(bytes([COMM_GET_VALUES]))


def _cmd_set_rpm(erpm: int) -> bytes:
    return _build_packet(bytes([COMM_SET_RPM]) + struct.pack(">i", erpm))


def _cmd_set_current(amps: float) -> bytes:
    return _build_packet(bytes([COMM_SET_CURRENT]) + struct.pack(">i", int(amps * 1000)))


def _cmd_set_current_brake(amps: float) -> bytes:
    return _build_packet(bytes([COMM_SET_CURRENT_BRAKE]) + struct.pack(">i", int(amps * 1000)))


def _cmd_set_duty(duty: float) -> bytes:
    return _build_packet(bytes([COMM_SET_DUTY]) + struct.pack(">i", int(duty * 100000)))


# ── Telemetry parser ─────────────────────────────────────────────────────────

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
    """Parse COMM_GET_VALUES response (54+ bytes after cmd byte)."""
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
    linear_x: float, angular_z: float, config: VescConfig
) -> tuple[int, int]:
    """Convert Twist to (left_erpm, right_erpm) for differential drive."""
    v_left = linear_x - (angular_z * config.wheel_separation / 2.0)
    v_right = linear_x + (angular_z * config.wheel_separation / 2.0)

    def vel_to_erpm(v: float) -> int:
        wheel_rps = v / (2.0 * math.pi * config.wheel_radius)
        mech_rpm = wheel_rps * 60.0
        erpm = int(mech_rpm * config.pole_pairs)
        max_erpm = config.max_rpm * config.pole_pairs
        return max(-max_erpm, min(max_erpm, erpm))

    return vel_to_erpm(v_left), vel_to_erpm(v_right)


def _tachometer_to_distance(tach_delta: int, config: VescConfig) -> float:
    """Convert tachometer count delta to linear distance (meters)."""
    wheel_revs = tach_delta / (config.pole_pairs * 6.0)
    return wheel_revs * 2.0 * math.pi * config.wheel_radius


# ── ROS2 Node ────────────────────────────────────────────────────────────────

class VescDriver(Node):
    """ROS2 node for VESC motor control and telemetry."""

    def __init__(self, port: str = "/dev/ttyACM0", baud: int = 115200):
        super().__init__("vesc_driver")

        self._port = port
        self._baud = baud
        self._serial: object | None = None
        self._serial_lock = threading.Lock()
        self._connect()

        # State
        self._config = VescConfig()
        self._e_stopped = True  # safety: start stopped
        self._last_cmd_vel: Twist | None = None
        self._last_cmd_time: float = 0.0
        self._last_tachometer: int | None = None
        self._last_odom_time: float | None = None
        self._latest_telemetry: VescTelemetry | None = None
        self._error_count = 0
        self._reconnect_time: float = 0.0
        self._angular_warned = False
        self._last_erpm_sent: int = 0

        # Subscribers
        qos = QoSProfile(depth=5)
        self._cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, qos,
        )
        self._e_stop_sub = self.create_subscription(
            Bool, "/e_stop", self._on_e_stop, qos,
        )
        self._config_sub = self.create_subscription(
            String, "/motor/config", self._on_config, qos,
        )

        # Publishers
        self._odom_pub = self.create_publisher(Odometry, "/wheel_odom", qos)
        self._status_pub = self.create_publisher(String, "/motor/status", qos)

        # Timers
        self._command_timer = self.create_timer(0.05, self._command_loop)
        self._telemetry_timer = self.create_timer(0.1, self._telemetry_loop)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        connected = "connected" if self._serial else "NOT connected (will retry)"
        self.get_logger().info(f"VESC driver started — {port} {connected}")

    # ── Serial ───────────────────────────────────────────────────────────

    def _connect(self) -> None:
        if serial is None:
            self.get_logger().warning("pyserial not installed — running without hardware")
            return
        try:
            self._serial = serial.Serial(self._port, self._baud, timeout=0.05)
            self.get_logger().info(f"Serial opened: {self._port}")
        except (serial.SerialException, OSError) as e:
            self._serial = None
            self.get_logger().warning(f"Serial {self._port} not available: {e}")

    def _try_reconnect(self) -> None:
        now = time.monotonic()
        if now - self._reconnect_time < 5.0:
            return
        self._reconnect_time = now
        self._connect()

    def _send(self, packet: bytes) -> None:
        with self._serial_lock:
            if self._serial is None:
                return
            try:
                self._serial.write(packet)  # type: ignore[union-attr]
            except (serial.SerialException, OSError):
                self.get_logger().error("Serial write failed — disconnected")
                self._serial = None

    def _read(self, size: int = 256) -> bytes:
        with self._serial_lock:
            if self._serial is None:
                return b""
            try:
                return self._serial.read(size)  # type: ignore[union-attr]
            except (serial.SerialException, OSError):
                self.get_logger().error("Serial read failed — disconnected")
                self._serial = None
                return b""

    # ── Callbacks ────────────────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_vel = msg
        self._last_cmd_time = time.monotonic()

    def _on_e_stop(self, msg: Bool) -> None:
        if msg.data != self._e_stopped:
            self._e_stopped = msg.data
            self.get_logger().info(f"E-STOP {'ENGAGED' if msg.data else 'RELEASED'}")
            if msg.data:
                self._send_brake()

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

    # ── Command loop (50ms) ──────────────────────────────────────────────

    def _command_loop(self) -> None:
        if self._serial is None:
            self._try_reconnect()
            return

        if self._e_stopped:
            self._send_brake()
            self._last_erpm_sent = 0
            return

        if self._config.mode == "manual":
            max_erpm = self._config.max_rpm * self._config.pole_pairs
            target_erpm = self._config.target_rpm * self._config.pole_pairs
            target_erpm = max(-max_erpm, min(max_erpm, target_erpm))
            self._send(_cmd_set_rpm(target_erpm))
            self._last_erpm_sent = target_erpm

        elif self._config.mode == "nav2":
            if self._last_cmd_vel is None:
                self._send(_cmd_set_rpm(0))
                self._last_erpm_sent = 0
                return
            age = time.monotonic() - self._last_cmd_time
            if age > 0.5:
                self._send(_cmd_set_rpm(0))
                self._last_erpm_sent = 0
                return
            # Single motor: only use linear.x; warn once about angular.z
            angular_z = self._last_cmd_vel.angular.z
            if abs(angular_z) > 0.01 and not self._angular_warned:
                self.get_logger().warning(
                    "angular.z ignored — single motor, no differential steering"
                )
                self._angular_warned = True
            # With one motor, compute as if both wheels get the same speed
            left_erpm, _right_erpm = _twist_to_erpm(
                self._last_cmd_vel.linear.x, 0.0, self._config,
            )
            self._send(_cmd_set_rpm(left_erpm))
            self._last_erpm_sent = left_erpm

    # ── Telemetry loop (100ms) ───────────────────────────────────────────

    def _telemetry_loop(self) -> None:
        if self._serial is None:
            return

        self._send(_cmd_get_values())
        time.sleep(0.02)  # VESC needs ~20ms to respond
        raw = self._read(256)
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

        # Re-send last command immediately to avoid VESC timeout gap
        if not self._e_stopped and self._last_erpm_sent != 0:
            self._send(_cmd_set_rpm(self._last_erpm_sent))

    def _publish_odom(self, telemetry: VescTelemetry) -> None:
        now = time.time()
        tach = telemetry.tachometer

        if self._last_tachometer is not None and self._last_odom_time is not None:
            dt = now - self._last_odom_time
            if dt > 0:
                tach_delta = tach - self._last_tachometer
                distance = _tachometer_to_distance(tach_delta, self._config)
                velocity = distance / dt

                msg = Odometry()
                msg.header.frame_id = "odom"
                msg.child_frame_id = "base_link"
                sec = int(now)
                msg.header.stamp = RosTime(sec=sec, nanosec=int((now - sec) * 1e9))
                msg.twist.twist.linear.x = velocity
                # High covariance for angular (no dual-motor steering info yet)
                msg.twist.covariance[0] = 0.01  # linear.x variance
                msg.twist.covariance[35] = 99.0  # angular.z variance (unknown)
                self._odom_pub.publish(msg)

        self._last_tachometer = tach
        self._last_odom_time = now

    # ── Status publish (1Hz) ─────────────────────────────────────────────

    def _publish_status(self) -> None:
        t = self._latest_telemetry
        mech_rpm = t.erpm / self._config.pole_pairs if t else 0
        cmd_vel_age = None
        if self._last_cmd_vel is not None:
            cmd_vel_age = int((time.monotonic() - self._last_cmd_time) * 1000)

        status = {
            "connected": self._serial is not None,
            "mode": self._config.mode,
            "e_stopped": self._e_stopped,
            "rpm": int(mech_rpm),
            "erpm": t.erpm if t else 0,
            "temp_fet": t.temp_fet if t else 0.0,
            "temp_motor": t.temp_motor if t else 0.0,
            "current_motor": t.current_motor if t else 0.0,
            "current_input": t.current_input if t else 0.0,
            "voltage_input": t.voltage_input if t else 0.0,
            "duty_cycle": t.duty_cycle if t else 0.0,
            "tachometer": t.tachometer if t else 0,
            "fault_code": t.fault_code if t else 0,
            "fault_name": FAULT_NAMES.get(t.fault_code, "UNKNOWN") if t else "NONE",
            "target_rpm": self._config.target_rpm,
            "max_rpm": self._config.max_rpm,
            "cmd_vel_age_ms": cmd_vel_age,
            "errors": self._error_count,
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    # ── Helpers ──────────────────────────────────────────────────────────

    def _send_brake(self) -> None:
        self._send(_cmd_set_current(0.0))

    def shutdown(self) -> None:
        self._send_brake()
        with self._serial_lock:
            if self._serial is not None:
                try:
                    self._serial.close()  # type: ignore[union-attr]
                except Exception:
                    pass


# ── CLI entry point ──────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="VESC motor driver for ROS2")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
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
