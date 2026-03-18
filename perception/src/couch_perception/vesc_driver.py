"""VESC motor driver — Flipsky Dual FSESC 6.7 over USB serial.

Dual ESC: master (right wheel, inverted) direct over USB,
slave (left wheel, CAN ID 19) via CAN forwarding.

Uses COMM_SET_RPM for both motors. Differential drive from /cmd_vel
maps to left/right wheel ERPM targets with proper inversion.

Protocol reference: SELF_DRIVING_STACK.md § "VESC Communication Protocol"
Known issue: VESC firmware comm_usb.c has a sticky was_timeout flag that
can kill USB comms. Keep serial timeouts ≤100ms and read promptly.
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

# Hardware defaults (Flipsky Dual FSESC 6.7)
DEFAULT_SLAVE_CAN_ID = 19
DEFAULT_POLE_PAIRS = 7
DEFAULT_WHEEL_RADIUS = 0.0415  # 83mm diameter from vesc_mcconf_24v.xml
DEFAULT_WHEEL_SEPARATION = 0.5
DEFAULT_GEAR_RATIO = 3


def _find_vesc_port() -> str | None:
    """Auto-detect VESC serial port by USB vendor/product ID, then by probe."""
    if serial is None:
        return None
    try:
        from serial.tools.list_ports import comports

        for port in comports():
            if port.vid == _VESC_USB_VID and port.pid == _VESC_USB_PID:
                return port.device
    except Exception:
        pass
    for pattern in ["/dev/ttyACM*", "/dev/cu.usbmodem*"]:
        for path in sorted(glob.glob(pattern)):
            try:
                with serial.Serial(path, 115200, timeout=0.1) as s:
                    s.write(_build_packet(bytes([COMM_GET_VALUES])))
                    resp = s.read(256)
                    if resp and _parse_response(resp) is not None:
                        return path
            except (serial.SerialException, OSError):
                continue
    return None


# ── VESC protocol constants ──────────────────────────────────────────────────

COMM_GET_VALUES = 4
COMM_SET_CURRENT = 6
COMM_SET_RPM = 8
COMM_FORWARD_CAN = 34

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
    crc = 0x0000
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def _build_packet(payload: bytes) -> bytes:
    crc = _crc16(payload)
    return (
        bytes([0x02, len(payload)]) + payload + crc.to_bytes(2, "big") + bytes([0x03])
    )


def _parse_response(buf: bytes) -> bytes | None:
    if len(buf) < 5:
        return None
    if buf[0] == 0x02:
        length = buf[1]
        if len(buf) < length + 5:
            return None
        payload = buf[2 : 2 + length]
        crc_recv = struct.unpack(">H", buf[2 + length : 4 + length])[0]
        return payload if _crc16(payload) == crc_recv else None
    if buf[0] == 0x03:
        length = struct.unpack(">H", buf[1:3])[0]
        if len(buf) < length + 6:
            return None
        payload = buf[3 : 3 + length]
        crc_recv = struct.unpack(">H", buf[3 + length : 5 + length])[0]
        return payload if _crc16(payload) == crc_recv else None
    return None


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
    wheel_radius: float = DEFAULT_WHEEL_RADIUS
    wheel_separation: float = DEFAULT_WHEEL_SEPARATION
    pole_pairs: int = DEFAULT_POLE_PAIRS
    gear_ratio: int = DEFAULT_GEAR_RATIO
    slave_can_id: int = DEFAULT_SLAVE_CAN_ID
    invert_master: bool = True
    invert_slave: bool = False
    ramp_up_rpm_s: int = 500  # acceleration rate, RPM/s (0 = no limit)
    ramp_down_rpm_s: int = 500  # deceleration rate, RPM/s (0 = no limit)
    stop_rpm: int = 0  # below this RPM, coast instead of PID hold (0 = disabled)
    max_linear_vel: float = 0.0  # clamp cmd_vel linear.x (0 = no limit)
    max_angular_vel: float = 0.0  # clamp cmd_vel angular.z (0 = no limit)


def _twist_to_erpm(
    linear_x: float, angular_z: float, cfg: VescConfig
) -> tuple[int, int]:
    """Differential drive: Twist → (left_erpm, right_erpm).

    Returns ERPM values with inversion already applied for each motor.
    Master = right wheel, slave = left wheel.
    """
    v_left = linear_x - (angular_z * cfg.wheel_separation / 2.0)
    v_right = linear_x + (angular_z * cfg.wheel_separation / 2.0)
    max_erpm = cfg.max_rpm * cfg.pole_pairs

    def to_erpm(v: float) -> int:
        wheel_rpm = v / (2.0 * math.pi * cfg.wheel_radius) * 60.0
        motor_erpm = int(wheel_rpm * cfg.gear_ratio * cfg.pole_pairs)
        return max(-max_erpm, min(max_erpm, motor_erpm))

    left_erpm = to_erpm(v_left)
    right_erpm = to_erpm(v_right)

    # Apply per-motor inversion: master=right, slave=left
    master_erpm = -right_erpm if cfg.invert_master else right_erpm
    slave_erpm = -left_erpm if cfg.invert_slave else left_erpm

    return master_erpm, slave_erpm


def _tachometer_to_distance(tach_delta: int, cfg: VescConfig) -> float:
    """Hall sensor counts → meters. 6 transitions per electrical revolution."""
    motor_revs = tach_delta / (cfg.pole_pairs * 6.0)
    wheel_revs = motor_revs / cfg.gear_ratio
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

        # Per-motor state: None = master (USB), can_id = slave (CAN)
        self._last_tachometer: dict[int | None, int | None] = {None: None}
        self._last_odom_time: float | None = None
        self._latest_telemetry: dict[int | None, VescTelemetry | None] = {None: None}
        self._error_count: int = 0
        self._reconnect_time: float = 0.0
        self._commanded_erpm: dict[int | None, int] = {None: 0}
        self._telemetry_tick: int = 0

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

    @property
    def _motor_ids(self) -> list[int | None]:
        """Motor IDs to control: None=master, can_id=slave."""
        return [None, self._config.slave_can_id]

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
            self._serial = serial.Serial(port, self._baud, timeout=0.1, write_timeout=0.1)
            self.get_logger().info(f"Connected to VESC on {port}")
        except (serial.SerialException, OSError):
            self._serial = None

    def _try_reconnect(self) -> None:
        now = time.monotonic()
        if now - self._reconnect_time < 5.0:
            return
        self._reconnect_time = now
        self._connect()

    def _send(self, payload: bytes, can_id: int | None = None) -> None:
        """Fire-and-forget command. can_id=None for master, int for CAN slave."""
        if can_id is not None:
            payload = bytes([COMM_FORWARD_CAN, can_id]) + payload
        packet = _build_packet(payload)
        with self._serial_lock:
            if self._serial is None:
                return
            try:
                self._serial.write(packet)
            except (serial.SerialException, OSError):
                self._serial = None

    def _read_packet(self, timeout: float = 0.1) -> bytes:
        """Read a complete VESC packet, respecting the USB timeout budget."""
        buf = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._serial is None:
                return b""
            avail = self._serial.in_waiting
            if avail:
                buf += self._serial.read(avail)
                if len(buf) >= 5 and buf[0] == 0x02:
                    expected = buf[1] + 5
                    if len(buf) >= expected:
                        return buf[:expected]
                if len(buf) >= 6 and buf[0] == 0x03:
                    plen = struct.unpack(">H", buf[1:3])[0]
                    expected = plen + 6
                    if len(buf) >= expected:
                        return buf[:expected]
            else:
                time.sleep(0.001)
        return buf

    def _transact(self, payload: bytes, can_id: int | None = None) -> VescTelemetry | None:
        """Send COMM_GET_VALUES and parse the response. Thread-safe."""
        if can_id is not None:
            full_payload = bytes([COMM_FORWARD_CAN, can_id]) + payload
            timeout = 0.08  # CAN adds latency
        else:
            full_payload = payload
            timeout = 0.05

        with self._serial_lock:
            if self._serial is None:
                return None
            try:
                self._serial.reset_input_buffer()
                self._serial.write(_build_packet(full_payload))
            except (serial.SerialException, OSError):
                self._serial = None
                return None

        raw = self._read_packet(timeout)
        if not raw:
            return None
        resp = _parse_response(raw)
        if resp is None:
            return None
        return _parse_get_values(resp)

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_vel = msg
        self._last_cmd_time = time.monotonic()

    def _on_e_stop(self, msg: Bool) -> None:
        if msg.data != self._e_stopped:
            self._e_stopped = msg.data
            self.get_logger().info(f"E-STOP {'ENGAGED' if msg.data else 'RELEASED'}")
            if msg.data:
                for mid in self._motor_ids:
                    self._send(
                        bytes([COMM_SET_CURRENT]) + struct.pack(">i", 0),
                        can_id=mid,
                    )

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
        if "invert_master" in data:
            self._config.invert_master = bool(data["invert_master"])
        if "invert_slave" in data:
            self._config.invert_slave = bool(data["invert_slave"])
        if "max_ramp_rpm_s" in data:
            v = max(0, int(data["max_ramp_rpm_s"]))
            self._config.ramp_up_rpm_s = v
            self._config.ramp_down_rpm_s = v
        if "ramp_up_rpm_s" in data:
            self._config.ramp_up_rpm_s = max(0, int(data["ramp_up_rpm_s"]))
        if "ramp_down_rpm_s" in data:
            self._config.ramp_down_rpm_s = max(0, int(data["ramp_down_rpm_s"]))
        if "stop_rpm" in data:
            self._config.stop_rpm = max(0, int(data["stop_rpm"]))
        if "max_linear_vel" in data:
            self._config.max_linear_vel = max(0.0, float(data["max_linear_vel"]))
        if "max_angular_vel" in data:
            self._config.max_angular_vel = max(0.0, float(data["max_angular_vel"]))

    # ── Command loop (50ms / 20Hz) ───────────────────────────────────────

    _COMMAND_PERIOD_S = 0.05

    def _ramp_erpm(self, motor_id: int | None, target: int) -> int:
        """Slew-rate limit ERPM toward target to prevent torque spikes."""
        current = self._commanded_erpm.get(motor_id, 0)
        delta = target - current
        if delta == 0:
            return target

        # Pick rate: accelerating (delta same sign as current) vs decelerating
        if current == 0 or (delta > 0) == (current > 0):
            ramp_rpm_s = self._config.ramp_up_rpm_s
        else:
            ramp_rpm_s = self._config.ramp_down_rpm_s

        if ramp_rpm_s <= 0:
            return target

        max_step = max(
            1, int(ramp_rpm_s * self._config.pole_pairs * self._COMMAND_PERIOD_S)
        )
        if abs(delta) <= max_step:
            return target
        return current + (max_step if delta > 0 else -max_step)

    def _compute_target_erpm(self) -> tuple[int, int]:
        """Compute (master_erpm, slave_erpm) with inversion applied."""
        if self._e_stopped:
            return 0, 0
        if self._config.mode == "manual":
            max_erpm = self._config.max_rpm * self._config.pole_pairs
            target = self._config.target_rpm * self._config.pole_pairs
            clamped = max(-max_erpm, min(max_erpm, target))
            # Manual mode: same RPM to both, apply inversion
            m = -clamped if self._config.invert_master else clamped
            s = -clamped if self._config.invert_slave else clamped
            return m, s
        if self._config.mode == "nav2":
            if (
                self._last_cmd_vel is None
                or time.monotonic() - self._last_cmd_time > 0.5
            ):
                return 0, 0
            linear_x = self._last_cmd_vel.linear.x
            angular_z = self._last_cmd_vel.angular.z
            if self._config.max_linear_vel > 0:
                linear_x = max(-self._config.max_linear_vel, min(self._config.max_linear_vel, linear_x))
            if self._config.max_angular_vel > 0:
                angular_z = max(-self._config.max_angular_vel, min(self._config.max_angular_vel, angular_z))
            return _twist_to_erpm(linear_x, angular_z, self._config)
        return 0, 0

    def _command_loop(self) -> None:
        master_target, slave_target = self._compute_target_erpm()

        if self._serial is None:
            # Motor physically stopped — reset ramp so we start from 0 on reconnect
            self._commanded_erpm[None] = 0
            self._commanded_erpm[self._config.slave_can_id] = 0
            self._try_reconnect()
            return

        if self._e_stopped:
            # Bypass ramp — immediate zero current for safety
            self._commanded_erpm[None] = 0
            self._commanded_erpm[self._config.slave_can_id] = 0
            zero_current = bytes([COMM_SET_CURRENT]) + struct.pack(">i", 0)
            self._send(zero_current)
            self._send(zero_current, can_id=self._config.slave_can_id)
            return

        master_erpm = self._ramp_erpm(None, master_target)
        slave_erpm = self._ramp_erpm(self._config.slave_can_id, slave_target)

        # Stop threshold: coast with zero current instead of PID hold at low RPM
        stop_erpm = self._config.stop_rpm * self._config.pole_pairs
        if stop_erpm > 0:
            if abs(master_target) <= stop_erpm and abs(master_erpm) <= stop_erpm:
                master_erpm = 0
            if abs(slave_target) <= stop_erpm and abs(slave_erpm) <= stop_erpm:
                slave_erpm = 0

        self._commanded_erpm[None] = master_erpm
        self._commanded_erpm[self._config.slave_can_id] = slave_erpm

        zero_current = bytes([COMM_SET_CURRENT]) + struct.pack(">i", 0)
        if master_erpm == 0:
            self._send(zero_current)
        else:
            self._send(bytes([COMM_SET_RPM]) + struct.pack(">i", master_erpm))
        if slave_erpm == 0:
            self._send(zero_current, can_id=self._config.slave_can_id)
        else:
            self._send(
                bytes([COMM_SET_RPM]) + struct.pack(">i", slave_erpm),
                can_id=self._config.slave_can_id,
            )

    # ── Telemetry loop (100ms / 10Hz) ────────────────────────────────────

    def _telemetry_loop(self) -> None:
        if self._serial is None:
            return

        self._telemetry_tick += 1

        # Master: every tick (10Hz)
        master_telem = self._transact(bytes([COMM_GET_VALUES]))
        if master_telem is not None:
            self._latest_telemetry[None] = master_telem
        else:
            self._error_count += 1

        # Slave: every 2nd tick (5Hz) to reduce USB bus contention
        slave_id = self._config.slave_can_id
        if self._telemetry_tick % 2 == 0:
            slave_telem = self._transact(bytes([COMM_GET_VALUES]), can_id=slave_id)
            if slave_telem is not None:
                self._latest_telemetry[slave_id] = slave_telem

        self._publish_odom()

        # Re-send motor commands to avoid VESC 1000ms timeout
        if not self._e_stopped:
            m_erpm = self._commanded_erpm.get(None, 0)
            s_erpm = self._commanded_erpm.get(slave_id, 0)
            if m_erpm != 0:
                self._send(bytes([COMM_SET_RPM]) + struct.pack(">i", m_erpm))
            if s_erpm != 0:
                self._send(
                    bytes([COMM_SET_RPM]) + struct.pack(">i", s_erpm),
                    can_id=slave_id,
                )

    def _publish_odom(self) -> None:
        now = time.time()
        m_telem = self._latest_telemetry.get(None)
        s_telem = self._latest_telemetry.get(self._config.slave_can_id)

        if m_telem is None:
            return

        m_tach = m_telem.tachometer
        # Invert tachometer reading to match physical direction
        if self._config.invert_master:
            m_tach = -m_tach

        s_tach = None
        if s_telem is not None:
            s_tach = s_telem.tachometer
            if self._config.invert_slave:
                s_tach = -s_tach

        last_m = self._last_tachometer.get(None)
        if last_m is not None and self._last_odom_time is not None:
            dt = now - self._last_odom_time
            if dt > 0:
                m_dist = _tachometer_to_distance(m_tach - last_m, self._config)

                if s_tach is not None:
                    last_s = self._last_tachometer.get(self._config.slave_can_id)
                    if last_s is not None:
                        s_dist = _tachometer_to_distance(s_tach - last_s, self._config)
                        linear_v = (m_dist + s_dist) / (2.0 * dt)
                        angular_v = (m_dist - s_dist) / (self._config.wheel_separation * dt)
                    else:
                        linear_v = m_dist / dt
                        angular_v = 0.0
                else:
                    linear_v = m_dist / dt
                    angular_v = 0.0

                msg = Odometry()
                msg.header.stamp = _stamp_from_epoch(now)
                msg.header.frame_id = "odom"
                msg.child_frame_id = "base_link"
                msg.twist.twist.linear.x = linear_v
                msg.twist.twist.angular.z = angular_v
                msg.twist.covariance[0] = 0.01
                msg.twist.covariance[35] = 0.05 if s_tach is not None else 99.0
                self._odom_pub.publish(msg)

        self._last_tachometer[None] = m_tach
        if s_tach is not None:
            self._last_tachometer[self._config.slave_can_id] = s_tach
        self._last_odom_time = now

    # ── Status (1Hz) ──────────────────────────────────────────────────────

    def _publish_status(self) -> None:
        zero = VescTelemetry()
        m = self._latest_telemetry.get(None) or zero
        s = self._latest_telemetry.get(self._config.slave_can_id) or zero
        pp = self._config.pole_pairs
        m_cmd = self._commanded_erpm.get(None, 0)
        s_cmd = self._commanded_erpm.get(self._config.slave_can_id, 0)

        def _motor_dict(t: VescTelemetry, cmd: int) -> dict:
            return {
                "rpm": t.erpm // pp,
                "erpm": t.erpm,
                "temp_fet": t.temp_fet,
                "temp_motor": t.temp_motor,
                "current_motor": t.current_motor,
                "current_input": t.current_input,
                "voltage_input": t.voltage_input,
                "duty_cycle": t.duty_cycle,
                "tachometer": t.tachometer,
                "fault_code": t.fault_code,
                "fault_name": FAULT_NAMES.get(t.fault_code, "UNKNOWN"),
                "commanded_erpm": cmd,
                "commanded_rpm": cmd // pp,
            }

        # Top-level fields use master values for Foxglove panel compatibility
        status: dict = {
            "connected": self._serial is not None,
            "mode": self._config.mode,
            "e_stopped": self._e_stopped,
            **_motor_dict(m, m_cmd),
            "master": _motor_dict(m, m_cmd),
            "slave": _motor_dict(s, s_cmd),
            "target_rpm": self._config.target_rpm,
            "max_rpm": self._config.max_rpm,
            "cmd_vel_age_ms": (
                int((time.monotonic() - self._last_cmd_time) * 1000)
                if self._last_cmd_vel is not None
                else None
            ),
            "invert_master": self._config.invert_master,
            "invert_slave": self._config.invert_slave,
            "ramp_up_rpm_s": self._config.ramp_up_rpm_s,
            "ramp_down_rpm_s": self._config.ramp_down_rpm_s,
            "stop_rpm": self._config.stop_rpm,
            "max_linear_vel": self._config.max_linear_vel,
            "max_angular_vel": self._config.max_angular_vel,
            "errors": self._error_count,
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    # ── Lifecycle ─────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        zero_current = bytes([COMM_SET_CURRENT]) + struct.pack(">i", 0)
        self._send(zero_current)
        self._send(zero_current, can_id=self._config.slave_can_id)
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
