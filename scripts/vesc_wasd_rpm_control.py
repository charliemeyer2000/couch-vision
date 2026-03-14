"""Interactive WASD teleop using pure ERPM commands.

While a movement key is held, this script sends direct ERPM commands. When the
key is released, it switches to the same tachometer-based outer hold loop used
by ``hold_test_erpm.py`` and commands ERPM back toward the release position.

Because terminal input has no true key-up event, release is detected with a
short timeout on repeated keypresses.
"""

from __future__ import annotations

import argparse
import glob
import select
import struct
import sys
import termios
import time
import tty
from contextlib import AbstractContextManager
from dataclasses import dataclass, field

import serial
from serial.tools.list_ports import comports

_VESC_USB_VID = 0x0483
_VESC_USB_PID = 0x5740

COMM_GET_VALUES = 4
COMM_SET_RPM = 8
COMM_FORWARD_CAN = 34

DEFAULT_SLAVE_CAN = 19
DEFAULT_BAUD = 115200
DEFAULT_POLE_PAIRS = 7

SAFETY = 4
DEFAULT_DRIVE_RPM = 300 # 450 / SAFETY
DEFAULT_TURN_RPM = 300 # 450 / SAFETY
DEFAULT_RATE = 25.0
DEFAULT_RELEASE_TIMEOUT = 0.30
DEFAULT_MAX_ERPM = 1200
DEFAULT_MAX_DISPLACEMENT = 500
DEFAULT_INVERT_MASTER = True
DEFAULT_INVERT_SLAVE = False
DEFAULT_TURN_MODE = "spin"
DEFAULT_MASTER_SIDE = "right"
MASTER_TELEMETRY_TIMEOUT = 0.005
SLAVE_TELEMETRY_TIMEOUT = 0.008
SLAVE_POLL_DIVIDER = 4
TELEMETRY_STALE_TIMEOUT = 0.30

FAULT_NAMES = {
    0: "NONE",
    1: "OVER_VOLTAGE",
    2: "UNDER_VOLTAGE",
    3: "DRV",
    4: "ABS_OVER_CURRENT",
    5: "OVER_TEMP_FET",
    6: "OVER_TEMP_MOTOR",
}


@dataclass
class PID:
    """Host-side PID controller used for ERPM hold."""

    kp: float
    ki: float
    kd: float
    max_output: float
    _integral: float = field(default=0.0, init=False, repr=False)
    _prev_error: float = field(default=0.0, init=False, repr=False)
    _prev_time: float = field(default=0.0, init=False, repr=False)

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = 0.0

    def update(self, error: float, now: float) -> float:
        if self._prev_time == 0.0:
            self._prev_time = now
            self._prev_error = error
            return max(-self.max_output, min(self.max_output, self.kp * error))

        dt = now - self._prev_time
        if dt <= 0:
            return 0.0

        proportional = self.kp * error

        self._integral += error * dt
        integral_limit = self.max_output / max(self.ki, 1e-9)
        self._integral = max(-integral_limit, min(integral_limit, self._integral))
        integral = self.ki * self._integral

        derivative = self.kd * (error - self._prev_error) / dt

        self._prev_error = error
        self._prev_time = now

        output = proportional + integral + derivative
        return max(-self.max_output, min(self.max_output, output))


@dataclass(frozen=True)
class MotionCommand:
    """Per-controller ERPM targets for a teleop action."""

    master_erpm: int
    slave_erpm: int
    label: str


class RawTerminal(AbstractContextManager["RawTerminal"]):
    """Temporarily switch stdin into cbreak mode for single-key reads."""

    def __init__(self) -> None:
        self._fd = sys.stdin.fileno()
        self._original_settings: list[int] | None = None

    def __enter__(self) -> "RawTerminal":
        self._original_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        if self._original_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._original_settings)


class VescConn:
    """Minimal VESC serial transport with telemetry and ERPM commands."""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD) -> None:
        self.ser = serial.Serial(port, baud, timeout=0.1, write_timeout=0.1)

    def _read_packet(self, timeout: float = 0.2) -> bytes:
        buf = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            available = self.ser.in_waiting
            if available:
                buf += self.ser.read(available)
                if len(buf) >= 5 and buf[0] == 0x02:
                    expected = buf[1] + 5
                    if len(buf) >= expected:
                        return buf[:expected]
                if len(buf) >= 6 and buf[0] == 0x03:
                    payload_length = struct.unpack(">H", buf[1:3])[0]
                    expected = payload_length + 6
                    if len(buf) >= expected:
                        return buf[:expected]
            else:
                time.sleep(0.001)
        return buf

    def _transact(self, payload: bytes, timeout: float = 0.2) -> bytes | None:
        self.ser.reset_input_buffer()
        self.ser.write(_build_packet(payload))
        return _parse_response(self._read_packet(timeout))

    def _send(self, payload: bytes) -> None:
        self.ser.write(_build_packet(payload))

    def get_values(
        self,
        can_id: int | None = None,
        timeout: float | None = None,
    ) -> dict[str, float | int] | None:
        if can_id is not None:
            payload = bytes([COMM_FORWARD_CAN, can_id, COMM_GET_VALUES])
            timeout = SLAVE_TELEMETRY_TIMEOUT if timeout is None else timeout
        else:
            payload = bytes([COMM_GET_VALUES])
            timeout = MASTER_TELEMETRY_TIMEOUT if timeout is None else timeout

        response = self._transact(payload, timeout)
        if response is None or len(response) < 54 or response[0] != COMM_GET_VALUES:
            return None

        data = response[1:]
        return {
            "tachometer": struct.unpack(">i", data[44:48])[0],
            "erpm": struct.unpack(">i", data[22:26])[0],
            "voltage": struct.unpack(">h", data[26:28])[0] / 10.0,
            "current_motor": struct.unpack(">i", data[4:8])[0] / 100.0,
            "current_input": struct.unpack(">i", data[8:12])[0] / 100.0,
            "temp_fet": struct.unpack(">h", data[0:2])[0] / 10.0,
            "fault": data[52],
        }

    def set_rpm(self, erpm: int, can_id: int | None = None) -> None:
        rpm_bytes = struct.pack(">i", erpm)
        if can_id is not None:
            self._send(bytes([COMM_FORWARD_CAN, can_id, COMM_SET_RPM]) + rpm_bytes)
            return
        self._send(bytes([COMM_SET_RPM]) + rpm_bytes)

    def close(self) -> None:
        self.ser.close()


def _crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def _build_packet(payload: bytes) -> bytes:
    crc = _crc16(payload)
    return bytes([0x02, len(payload)]) + payload + crc.to_bytes(2, "big") + bytes([0x03])


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


def find_vesc_port() -> str | None:
    """Locate the VESC serial device."""
    for port in comports():
        if port.vid == _VESC_USB_VID and port.pid == _VESC_USB_PID:
            return port.device
    for pattern in ("/dev/ttyACM*", "/dev/cu.usbmodem*"):
        for path in sorted(glob.glob(pattern)):
            return path
    return None


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "WASD teleop using direct ERPM while a key is held, and ERPM hold "
            "control when released."
        )
    )
    parser.add_argument("--port", default="auto", help="Serial port or 'auto'")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--pole-pairs", type=int, default=DEFAULT_POLE_PAIRS)
    parser.add_argument("--drive-rpm", type=int, default=DEFAULT_DRIVE_RPM)
    parser.add_argument("--turn-rpm", type=int, default=DEFAULT_TURN_RPM)
    parser.add_argument("--kp", type=float, default=20.0)
    parser.add_argument("--ki", type=float, default=0.0)
    parser.add_argument("--kd", type=float, default=0.0)
    parser.add_argument("--max-erpm", type=int, default=DEFAULT_MAX_ERPM)
    parser.add_argument("--max-displacement", type=int, default=DEFAULT_MAX_DISPLACEMENT)
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE)
    parser.add_argument("--release-timeout", type=float, default=DEFAULT_RELEASE_TIMEOUT)
    parser.add_argument("--slave-can-id", type=int, default=DEFAULT_SLAVE_CAN)
    parser.add_argument("--single", action="store_true", help="Master motor only")
    parser.add_argument(
        "--invert-master",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_INVERT_MASTER,
        help="Invert master ERPM direction.",
    )
    parser.add_argument(
        "--invert-slave",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_INVERT_SLAVE,
        help="Invert slave ERPM direction.",
    )
    parser.add_argument(
        "--turn-mode",
        choices=("pivot", "spin"),
        default=DEFAULT_TURN_MODE,
        help="Spin does in-place turns; pivot stops one wheel.",
    )
    parser.add_argument(
        "--master-side",
        choices=("left", "right"),
        default=DEFAULT_MASTER_SIDE,
        help="Which physical wheel is connected to the USB/master controller.",
    )
    return parser.parse_args()


def apply_inversion(value: int, invert: bool) -> int:
    """Apply controller direction inversion."""
    return -value if invert else value


def mechanical_to_erpm(rpm: int, pole_pairs: int) -> int:
    """Convert mechanical RPM to ERPM."""
    return rpm * pole_pairs


def erpm_to_rpm(erpm: int, pole_pairs: int) -> float:
    """Convert ERPM back to mechanical RPM."""
    return erpm / pole_pairs


def wheel_rpms_to_command(
    left_rpm: int,
    right_rpm: int,
    args: argparse.Namespace,
) -> MotionCommand:
    """Map semantic left/right wheel RPM targets onto master/slave controllers."""
    left_erpm = mechanical_to_erpm(left_rpm, args.pole_pairs)
    right_erpm = mechanical_to_erpm(right_rpm, args.pole_pairs)

    if args.master_side == "left":
        master_erpm = apply_inversion(left_erpm, args.invert_master)
        slave_erpm = apply_inversion(right_erpm, args.invert_slave)
    else:
        master_erpm = apply_inversion(right_erpm, args.invert_master)
        slave_erpm = apply_inversion(left_erpm, args.invert_slave)

    return MotionCommand(master_erpm=master_erpm, slave_erpm=slave_erpm, label="")


def command_for_key(key: str, args: argparse.Namespace) -> MotionCommand | None:
    """Map a movement key to master/slave ERPM commands."""
    lowered = key.lower()
    if lowered == "w":
        command = wheel_rpms_to_command(args.drive_rpm, args.drive_rpm, args)
        return MotionCommand(command.master_erpm, command.slave_erpm, "forward")
    if lowered == "s":
        command = wheel_rpms_to_command(-args.drive_rpm, -args.drive_rpm, args)
        return MotionCommand(command.master_erpm, command.slave_erpm, "reverse")
    if lowered == "a":
        if args.turn_mode == "spin":
            command = wheel_rpms_to_command(-args.turn_rpm, args.turn_rpm, args)
            return MotionCommand(command.master_erpm, command.slave_erpm, "rotate left")
        command = wheel_rpms_to_command(0, args.turn_rpm, args)
        return MotionCommand(command.master_erpm, command.slave_erpm, "pivot left")
    if lowered == "d":
        if args.turn_mode == "spin":
            command = wheel_rpms_to_command(args.turn_rpm, -args.turn_rpm, args)
            return MotionCommand(command.master_erpm, command.slave_erpm, "rotate right")
        command = wheel_rpms_to_command(args.turn_rpm, 0, args)
        return MotionCommand(command.master_erpm, command.slave_erpm, "pivot right")
    if key == " ":
        return MotionCommand(0, 0, "hold")
    return None


def read_key(timeout: float) -> str | None:
    """Read the latest pending key within the timeout window."""
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None

    latest = sys.stdin.read(1)
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return latest
        latest = sys.stdin.read(1)


def set_hold_targets(
    motor_ids: list[int | None],
    latest_values: dict[int | None, dict[str, float | int]],
    hold_targets: dict[int | None, int],
    pids: dict[int | None, PID],
) -> None:
    """Capture current tachometer values as the new hold positions."""
    for motor_id in motor_ids:
        values = latest_values.get(motor_id)
        if values is None:
            continue
        hold_targets[motor_id] = int(values["tachometer"])
        pids[motor_id].reset()


def telemetry_poll_due(motor_id: int | None, loop_index: int) -> bool:
    """Poll the master every cycle and the slave less often to protect USB."""
    if motor_id is None:
        return True
    return loop_index % SLAVE_POLL_DIVIDER == 0


def print_status(
    mode_label: str,
    motor_ids: list[int | None],
    latest_values: dict[int | None, dict[str, float | int]],
    commanded_erpm: dict[int | None, int],
    hold_targets: dict[int | None, int],
    telemetry_timestamps: dict[int | None, float],
    pole_pairs: int,
    now_monotonic: float,
) -> None:
    """Print compact status for both controllers."""
    parts: list[str] = []
    for motor_id in motor_ids:
        values = latest_values.get(motor_id)
        label = "M" if motor_id is None else "S"
        if values is None:
            parts.append(f"[{label}] NO_DATA")
            continue

        tachometer = int(values["tachometer"])
        delta = hold_targets.get(motor_id, tachometer) - tachometer
        actual_rpm = erpm_to_rpm(int(values["erpm"]), pole_pairs)
        commanded_rpm = erpm_to_rpm(commanded_erpm.get(motor_id, 0), pole_pairs)
        telemetry_age_ms = 1000.0 * (
            now_monotonic - telemetry_timestamps.get(motor_id, now_monotonic)
        )
        stale_marker = " STALE" if telemetry_age_ms > TELEMETRY_STALE_TIMEOUT * 1000.0 else ""
        parts.append(
            f"[{label}] rpm={actual_rpm:+7.1f} delta={delta:+4d} "
            f"age={telemetry_age_ms:5.0f}ms{stale_marker} "
            f"cmd={commanded_rpm:+7.1f}rpm"
        )

    print(f"{mode_label:<12s} " + " | ".join(parts))


def main() -> None:
    """Run teleop until quit, using ERPM for both motion and hold."""
    args = parse_args()
    port = args.port
    if port == "auto":
        port = find_vesc_port()
        if port is None:
            print("ERROR: No VESC found. Connect via USB or specify --port.")
            print("       If VESC Tool is open, close it first.")
            sys.exit(1)

    print(f"Connecting to {port}...")
    try:
        vesc = VescConn(port, args.baud)
    except serial.SerialException as exc:
        if "Resource busy" in str(exc):
            print(f"ERROR: {port} is busy — close VESC Tool first.")
        else:
            print(f"ERROR: {exc}")
        sys.exit(1)

    print("Health check...")
    if vesc.get_values() is None:
        print("ERROR: VESC not responding — power cycle the ESC and try again.")
        print("       (Known STM32 USB CDC bug: comm_usb.c was_timeout flag latches)")
        vesc.close()
        sys.exit(1)

    motor_ids: list[int | None] = [None]
    if not args.single:
        motor_ids.append(args.slave_can_id)

    latest_values: dict[int | None, dict[str, float | int]] = {}
    for motor_id in motor_ids:
        values = vesc.get_values(motor_id)
        if values is not None:
            latest_values[motor_id] = values

    if not latest_values:
        print("ERROR: No motors responding. Power cycle the ESC and close VESC Tool.")
        vesc.close()
        sys.exit(1)

    missing_motors = [motor_id for motor_id in motor_ids if motor_id not in latest_values]
    for motor_id in missing_motors:
        label = "master" if motor_id is None else f"slave CAN {motor_id}"
        print(f"WARNING: {label} did not respond at startup; skipping it.")
    motor_ids = [motor_id for motor_id in motor_ids if motor_id in latest_values]

    hold_targets: dict[int | None, int] = {
        motor_id: int(values["tachometer"]) for motor_id, values in latest_values.items()
    }
    telemetry_timestamps: dict[int | None, float] = {
        motor_id: time.monotonic() for motor_id in latest_values
    }
    pids: dict[int | None, PID] = {
        motor_id: PID(args.kp, args.ki, args.kd, float(args.max_erpm))
        for motor_id in motor_ids
    }
    active_key: str | None = None
    last_input_time = 0.0
    motion_command = MotionCommand(0, 0, "hold")
    mode_label = "hold"
    last_mode_label: str | None = None
    last_status_print = 0.0
    period = 1.0 / args.rate
    faulted: set[int | None] = set()
    loop_index = 0

    print("Controls: W forward, S reverse, A left, D right, space hold, Q quit")
    print(
        f"drive_rpm={args.drive_rpm} turn_rpm={args.turn_rpm} "
        f"hold_pid=({args.kp}, {args.ki}, {args.kd}) max_erpm={args.max_erpm}"
    )
    print(
        f"master invert={args.invert_master} slave invert={args.invert_slave} "
        f"turn_mode={args.turn_mode} master_side={args.master_side} "
        f"release_timeout={args.release_timeout:.2f}s"
    )
    print(
        "Movement keys rely on terminal key repeat. If release feels sticky, "
        "lower `--release-timeout`; if it drops too quickly, raise it."
    )

    with RawTerminal():
        try:
            while True:
                loop_start = time.monotonic()
                key = read_key(timeout=period)
                if key is not None:
                    if key.lower() == "q":
                        break
                    if key in ("w", "W", "a", "A", "s", "S", "d", "D"):
                        active_key = key.lower()
                        last_input_time = loop_start
                    elif key == " ":
                        active_key = None
                        last_input_time = 0.0

                movement_active = (
                    active_key is not None
                    and loop_start - last_input_time <= args.release_timeout
                )

                commanded_erpm: dict[int | None, int] = {}
                now = time.monotonic()

                if movement_active:
                    next_command = command_for_key(active_key, args)
                    if next_command is not None:
                        motion_command = next_command
                    mode_label = motion_command.label
                else:
                    if mode_label != "hold":
                        set_hold_targets(motor_ids, latest_values, hold_targets, pids)
                        motion_command = MotionCommand(0, 0, "hold")
                    active_key = None
                    mode_label = "hold"

                if mode_label != last_mode_label:
                    if mode_label == "hold":
                        print("Switching to hold at current position.")
                    else:
                        print(f"Command: {mode_label}")
                    last_mode_label = mode_label

                for motor_id in motor_ids:
                    if motor_id in faulted:
                        continue

                    values = latest_values.get(motor_id)
                    if values is not None:
                        fault = int(values["fault"])
                        if fault != 0:
                            label = "M" if motor_id is None else "S"
                            fault_name = FAULT_NAMES.get(fault, f"UNKNOWN({fault})")
                            print(f"[{label}] FAULT: {fault_name} — disabling motor")
                            vesc.set_rpm(0, motor_id)
                            faulted.add(motor_id)
                            continue

                    if mode_label != "hold":
                        command = (
                            motion_command.master_erpm
                            if motor_id is None
                            else motion_command.slave_erpm
                        )
                        vesc.set_rpm(command, motor_id)
                        commanded_erpm[motor_id] = command
                        continue

                    if values is None:
                        vesc.set_rpm(0, motor_id)
                        commanded_erpm[motor_id] = 0
                        continue

                    telemetry_age = loop_start - telemetry_timestamps.get(
                        motor_id, float("-inf")
                    )
                    if telemetry_age > TELEMETRY_STALE_TIMEOUT:
                        vesc.set_rpm(0, motor_id)
                        commanded_erpm[motor_id] = 0
                        continue

                    tachometer = int(values["tachometer"])
                    error = hold_targets.get(motor_id, tachometer) - tachometer
                    if abs(error) > args.max_displacement:
                        label = "M" if motor_id is None else "S"
                        print(f"[{label}] SAFETY: displacement {error} exceeds limit — disabling")
                        vesc.set_rpm(0, motor_id)
                        faulted.add(motor_id)
                        continue

                    hold_erpm = int(pids[motor_id].update(float(error), now))
                    vesc.set_rpm(hold_erpm, motor_id)
                    commanded_erpm[motor_id] = hold_erpm

                for motor_id in motor_ids:
                    if motor_id in faulted or not telemetry_poll_due(motor_id, loop_index):
                        continue

                    values = vesc.get_values(motor_id)
                    if values is not None:
                        latest_values[motor_id] = values
                        telemetry_timestamps[motor_id] = time.monotonic()

                        fault = int(values["fault"])
                        if fault != 0:
                            label = "M" if motor_id is None else "S"
                            fault_name = FAULT_NAMES.get(fault, f"UNKNOWN({fault})")
                            print(f"[{label}] FAULT: {fault_name} — disabling motor")
                            vesc.set_rpm(0, motor_id)
                            faulted.add(motor_id)

                if latest_values and now - last_status_print >= 0.5:
                    print_status(
                        mode_label,
                        motor_ids,
                        latest_values,
                        commanded_erpm,
                        hold_targets,
                        telemetry_timestamps,
                        args.pole_pairs,
                        loop_start,
                    )
                    last_status_print = now

                if len(faulted) >= len(motor_ids):
                    print("All motors faulted or were disabled. Exiting.")
                    break

                loop_index += 1
                elapsed = time.monotonic() - loop_start
                if elapsed < period:
                    time.sleep(period - elapsed)
        except KeyboardInterrupt:
            pass
        finally:
            print("Stopping motors...")
            for motor_id in motor_ids:
                try:
                    vesc.set_rpm(0, motor_id)
                    time.sleep(0.01)
                except serial.SerialException:
                    pass
            vesc.close()
            print("Done.")


if __name__ == "__main__":
    main()
