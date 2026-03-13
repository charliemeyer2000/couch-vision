"""Interactive WASD teleop for commanding VESC RPM targets from the terminal."""

from __future__ import annotations

import argparse
import select
import struct
import sys
import termios
import time
import tty
from contextlib import AbstractContextManager
from dataclasses import dataclass

import serial
from serial.tools.list_ports import comports

COMM_SET_RPM = 8
COMM_FORWARD_CAN = 34

DEVICE_VID = 0x0483
DEVICE_PID = 0x5740
DEFAULT_SLAVE_CAN = 19
DEFAULT_BAUD = 115200
DEFAULT_POLE_PAIRS = 7
DEFAULT_DRIVE_RPM = 400
DEFAULT_TURN_RPM = 300
DEFAULT_SEND_PERIOD = 0.05
DEFAULT_INVERT_MASTER = True
DEFAULT_INVERT_SLAVE = False


@dataclass(frozen=True)
class CommandTargets:
    """Per-controller mechanical RPM targets."""

    left_rpm: int
    right_rpm: int
    label: str


@dataclass(frozen=True)
class TeleopConfig:
    """Runtime teleop settings."""

    drive_rpm: int
    turn_rpm: int
    pole_pairs: int
    slave_can: int
    invert_master: bool
    invert_slave: bool
    turn_mode: str


class RawTerminal(AbstractContextManager["RawTerminal"]):
    """Temporarily switch stdin to cbreak mode for single-key reads."""

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


def crc16(data: bytes) -> int:
    """Compute the VESC CRC16 checksum for a payload."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def pkt(payload: bytes) -> bytes:
    """Wrap a VESC payload in the short packet format."""
    checksum = crc16(payload)
    return (
        bytes([0x02, len(payload)])
        + payload
        + checksum.to_bytes(2, "big")
        + bytes([0x03])
    )


def find_vesc_port() -> str:
    """Locate the USB-attached VESC master controller."""
    for port in comports():
        if port.vid == DEVICE_VID and port.pid == DEVICE_PID:
            return port.device
    raise RuntimeError("No VESC found")


def set_rpm(connection: serial.Serial, erpm: int, can_id: int | None = None) -> None:
    """Send a `COMM_SET_RPM` command to the master or forwarded slave."""
    payload = struct.pack(">i", erpm)
    if can_id is not None:
        connection.write(pkt(bytes([COMM_FORWARD_CAN, can_id, COMM_SET_RPM]) + payload))
        return
    connection.write(pkt(bytes([COMM_SET_RPM]) + payload))


def mechanical_to_erpm(rpm: int, pole_pairs: int) -> int:
    """Convert mechanical RPM to electrical RPM."""
    return rpm * pole_pairs


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Drive the dual VESC from the terminal with WASD. "
            "W/S drive both wheels, A/D rotate in place, space stops, Q quits."
        )
    )
    parser.add_argument("--drive-rpm", type=int, default=DEFAULT_DRIVE_RPM)
    parser.add_argument("--turn-rpm", type=int, default=DEFAULT_TURN_RPM)
    parser.add_argument("--slave-can", type=int, default=DEFAULT_SLAVE_CAN)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--pole-pairs", type=int, default=DEFAULT_POLE_PAIRS)
    parser.add_argument("--send-period", type=float, default=DEFAULT_SEND_PERIOD)
    parser.add_argument(
        "--invert-master",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_INVERT_MASTER,
        help="Invert the master/left controller direction.",
    )
    parser.add_argument(
        "--invert-slave",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_INVERT_SLAVE,
        help="Invert the slave/right controller direction.",
    )
    parser.add_argument(
        "--turn-mode",
        choices=("pivot", "spin"),
        default="pivot",
        help="Pivot stops one wheel; spin drives wheels in opposite directions.",
    )
    return parser.parse_args()


def command_for_key(key: str, config: TeleopConfig) -> CommandTargets | None:
    """Map a pressed key to left/right RPM targets."""
    lowered = key.lower()
    if lowered == "w":
        return CommandTargets(
            left_rpm=apply_inversion(config.drive_rpm, config.invert_master),
            right_rpm=apply_inversion(config.drive_rpm, config.invert_slave),
            label="forward",
        )
    if lowered == "s":
        return CommandTargets(
            left_rpm=apply_inversion(-config.drive_rpm, config.invert_master),
            right_rpm=apply_inversion(-config.drive_rpm, config.invert_slave),
            label="reverse",
        )
    if lowered == "a":
        if config.turn_mode == "spin":
            return CommandTargets(
                left_rpm=apply_inversion(-config.turn_rpm, config.invert_master),
                right_rpm=apply_inversion(config.turn_rpm, config.invert_slave),
                label="rotate left",
            )
        return CommandTargets(
            left_rpm=0,
            right_rpm=apply_inversion(config.turn_rpm, config.invert_slave),
            label="pivot left",
        )
    if lowered == "d":
        if config.turn_mode == "spin":
            return CommandTargets(
                left_rpm=apply_inversion(config.turn_rpm, config.invert_master),
                right_rpm=apply_inversion(-config.turn_rpm, config.invert_slave),
                label="rotate right",
            )
        return CommandTargets(
            left_rpm=apply_inversion(config.turn_rpm, config.invert_master),
            right_rpm=0,
            label="pivot right",
        )
    if key == " ":
        return CommandTargets(left_rpm=0, right_rpm=0, label="stop")
    return None


def apply_inversion(rpm: int, invert: bool) -> int:
    """Apply an optional sign inversion to a target RPM."""
    return -rpm if invert else rpm


def read_key(timeout: float) -> str | None:
    """Read a single keypress if one is available before timeout."""
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    return sys.stdin.read(1)


def send_targets(
    connection: serial.Serial,
    targets: CommandTargets,
    pole_pairs: int,
    slave_can: int,
) -> None:
    """Send left/right RPM targets to the master and slave controllers."""
    set_rpm(connection, mechanical_to_erpm(targets.left_rpm, pole_pairs))
    set_rpm(connection, mechanical_to_erpm(targets.right_rpm, pole_pairs), slave_can)


def print_command(targets: CommandTargets) -> None:
    """Print the active teleop command."""
    print(
        f"Command: {targets.label:<12s} "
        f"left={targets.left_rpm:+5d} RPM "
        f"right={targets.right_rpm:+5d} RPM"
    )


def main() -> None:
    """Run terminal-based RPM teleop until the user quits."""
    args = parse_args()
    port = find_vesc_port()
    config = TeleopConfig(
        drive_rpm=args.drive_rpm,
        turn_rpm=args.turn_rpm,
        pole_pairs=args.pole_pairs,
        slave_can=args.slave_can,
        invert_master=args.invert_master,
        invert_slave=args.invert_slave,
        turn_mode=args.turn_mode,
    )
    current_targets = CommandTargets(left_rpm=0, right_rpm=0, label="stop")
    last_send_time = 0.0

    print(f"Port: {port}")
    print("Controls: W forward, S reverse, A rotate left, D rotate right, space stop, Q quit")
    print(
        f"Using drive RPM={args.drive_rpm}, turn RPM={args.turn_rpm}, "
        f"slave CAN={args.slave_can}, send period={args.send_period:.3f}s"
    )
    print(
        f"Master controller is left wheel (invert={config.invert_master}). "
        f"Slave controller is right wheel (invert={config.invert_slave})."
    )
    print(f"Turn mode: {config.turn_mode}")
    print_command(current_targets)

    with serial.Serial(port, args.baud, timeout=0.02) as connection, RawTerminal():
        try:
            while True:
                key = read_key(timeout=args.send_period)
                if key is not None:
                    if key.lower() == "q":
                        break
                    next_targets = command_for_key(key, config)
                    if next_targets is not None and next_targets != current_targets:
                        current_targets = next_targets
                        print_command(current_targets)

                now = time.monotonic()
                if now - last_send_time >= args.send_period:
                    send_targets(
                        connection,
                        current_targets,
                        pole_pairs=config.pole_pairs,
                        slave_can=config.slave_can,
                    )
                    last_send_time = now
        except KeyboardInterrupt:
            pass
        finally:
            stop_targets = CommandTargets(left_rpm=0, right_rpm=0, label="stop")
            send_targets(
                connection,
                stop_targets,
                pole_pairs=config.pole_pairs,
                slave_can=config.slave_can,
            )
            print("\nStopped.")


if __name__ == "__main__":
    main()
