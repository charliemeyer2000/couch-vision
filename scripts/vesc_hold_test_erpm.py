#!/usr/bin/env python3
"""VESC dual-motor position hold test using ERPM commands.

This variant keeps the same tachometer-based outer position loop as
``hold_test.py`` but commands ``COMM_SET_RPM`` / ERPM instead of motor current.
It exists for comparison and tuning experiments; on this hardware the VESC
speed PID is known to be less stable than direct current control.

Both motors resist displacement and actively return to their starting position.
Master motor is controlled directly over USB; slave motor via CAN forwarding
(slave CAN ID 19).

Usage:
    uv run --with pyserial python hold_test_erpm.py
    uv run --with pyserial python hold_test_erpm.py --single
    uv run --with pyserial python hold_test_erpm.py --kp 20 --max-erpm 1200
"""

from __future__ import annotations

import argparse
import glob
import signal
import struct
import sys
import time
from dataclasses import dataclass, field

import serial
from serial.tools.list_ports import comports

_VESC_USB_VID = 0x0483
_VESC_USB_PID = 0x5740

COMM_GET_VALUES = 4
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


def _crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
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
    for port in comports():
        if port.vid == _VESC_USB_VID and port.pid == _VESC_USB_PID:
            return port.device
    for pattern in ["/dev/ttyACM*", "/dev/cu.usbmodem*"]:
        for path in sorted(glob.glob(pattern)):
            return path
    return None


@dataclass
class PID:
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
            return self.kp * error

        dt = now - self._prev_time
        if dt <= 0:
            return 0.0

        p = self.kp * error

        self._integral += error * dt
        i_max = self.max_output / max(self.ki, 1e-9)
        self._integral = max(-i_max, min(i_max, self._integral))
        i = self.ki * self._integral

        d = self.kd * (error - self._prev_error) / dt

        self._prev_error = error
        self._prev_time = now

        return max(-self.max_output, min(self.max_output, p + i + d))


class VescConn:
    def __init__(self, port: str, baud: int = 115200) -> None:
        # Keep timeouts well under the VESC USB timeout bug threshold.
        self.ser = serial.Serial(port, baud, timeout=0.1, write_timeout=0.1)

    def _read_packet(self, timeout: float = 0.2) -> bytes:
        buf = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            avail = self.ser.in_waiting
            if avail:
                buf += self.ser.read(avail)
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

    def _transact(self, payload: bytes, timeout: float = 0.2) -> bytes | None:
        self.ser.reset_input_buffer()
        self.ser.write(_build_packet(payload))
        raw = self._read_packet(timeout)
        return _parse_response(raw)

    def _send(self, payload: bytes) -> None:
        self.ser.write(_build_packet(payload))

    def get_values(self, can_id: int | None = None) -> dict[str, float | int] | None:
        if can_id is not None:
            cmd = bytes([COMM_FORWARD_CAN, can_id, COMM_GET_VALUES])
            timeout = 0.5
        else:
            cmd = bytes([COMM_GET_VALUES])
            timeout = 0.2

        resp = self._transact(cmd, timeout)
        if resp is None or len(resp) < 54 or resp[0] != COMM_GET_VALUES:
            return None

        d = resp[1:]
        return {
            "tachometer": struct.unpack(">i", d[44:48])[0],
            "erpm": struct.unpack(">i", d[22:26])[0],
            "voltage": struct.unpack(">h", d[26:28])[0] / 10.0,
            "current_motor": struct.unpack(">i", d[4:8])[0] / 100.0,
            "current_input": struct.unpack(">i", d[8:12])[0] / 100.0,
            "temp_fet": struct.unpack(">h", d[0:2])[0] / 10.0,
            "fault": d[52],
        }

    def set_rpm(self, erpm: int, can_id: int | None = None) -> None:
        rpm_bytes = struct.pack(">i", erpm)
        if can_id is not None:
            self._send(bytes([COMM_FORWARD_CAN, can_id, COMM_SET_RPM]) + rpm_bytes)
        else:
            self._send(bytes([COMM_SET_RPM]) + rpm_bytes)

    def close(self) -> None:
        self.ser.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="VESC dual-motor position hold using ERPM")
    parser.add_argument("--port", default="auto", help="Serial port or 'auto'")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--kp", type=float, default=20.0, help="Proportional gain (ERPM per tach count)")
    parser.add_argument("--ki", type=float, default=0.0, help="Integral gain")
    parser.add_argument("--kd", type=float, default=0.0, help="Derivative gain")
    parser.add_argument("--max-erpm", type=int, default=1200, help="Max ERPM command magnitude")
    parser.add_argument(
        "--max-displacement",
        type=int,
        default=500,
        help="Safety cutoff: max tach counts from home before disabling",
    )
    parser.add_argument("--rate", type=float, default=25.0, help="Control loop target Hz")
    parser.add_argument("--slave-can-id", type=int, default=19, help="CAN ID of slave motor")
    parser.add_argument("--single", action="store_true", help="Master motor only, skip slave")
    parser.add_argument(
        "--invert",
        action="store_true",
        help="Invert ERPM command direction (use if motor runs away instead of holding)",
    )
    args = parser.parse_args()

    port = args.port
    if port == "auto":
        port = find_vesc_port()
        if port is None:
            print("ERROR: No VESC found. Connect via USB or specify --port.")
            print("       If VESC Tool is open, close it first (exclusive serial access).")
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
    health_check = vesc.get_values()
    if health_check is None:
        print("ERROR: VESC not responding — power cycle the ESC and try again.")
        print("       (Known STM32 USB CDC bug: comm_usb.c was_timeout flag latches)")
        vesc.close()
        sys.exit(1)

    sign = -1 if args.invert else 1

    motor_ids: list[int | None] = [None]
    if not args.single:
        motor_ids.append(args.slave_can_id)

    homes: dict[int | None, int] = {}
    for mid in motor_ids:
        vals = vesc.get_values(mid)
        label = "Master (USB)" if mid is None else f"Slave  (CAN {mid})"
        if vals is None:
            print(f"  {label}: no response — skipping")
            continue
        homes[mid] = int(vals["tachometer"])
        print(
            f"  {label}: tach={vals['tachometer']}, {vals['voltage']:.1f}V, "
            f"{vals['temp_fet']:.1f}°C"
        )

    if not homes:
        print("ERROR: No motors responding.")
        vesc.close()
        sys.exit(1)

    pids: dict[int | None, PID] = {
        mid: PID(kp=args.kp, ki=args.ki, kd=args.kd, max_output=float(args.max_erpm))
        for mid in homes
    }

    running = True

    def _stop(sig: int, frame: object) -> None:
        del sig, frame
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    print(f"\nPID: Kp={args.kp}  Ki={args.ki}  Kd={args.kd}  max={args.max_erpm} ERPM")
    print("Using VESC speed control underneath the outer position loop.")
    print("If it oscillates, reduce --kp / --max-erpm or use hold_test.py instead.\n")

    period = 1.0 / args.rate
    last_print = 0.0
    faulted: set[int | None] = set()

    try:
        while running:
            tick_start = time.monotonic()
            now = time.monotonic()
            parts: list[str] = []

            for mid in list(homes.keys()):
                if mid in faulted:
                    continue

                vals = vesc.get_values(mid)
                if vals is None:
                    continue

                if vals["fault"] != 0:
                    name = FAULT_NAMES.get(int(vals["fault"]), f"UNKNOWN({vals['fault']})")
                    label = "M" if mid is None else "S"
                    print(f"\n[{label}] FAULT: {name} — disabling motor")
                    vesc.set_rpm(0, mid)
                    faulted.add(mid)
                    continue

                tach = int(vals["tachometer"])
                error = homes[mid] - tach

                if abs(error) > args.max_displacement:
                    label = "M" if mid is None else "S"
                    print(f"\n[{label}] SAFETY: displacement {error} exceeds limit — cutting ERPM")
                    print(f"         If motor was pushing AWAY from home, try --invert")
                    vesc.set_rpm(0, mid)
                    faulted.add(mid)
                    continue

                commanded_erpm = int(sign * pids[mid].update(float(error), now))
                vesc.set_rpm(commanded_erpm, mid)

                label = "M" if mid is None else "S"
                parts.append(
                    f"[{label}] err={error:+4d} cmd={commanded_erpm:+5d}erpm "
                    f"act={int(vals['erpm']):+5d}erpm iin={float(vals['current_input']):+5.2f}A"
                )

            if parts and now - last_print >= 0.5:
                print("  " + "  |  ".join(parts))
                last_print = now

            if len(faulted) >= len(homes):
                print("\nAll motors faulted or disabled — exiting.")
                break

            elapsed = time.monotonic() - tick_start
            if elapsed < period:
                time.sleep(period - elapsed)

    finally:
        print("\nReleasing motors (zero ERPM)...")
        for mid in homes:
            vesc.set_rpm(0, mid)
            time.sleep(0.01)
        vesc.close()
        print("Done.")


if __name__ == "__main__":
    main()
