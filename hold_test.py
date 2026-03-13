#!/usr/bin/env python3
"""VESC dual-motor position hold test.

Both motors resist displacement and actively return to their starting position.
Uses COMM_SET_CURRENT (bypasses VESC speed PID) with a tachometer-based position
PID running on this host. Master motor controlled directly over USB; slave motor
via CAN forwarding (slave CAN ID 19).

Monitors total input current from PSU and scales back if approaching the limit.
Requires VESC Tool to be disconnected (serial port exclusive access).

Usage:
    uv run --with pyserial python hold_test.py
    uv run --with pyserial python hold_test.py --single       # master only
    uv run --with pyserial python hold_test.py --kp 1.0 --max-current 5.0
    uv run --with pyserial python hold_test.py --psu-limit 20  # 20A PSU budget
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

# ── VESC USB IDs (STM32 VCP on Flipsky FSESC) ───────────────────────────────

_VESC_USB_VID = 0x0483
_VESC_USB_PID = 0x5740

# ── VESC protocol ────────────────────────────────────────────────────────────

COMM_GET_VALUES = 4
COMM_SET_CURRENT = 6
COMM_FORWARD_CAN = 34

FAULT_NAMES = {
    0: "NONE", 1: "OVER_VOLTAGE", 2: "UNDER_VOLTAGE", 3: "DRV",
    4: "ABS_OVER_CURRENT", 5: "OVER_TEMP_FET", 6: "OVER_TEMP_MOTOR",
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


# ── Port detection ───────────────────────────────────────────────────────────


def find_vesc_port() -> str | None:
    for port in comports():
        if port.vid == _VESC_USB_VID and port.pid == _VESC_USB_PID:
            return port.device
    for pattern in ["/dev/ttyACM*", "/dev/cu.usbmodem*"]:
        for path in sorted(glob.glob(pattern)):
            return path
    return None


# ── PID controller ───────────────────────────────────────────────────────────


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
            return self.kp * error  # first tick: P-only

        dt = now - self._prev_time
        if dt <= 0:
            return 0.0

        p = self.kp * error

        self._integral += error * dt
        # Anti-windup: clamp integral to prevent saturation
        i_max = self.max_output / max(self.ki, 1e-9)
        self._integral = max(-i_max, min(i_max, self._integral))
        i = self.ki * self._integral

        d = self.kd * (error - self._prev_error) / dt

        self._prev_error = error
        self._prev_time = now

        return max(-self.max_output, min(self.max_output, p + i + d))


# ── VESC serial communication ───────────────────────────────────────────────


class VescConn:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)

    def _read_packet(self, timeout: float = 0.2) -> bytes:
        """Read until a complete VESC packet is received or timeout."""
        buf = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            avail = self.ser.in_waiting
            if avail:
                buf += self.ser.read(avail)
                # Check for complete short packet (0x02)
                if len(buf) >= 5 and buf[0] == 0x02:
                    expected = buf[1] + 5
                    if len(buf) >= expected:
                        return buf[:expected]
                # Check for complete long packet (0x03)
                if len(buf) >= 6 and buf[0] == 0x03:
                    plen = struct.unpack(">H", buf[1:3])[0]
                    expected = plen + 6
                    if len(buf) >= expected:
                        return buf[:expected]
            else:
                time.sleep(0.001)
        return buf

    def _transact(self, payload: bytes, timeout: float = 0.2) -> bytes | None:
        """Send command, read and parse response. Returns payload or None."""
        self.ser.reset_input_buffer()
        self.ser.write(_build_packet(payload))
        raw = self._read_packet(timeout)
        return _parse_response(raw)

    def _send(self, payload: bytes) -> None:
        """Fire-and-forget command (no response expected)."""
        self.ser.write(_build_packet(payload))

    def get_values(self, can_id: int | None = None) -> dict | None:
        """Read telemetry. can_id=None for master, integer for CAN-forwarded."""
        if can_id is not None:
            cmd = bytes([COMM_FORWARD_CAN, can_id, COMM_GET_VALUES])
            # CAN forwarding adds round-trip latency
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

    def scan_can(self, max_id: int = 3) -> list[int]:
        """Probe CAN IDs 0..max_id and return those that respond."""
        found = []
        for cid in range(max_id + 1):
            vals = self.get_values(can_id=cid)
            if vals is not None:
                found.append(cid)
        return found

    def set_current(self, amps: float, can_id: int | None = None) -> None:
        """Command motor current. Positive = forward, negative = reverse."""
        current_bytes = struct.pack(">i", int(amps * 1000))
        if can_id is not None:
            self._send(bytes([COMM_FORWARD_CAN, can_id, COMM_SET_CURRENT]) + current_bytes)
        else:
            self._send(bytes([COMM_SET_CURRENT]) + current_bytes)

    def close(self) -> None:
        self.ser.close()


# ── Main hold loop ───────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(description="VESC dual-motor position hold")
    parser.add_argument("--port", default="auto", help="Serial port or 'auto'")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--kp", type=float, default=0.5, help="Proportional gain (A per tach count)")
    parser.add_argument("--ki", type=float, default=0.05, help="Integral gain")
    parser.add_argument("--kd", type=float, default=0.1, help="Derivative gain")
    parser.add_argument("--max-current", type=float, default=8.0, help="Max hold current (A)")
    parser.add_argument("--max-displacement", type=int, default=500,
                        help="Safety cutoff: max tach counts from home before disabling (~12 revs at 42 counts/rev)")
    parser.add_argument("--rate", type=float, default=50.0, help="Control loop target Hz")
    parser.add_argument("--slave-can-id", type=int, default=19, help="CAN ID of slave motor (default: 19)")
    parser.add_argument("--single", action="store_true", help="Master motor only, skip slave")
    parser.add_argument("--invert", action="store_true",
                        help="Invert current direction (use if motor runs away instead of holding)")
    args = parser.parse_args()

    # Find port
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
    except serial.SerialException as e:
        if "Resource busy" in str(e):
            print(f"ERROR: {port} is busy — close VESC Tool first.")
        else:
            print(f"ERROR: {e}")
        sys.exit(1)

    sign = -1.0 if args.invert else 1.0

    # Which motors to control: None = master (direct USB), 19 = slave (CAN)
    motor_ids: list[int | None] = [None]
    if not args.single:
        motor_ids.append(args.slave_can_id)

    # Read initial positions
    homes: dict[int | None, int] = {}
    for mid in motor_ids:
        vals = vesc.get_values(mid)
        label = "Master (USB)" if mid is None else f"Slave  (CAN {mid})"
        if vals is None:
            print(f"  {label}: no response — skipping")
            continue
        homes[mid] = vals["tachometer"]
        print(f"  {label}: tach={vals['tachometer']}, {vals['voltage']:.1f}V, {vals['temp_fet']:.1f}°C")

    if not homes:
        print("ERROR: No motors responding.")
        vesc.close()
        sys.exit(1)

    # Create PID per motor
    pids: dict[int | None, PID] = {
        mid: PID(kp=args.kp, ki=args.ki, kd=args.kd, max_output=args.max_current)
        for mid in homes
    }

    # Ctrl+C handler
    running = True

    def _stop(sig: int, frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    print(f"\nPID: Kp={args.kp}  Ki={args.ki}  Kd={args.kd}  max={args.max_current}A")
    print(f"Holding {len(homes)} motor(s). Try turning them by hand. Ctrl+C to release.\n")

    period = 1.0 / args.rate
    last_print = 0.0
    faulted: set[int | None] = set()

    try:
        while running:
            t0 = time.monotonic()
            now = time.time()
            parts: list[str] = []

            for mid in list(homes.keys()):
                if mid in faulted:
                    continue

                vals = vesc.get_values(mid)
                if vals is None:
                    continue

                # Fault check
                if vals["fault"] != 0:
                    name = FAULT_NAMES.get(vals["fault"], f"UNKNOWN({vals['fault']})")
                    label = "M" if mid is None else "S"
                    print(f"\n[{label}] FAULT: {name} — disabling motor")
                    vesc.set_current(0.0, mid)
                    faulted.add(mid)
                    continue

                tach = vals["tachometer"]
                error = homes[mid] - tach

                # Runaway safety: if displaced too far, the PID sign might be wrong
                if abs(error) > args.max_displacement:
                    label = "M" if mid is None else "S"
                    print(f"\n[{label}] SAFETY: displacement {error} exceeds limit — cutting current")
                    print(f"         If motor was pushing AWAY from home, try --invert")
                    vesc.set_current(0.0, mid)
                    faulted.add(mid)
                    continue

                current = sign * pids[mid].update(float(error), now)
                vesc.set_current(current, mid)

                label = "M" if mid is None else "S"
                parts.append(
                    f"[{label}] err={error:+4d} cmd={current:+5.2f}A act={vals['current_motor']:+5.2f}A"
                )

            # Print status at ~2Hz (avoid flooding terminal)
            if parts and now - last_print >= 0.5:
                print("  " + "  |  ".join(parts))
                last_print = now

            if len(faulted) >= len(homes):
                print("\nAll motors faulted or disabled — exiting.")
                break

            elapsed = time.monotonic() - t0
            if elapsed < period:
                time.sleep(period - elapsed)

    finally:
        print("\nReleasing motors (zero current)...")
        for mid in homes:
            vesc.set_current(0.0, mid)
            time.sleep(0.01)
        vesc.close()
        print("Done.")


if __name__ == "__main__":
    main()
