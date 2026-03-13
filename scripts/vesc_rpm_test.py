import argparse
import struct
import time

import serial
from serial.tools.list_ports import comports

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


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def pkt(payload):
    crc = crc16(payload)
    return bytes([0x02, len(payload)]) + payload + crc.to_bytes(2, "big") + bytes([0x03])


def parse(buf):
    if len(buf) < 5 or buf[0] != 0x02:
        return None
    length = buf[1]
    if len(buf) < length + 5:
        return None
    payload = buf[2 : 2 + length]
    crc = struct.unpack(">H", buf[2 + length : 4 + length])[0]
    return payload if crc16(payload) == crc else None


def read_pkt(serial_port, timeout=0.08):
    buf = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        available = serial_port.in_waiting
        if available:
            buf += serial_port.read(available)
            if len(buf) >= 5 and buf[0] == 0x02 and len(buf) >= buf[1] + 5:
                return buf
        else:
            time.sleep(0.002)
    return buf


def find_vesc_port():
    for port in comports():
        if port.vid == 0x0483 and port.pid == 0x5740:
            return port.device
    raise RuntimeError("No VESC found")


def set_rpm(serial_port, erpm, can_id=None):
    payload = struct.pack(">i", erpm)
    if can_id is not None:
        serial_port.write(pkt(bytes([COMM_FORWARD_CAN, can_id, COMM_SET_RPM]) + payload))
        return
    serial_port.write(pkt(bytes([COMM_SET_RPM]) + payload))


def get_values(serial_port, can_id=None):
    serial_port.reset_input_buffer()
    if can_id is not None:
        serial_port.write(pkt(bytes([COMM_FORWARD_CAN, can_id, COMM_GET_VALUES])))
    else:
        serial_port.write(pkt(bytes([COMM_GET_VALUES])))
    payload = parse(read_pkt(serial_port))
    if payload and len(payload) >= 54 and payload[0] == COMM_GET_VALUES:
        data = payload[1:]
        fault_code = data[52]
        return {
            "erpm": struct.unpack(">i", data[22:26])[0],
            "duty": struct.unpack(">h", data[20:22])[0] / 1000.0,
            "i_motor": struct.unpack(">i", data[4:8])[0] / 100.0,
            "i_input": struct.unpack(">i", data[8:12])[0] / 100.0,
            "v_in": struct.unpack(">h", data[26:28])[0] / 10.0,
            "fault_code": fault_code,
            "fault_name": FAULT_NAMES.get(fault_code, "UNKNOWN"),
        }
    return None


def controller_targets(mode, slave_can):
    if mode == "master":
        return [("M", None)]
    if mode == "slave":
        return [("S", slave_can)]
    return [("M", None), ("S", slave_can)]


def parse_args():
    parser = argparse.ArgumentParser(description="Send RPM commands to one or both VESC controllers.")
    parser.add_argument("--target-rpm", type=int, default=400)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--pole-pairs", type=int, default=7)
    parser.add_argument("--slave-can", type=int, default=19)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--controllers",
        choices=("master", "slave", "both"),
        default="both",
        help="Use 'master' or 'slave' to verify whether the supply only trips when both sides are active.",
    )
    parser.add_argument("--sample-period", type=float, default=0.3)
    return parser.parse_args()


def main():
    args = parse_args()
    port = find_vesc_port()
    serial_port = serial.Serial(port, args.baud, timeout=0.02)
    targets = controller_targets(args.controllers, args.slave_can)
    target_erpm = args.target_rpm * args.pole_pairs

    print(f"Port: {port}")
    print(
        f"Target: {args.target_rpm} RPM = {target_erpm} ERPM for {args.duration}s "
        f"on {args.controllers}\n"
    )
    print(f"{'t':>4s}  {'':>1s} {'ERPM':>6s} {'duty':>6s} {'I_mot':>7s} {'I_in':>7s} {'V':>5s}  fault")
    print("-" * 72)

    try:
        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            for _, can_id in targets:
                set_rpm(serial_port, target_erpm, can_id)

            t = time.monotonic() - start
            for label, can_id in targets:
                values = get_values(serial_port, can_id)
                if values is None:
                    print(f"{t:4.1f}  {label} {'--':>6s} {'--':>6s} {'--':>7s} {'--':>7s} {'--':>5s}  NO_DATA")
                    continue
                print(
                    f"{t:4.1f}  {label} "
                    f"{values['erpm']:+6d} "
                    f"{values['duty']:+6.3f} "
                    f"{values['i_motor']:+7.2f}A "
                    f"{values['i_input']:+7.2f}A "
                    f"{values['v_in']:5.1f}V  "
                    f"{values['fault_name']}"
                )
            time.sleep(args.sample_period)
    finally:
        for _, can_id in targets:
            set_rpm(serial_port, 0, can_id)
        serial_port.close()
        print("\nStopped.")


if __name__ == "__main__":
    main()
