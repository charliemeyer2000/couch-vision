import struct, time, serial
from serial.tools.list_ports import comports

COMM_SET_RPM = 8
COMM_GET_VALUES = 4
COMM_FORWARD_CAN = 34
SLAVE_CAN = 19
POLE_PAIRS = 7
TARGET_RPM = 200
TARGET_ERPM = TARGET_RPM * POLE_PAIRS  # 1400
DURATION = 5.0

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

def pkt(payload):
    c = crc16(payload)
    return bytes([0x02, len(payload)]) + payload + c.to_bytes(2, "big") + bytes([0x03])

def parse(buf):
    if len(buf) < 5 or buf[0] != 0x02: return None
    length = buf[1]
    if len(buf) < length + 5: return None
    p = buf[2:2+length]
    cr = struct.unpack(">H", buf[2+length:4+length])[0]
    return p if crc16(p) == cr else None

def read_pkt(s, timeout=0.3):
    buf = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        avail = s.in_waiting
        if avail:
            buf += s.read(avail)
            if len(buf) >= 5 and buf[0] == 0x02 and len(buf) >= buf[1] + 5:
                return buf
        else:
            time.sleep(0.002)
    return buf

port = None
for p in comports():
    if p.vid == 0x0483 and p.pid == 0x5740:
        port = p.device; break
assert port, "No VESC found"
s = serial.Serial(port, 115200, timeout=0.05)

def set_rpm(erpm, can_id=None):
    erpm_bytes = struct.pack(">i", erpm)
    if can_id is not None:
        s.write(pkt(bytes([COMM_FORWARD_CAN, can_id, COMM_SET_RPM]) + erpm_bytes))
    else:
        s.write(pkt(bytes([COMM_SET_RPM]) + erpm_bytes))

def get_values(can_id=None):
    s.reset_input_buffer()
    if can_id is not None:
        s.write(pkt(bytes([COMM_FORWARD_CAN, can_id, COMM_GET_VALUES])))
    else:
        s.write(pkt(bytes([COMM_GET_VALUES])))
    raw = read_pkt(s, 0.5)
    p = parse(raw)
    if p and len(p) >= 54 and p[0] == 4:
        d = p[1:]
        return {
            "erpm": struct.unpack(">i", d[22:26])[0],
            "duty": struct.unpack(">h", d[20:22])[0] / 1000.0,
            "i_motor": struct.unpack(">i", d[4:8])[0] / 100.0,
            "i_input": struct.unpack(">i", d[8:12])[0] / 100.0,
            "v_in": struct.unpack(">h", d[26:28])[0] / 10.0,
        }
    return None

print(f"Port: {port}")
print(f"Target: {TARGET_RPM} RPM = {TARGET_ERPM} ERPM for {DURATION}s\n")

print(f"{'t':>4s}  {'':>1s} {'ERPM':>6s} {'duty':>6s} {'I_mot':>6s} {'I_in':>6s} {'V':>5s}")
print("-" * 50)

start = time.monotonic()
while time.monotonic() - start < DURATION:
    set_rpm(TARGET_ERPM)               # master
    set_rpm(TARGET_ERPM, SLAVE_CAN)    # slave

    # Read master telemetry
    vm = get_values()
    vs = get_values(SLAVE_CAN)

    t = time.monotonic() - start
    if vm:
        print(f"{t:4.1f}  M {vm['erpm']:+6d} {vm['duty']:+6.3f} {vm['i_motor']:+6.2f}A {vm['i_input']:+6.2f}A {vm['v_in']:5.1f}V")
    if vs:
        print(f"      S {vs['erpm']:+6d} {vs['duty']:+6.3f} {vs['i_motor']:+6.2f}A {vs['i_input']:+6.2f}A {vs['v_in']:5.1f}V")

    time.sleep(0.3)

# Stop
zero = struct.pack(">i", 0)
s.write(pkt(bytes([COMM_SET_RPM]) + zero))
s.write(pkt(bytes([COMM_FORWARD_CAN, SLAVE_CAN, COMM_SET_RPM]) + zero))
print("\nStopped.")
s.close()
