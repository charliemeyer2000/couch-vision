import struct

import pytest

from couch_ekf.cdr import CdrReader


def _make_cdr(*fields: bytes) -> bytes:
    return b"\x00\x01\x00\x00" + b"".join(fields)


@pytest.mark.parametrize(
    ("value", "fmt"),
    [(3.14, "<d"), (-273.15, "<d"), (0.0, "<d")],
)
def test_float64(value: float, fmt: str) -> None:
    data = _make_cdr(struct.pack(fmt, value))
    r = CdrReader(data)
    assert r.float64() == pytest.approx(value)


def test_int32() -> None:
    data = _make_cdr(struct.pack("<i", -42))
    r = CdrReader(data)
    assert r.int32() == -42


def test_string() -> None:
    s = "hello"
    encoded = s.encode("utf-8") + b"\x00"
    data = _make_cdr(struct.pack("<I", len(encoded)) + encoded)
    r = CdrReader(data)
    assert r.string() == "hello"


def test_alignment() -> None:
    data = _make_cdr(b"\x07" + b"\x00" * 7 + struct.pack("<d", 2.718))
    r = CdrReader(data)
    assert r.uint8() == 7
    assert r.float64() == pytest.approx(2.718)


def test_float64_array() -> None:
    values = [1.0, 2.0, 3.0]
    packed = b"".join(struct.pack("<d", v) for v in values)
    data = _make_cdr(packed)
    r = CdrReader(data)
    assert r.float64_array(3) == pytest.approx(values)
