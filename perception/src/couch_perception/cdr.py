from __future__ import annotations

import struct


class CdrReader:
    """XCDR1 little-endian CDR deserializer."""

    def __init__(self, data: bytes) -> None:
        self.data = data[4:]  # skip 4-byte encapsulation header
        self.offset = 0

    def _align(self, boundary: int) -> None:
        rem = self.offset % boundary
        if rem:
            self.offset += boundary - rem

    def _unpack(self, fmt: str, size: int, align: int = 0) -> int | float:
        if align:
            self._align(align)
        (val,) = struct.unpack_from(fmt, self.data, self.offset)
        self.offset += size
        return val  # type: ignore[no-any-return]

    def uint8(self) -> int:
        val = self.data[self.offset]
        self.offset += 1
        return val

    def int8(self) -> int:
        return int(self._unpack("<b", 1))

    def uint16(self) -> int:
        return int(self._unpack("<H", 2, align=2))

    def int32(self) -> int:
        return int(self._unpack("<i", 4, align=4))

    def uint32(self) -> int:
        return int(self._unpack("<I", 4, align=4))

    def float32(self) -> float:
        return float(self._unpack("<f", 4, align=4))

    def float64(self) -> float:
        return float(self._unpack("<d", 8, align=8))

    def string(self) -> str:
        length = self.uint32()
        val = self.data[self.offset : self.offset + length - 1].decode("utf-8")
        self.offset += length
        return val

    def float64_array(self, count: int) -> list[float]:
        return [self.float64() for _ in range(count)]
