"""Phase 1 data quality verification for CouchVision bag files.

Reads an MCAP bag, checks Hz/message counts for all sensor topics,
and generates a PDF report with IMU plots, GPS scatter, odometry trail,
and sample camera frames.

Usage:
    uv run --with mcap,numpy,matplotlib verify_bag.py <bag_path>
    # or: make verify BAG=path/to/file.mcap
"""

from __future__ import annotations

import argparse
import struct
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from io import BytesIO
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
import numpy as np  # noqa: E402
from mcap.reader import make_reader  # noqa: E402


# ---------------------------------------------------------------------------
# CDR reader (inline copy from ekf/src/couch_ekf/cdr.py)
# ---------------------------------------------------------------------------

class CdrReader:
    def __init__(self, data: bytes) -> None:
        self.data = data[4:]
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
        return val

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

    def float64(self) -> float:
        return float(self._unpack("<d", 8, align=8))

    def string(self) -> str:
        length = self.uint32()
        val = self.data[self.offset : self.offset + length - 1].decode("utf-8")
        self.offset += length
        return val

    def float64_array(self, count: int) -> list[float]:
        return [self.float64() for _ in range(count)]

    def bytes_seq(self) -> bytes:
        length = self.uint32()
        val = self.data[self.offset : self.offset + length]
        self.offset += length
        return val


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class ImuSample:
    t: float
    qx: float
    qy: float
    qz: float
    qw: float
    wx: float
    wy: float
    wz: float
    ax: float
    ay: float
    az: float


@dataclass(slots=True)
class GpsFix:
    t: float
    lat: float
    lon: float
    alt: float


@dataclass(slots=True)
class OdomSample:
    t: float
    x: float
    y: float
    z: float


@dataclass(slots=True)
class HzStats:
    count: int
    duration_s: float
    hz_eff: float
    hz_median: float
    max_gap_s: float


@dataclass
class BagData:
    timestamps: dict[str, list[int]] = field(default_factory=lambda: defaultdict(list))
    imu: list[ImuSample] = field(default_factory=list)
    gps: list[GpsFix] = field(default_factory=list)
    odom: list[OdomSample] = field(default_factory=list)
    camera_frames: list[bytes] = field(default_factory=list)


# ---------------------------------------------------------------------------
# CDR parsers
# ---------------------------------------------------------------------------

def _parse_stamp(r: CdrReader) -> float:
    sec = r.int32()
    nsec = r.uint32()
    return sec + nsec * 1e-9


def _skip_header(r: CdrReader) -> float:
    t = _parse_stamp(r)
    r.string()  # frame_id
    return t


def _parse_imu(data: bytes) -> ImuSample:
    r = CdrReader(data)
    t = _skip_header(r)
    qx, qy, qz, qw = r.float64(), r.float64(), r.float64(), r.float64()
    r.float64_array(9)  # orientation covariance
    wx, wy, wz = r.float64(), r.float64(), r.float64()
    r.float64_array(9)  # angular velocity covariance
    ax, ay, az = r.float64(), r.float64(), r.float64()
    return ImuSample(t=t, qx=qx, qy=qy, qz=qz, qw=qw,
                     wx=wx, wy=wy, wz=wz, ax=ax, ay=ay, az=az)


def _parse_gps(data: bytes) -> GpsFix:
    r = CdrReader(data)
    t = _skip_header(r)
    r.int8()    # status
    r.uint16()  # service
    lat, lon, alt = r.float64(), r.float64(), r.float64()
    return GpsFix(t=t, lat=lat, lon=lon, alt=alt)


def _parse_odom(data: bytes) -> OdomSample:
    r = CdrReader(data)
    t = _skip_header(r)
    r.string()  # child_frame_id
    x, y, z = r.float64(), r.float64(), r.float64()
    return OdomSample(t=t, x=x, y=y, z=z)


def _parse_compressed_image(data: bytes) -> bytes:
    r = CdrReader(data)
    _parse_stamp(r)
    r.string()  # frame_id
    r.string()  # format
    return r.bytes_seq()


# ---------------------------------------------------------------------------
# Hz statistics
# ---------------------------------------------------------------------------

def compute_hz_stats(timestamps_ns: list[int]) -> HzStats:
    n = len(timestamps_ns)
    if n < 2:
        return HzStats(count=n, duration_s=0, hz_eff=0, hz_median=0, max_gap_s=0)
    duration_s = (timestamps_ns[-1] - timestamps_ns[0]) * 1e-9
    diffs = np.diff(np.array(timestamps_ns, dtype=np.float64)) * 1e-9
    diffs = diffs[diffs > 0]
    if len(diffs) == 0 or duration_s == 0:
        return HzStats(count=n, duration_s=0, hz_eff=0, hz_median=0, max_gap_s=0)
    return HzStats(
        count=n,
        duration_s=duration_s,
        hz_eff=(n - 1) / duration_s,
        hz_median=float(np.median(1.0 / diffs)),
        max_gap_s=float(np.max(diffs)),
    )


# ---------------------------------------------------------------------------
# Bag reader
# ---------------------------------------------------------------------------

FRAME_SAMPLE_INTERVAL = 30


def read_bag(path: Path) -> BagData:
    bag = BagData()
    camera_count = 0

    with open(path, "rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages():
            if channel is None:
                continue
            topic = channel.topic
            bag.timestamps[topic].append(message.log_time)

            try:
                if topic.endswith("/imu"):
                    bag.imu.append(_parse_imu(message.data))
                elif topic.endswith("/gps/fix"):
                    bag.gps.append(_parse_gps(message.data))
                elif topic.endswith("/odom"):
                    bag.odom.append(_parse_odom(message.data))
                elif topic.endswith("/image/compressed"):
                    camera_count += 1
                    if camera_count % FRAME_SAMPLE_INTERVAL == 1:
                        bag.camera_frames.append(_parse_compressed_image(message.data))
            except (struct.error, IndexError, UnicodeDecodeError):
                continue

    return bag


# ---------------------------------------------------------------------------
# Pass/fail checks
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class CheckResult:
    name: str
    passed: bool


def _has_topic(stats: dict[str, HzStats], suffix: str, *, min_hz: float = 0) -> bool:
    return any(
        s.hz_eff > min_hz if min_hz else s.count > 0
        for t, s in stats.items()
        if t.endswith(suffix)
    )


def run_checks(stats: dict[str, HzStats]) -> list[CheckResult]:
    return [
        CheckResult("IMU Hz > 50", _has_topic(stats, "/imu", min_hz=50)),
        CheckResult("Camera Hz > 5", (
            _has_topic(stats, "/image/compressed", min_hz=5)
            or _has_topic(stats, "/image", min_hz=5)
        )),
        CheckResult("GPS messages > 0", _has_topic(stats, "/gps/fix")),
        CheckResult("Odom messages > 0", _has_topic(stats, "/odom")),
        CheckResult("TF messages > 0", any(s.count > 0 for t, s in stats.items() if t == "/tf")),
        CheckResult("IMU max gap < 2s", all(
            s.max_gap_s < 2.0 for t, s in stats.items() if t.endswith("/imu")
        )),
    ]


# ---------------------------------------------------------------------------
# Report: text summary
# ---------------------------------------------------------------------------

def write_summary(
    out_dir: Path,
    stats: dict[str, HzStats],
    checks: list[CheckResult],
) -> str:
    lines = ["CouchVision Bag Verification Report", "=" * 40, ""]
    lines.append(f"{'Topic':<55} {'Count':>6} {'Hz eff':>8} {'Hz med':>8} {'Max gap':>8}")
    lines.append("-" * 90)
    for topic in sorted(stats):
        s = stats[topic]
        lines.append(
            f"{topic:<55} {s.count:>6} {s.hz_eff:>8.1f} {s.hz_median:>8.1f} "
            f"{s.max_gap_s:>7.3f}s"
        )
    lines.append("")
    lines.append("Pass/Fail Checks:")
    for c in checks:
        lines.append(f"  [{'PASS' if c.passed else 'FAIL'}] {c.name}")

    text = "\n".join(lines)
    (out_dir / "summary.txt").write_text(text)
    return text


# ---------------------------------------------------------------------------
# Report: PDF pages
# ---------------------------------------------------------------------------

def _page_summary_table(pdf: PdfPages, stats: dict[str, HzStats]) -> None:
    fig, ax = plt.subplots(figsize=(14, 8))
    ax.axis("off")
    ax.set_title("Bag Verification Summary", fontsize=16, fontweight="bold")
    rows = [
        [topic, str(s.count), f"{s.hz_eff:.1f}", f"{s.hz_median:.1f}", f"{s.max_gap_s:.3f}"]
        for topic, s in sorted(stats.items())
    ]
    if rows:
        table = ax.table(
            cellText=rows,
            colLabels=["Topic", "Count", "Hz (effective)", "Hz (median)", "Max gap (s)"],
            loc="center", cellLoc="right",
        )
        table.auto_set_font_size(False)
        table.set_fontsize(8)
        table.scale(1, 1.4)
        for (row, _col), cell in table.get_celld().items():
            if row == 0:
                cell.set_facecolor("#4472C4")
                cell.set_text_props(color="white", fontweight="bold")
            cell.set_edgecolor("#CCCCCC")
    plt.tight_layout()
    pdf.savefig(fig)
    plt.close(fig)


def _page_imu(pdf: PdfPages, imu: list[ImuSample]) -> None:
    if not imu:
        return
    t0 = imu[0].t
    t = np.array([s.t - t0 for s in imu])

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle("IMU Data", fontsize=14, fontweight="bold")

    groups: list[tuple[int, str, list[tuple[str, str]], str]] = [
        (0, "Orientation (quaternion)", [("qx", "qx"), ("qy", "qy"), ("qz", "qz"), ("qw", "qw")], "value"),
        (1, "Angular velocity", [("wx", "wx"), ("wy", "wy"), ("wz", "wz")], "rad/s"),
        (2, "Linear acceleration", [("ax", "ax"), ("ay", "ay"), ("az", "az")], "m/s²"),
    ]
    for idx, title, fields, ylabel in groups:
        ax = axes[idx]
        ax.set_title(title)
        for attr, label in fields:
            ax.plot(t, [getattr(s, attr) for s in imu], label=label, linewidth=0.5)
        ax.legend(loc="upper right")
        ax.set_ylabel(ylabel)
    axes[2].set_xlabel("time (s)")

    plt.tight_layout()
    pdf.savefig(fig)
    plt.close(fig)


def _page_gps(pdf: PdfPages, gps: list[GpsFix]) -> None:
    if not gps:
        return
    t0 = gps[0].t
    fig, ax = plt.subplots(figsize=(10, 10))
    fig.suptitle("GPS Fixes", fontsize=14, fontweight="bold")
    sc = ax.scatter(
        [s.lon for s in gps], [s.lat for s in gps],
        c=[s.t - t0 for s in gps], cmap="viridis", s=10,
    )
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    plt.colorbar(sc, ax=ax, label="time (s)")
    ax.set_aspect("equal")
    plt.tight_layout()
    pdf.savefig(fig)
    plt.close(fig)


def _page_odom(pdf: PdfPages, odom: list[OdomSample]) -> None:
    if not odom:
        return
    t0 = odom[0].t
    xs = [s.x for s in odom]
    ys = [s.y for s in odom]
    fig, ax = plt.subplots(figsize=(10, 10))
    fig.suptitle("Odometry Trail", fontsize=14, fontweight="bold")
    sc = ax.scatter(xs, ys, c=[s.t - t0 for s in odom], cmap="viridis", s=2)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    plt.colorbar(sc, ax=ax, label="time (s)")
    ax.set_aspect("equal")
    ax.plot(xs[0], ys[0], "go", markersize=10, label="start")
    ax.plot(xs[-1], ys[-1], "ro", markersize=10, label="end")
    ax.legend()
    plt.tight_layout()
    pdf.savefig(fig)
    plt.close(fig)


def _page_camera(pdf: PdfPages, frames: list[bytes]) -> None:
    if not frames:
        return
    n = min(len(frames), 6)
    cols = 3
    nrows = (n + cols - 1) // cols
    fig, axes = plt.subplots(nrows, cols, figsize=(14, 4 * nrows))
    fig.suptitle("Sample Camera Frames", fontsize=14, fontweight="bold")
    axes_flat = np.array(axes).flatten()
    for i, ax in enumerate(axes_flat):
        if i < n:
            ax.imshow(plt.imread(BytesIO(frames[i]), format="jpeg"))
            ax.set_title(f"Frame {i}")
        ax.axis("off")
    plt.tight_layout()
    pdf.savefig(fig)
    plt.close(fig)


def _page_hz_histograms(pdf: PdfPages, timestamps: dict[str, list[int]]) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("Frequency Distribution", fontsize=14, fontweight="bold")

    for ax, suffix, label in [(axes[0], "/imu", "IMU"), (axes[1], "/image/compressed", "Camera (compressed)")]:
        for topic in sorted(timestamps):
            ts_list = timestamps[topic]
            if topic.endswith(suffix) and len(ts_list) > 2:
                diffs = np.diff(np.array(ts_list, dtype=np.float64)) * 1e-9
                hz = 1.0 / diffs[diffs > 0]
                ax.hist(hz, bins=50, alpha=0.7)
        ax.set_title(f"{label} Hz")
        ax.set_xlabel("Hz")
        ax.set_ylabel("count")

    plt.tight_layout()
    pdf.savefig(fig)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Report: orchestrator
# ---------------------------------------------------------------------------

def generate_report(
    out_dir: Path,
    bag: BagData,
    stats: dict[str, HzStats],
    checks: list[CheckResult],
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(exist_ok=True)

    for i, jpeg in enumerate(bag.camera_frames):
        (frames_dir / f"frame_{i:04d}.jpg").write_bytes(jpeg)

    summary = write_summary(out_dir, stats, checks)
    print(summary)

    pdf_path = out_dir / "report.pdf"
    with PdfPages(str(pdf_path)) as pdf:
        _page_summary_table(pdf, stats)
        _page_imu(pdf, bag.imu)
        _page_gps(pdf, bag.gps)
        _page_odom(pdf, bag.odom)
        _page_camera(pdf, bag.camera_frames)
        _page_hz_histograms(pdf, bag.timestamps)

    print(f"\nReport saved to {pdf_path}")
    print(f"Sample frames saved to {frames_dir}/")
    print(f"Summary saved to {out_dir / 'summary.txt'}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Verify CouchVision bag data quality")
    parser.add_argument("bag", type=Path, help="Path to .mcap bag file")
    parser.add_argument("-o", "--output", type=Path, default=None,
                        help="Output directory (default: verify_output/<bagname>)")
    args = parser.parse_args()

    bag_path: Path = args.bag
    if not bag_path.exists():
        print(f"Error: {bag_path} not found")
        sys.exit(1)

    out_dir = args.output or Path("verify_output") / bag_path.stem

    print(f"Reading {bag_path}...")
    bag = read_bag(bag_path)

    stats = {topic: compute_hz_stats(ts) for topic, ts in bag.timestamps.items()}
    total = sum(s.count for s in stats.values())
    print(f"Parsed {total} messages across {len(stats)} topics")
    print(f"IMU: {len(bag.imu)}, GPS: {len(bag.gps)}, Odom: {len(bag.odom)}, Camera frames saved: {len(bag.camera_frames)}")
    print()

    checks = run_checks(stats)
    generate_report(out_dir, bag, stats, checks)


if __name__ == "__main__":
    main()
