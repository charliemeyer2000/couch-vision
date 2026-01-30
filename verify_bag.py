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
from io import BytesIO
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
from mcap.reader import make_reader


# ---------------------------------------------------------------------------
# CDR reader (inline copy from ekf/src/couch_ekf/cdr.py)
# ---------------------------------------------------------------------------

class CdrReader:
    def __init__(self, data: bytes) -> None:
        self.data = data[4:]  # skip 4-byte encapsulation header
        self.offset = 0

    def _align(self, boundary: int) -> None:
        rem = self.offset % boundary
        if rem:
            self.offset += boundary - rem

    def _unpack(self, fmt: str, size: int, align: int = 0):
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

    def bytes_seq(self) -> bytes:
        length = self.uint32()
        val = self.data[self.offset : self.offset + length]
        self.offset += length
        return val


# ---------------------------------------------------------------------------
# CDR parsers
# ---------------------------------------------------------------------------

def _parse_stamp(r: CdrReader) -> float:
    sec = r.int32()
    nsec = r.uint32()
    return sec + nsec * 1e-9


def _parse_imu(data: bytes) -> dict:
    r = CdrReader(data)
    t = _parse_stamp(r)
    _ = r.string()
    qx, qy, qz, qw = r.float64(), r.float64(), r.float64(), r.float64()
    _ = r.float64_array(9)
    wx, wy, wz = r.float64(), r.float64(), r.float64()
    _ = r.float64_array(9)
    ax, ay, az = r.float64(), r.float64(), r.float64()
    return {"t": t, "qx": qx, "qy": qy, "qz": qz, "qw": qw,
            "wx": wx, "wy": wy, "wz": wz, "ax": ax, "ay": ay, "az": az}


def _parse_gps(data: bytes) -> dict:
    r = CdrReader(data)
    t = _parse_stamp(r)
    _ = r.string()
    _ = r.int8()   # status
    _ = r.uint16() # service
    lat, lon, alt = r.float64(), r.float64(), r.float64()
    return {"t": t, "lat": lat, "lon": lon, "alt": alt}


def _parse_odom(data: bytes) -> dict:
    r = CdrReader(data)
    t = _parse_stamp(r)
    _ = r.string()  # frame_id
    _ = r.string()  # child_frame_id
    x, y, z = r.float64(), r.float64(), r.float64()
    qx, qy, qz, qw = r.float64(), r.float64(), r.float64(), r.float64()
    return {"t": t, "x": x, "y": y, "z": z, "qx": qx, "qy": qy, "qz": qz, "qw": qw}


def _parse_compressed_image(data: bytes) -> bytes:
    r = CdrReader(data)
    _ = _parse_stamp(r)
    _ = r.string()  # frame_id
    _ = r.string()  # format
    return r.bytes_seq()


# ---------------------------------------------------------------------------
# Hz statistics
# ---------------------------------------------------------------------------

def hz_stats(timestamps_ns: list[int]) -> dict:
    n = len(timestamps_ns)
    if n < 2:
        return {"count": n, "duration_s": 0, "hz_eff": 0,
                "hz_median": 0, "max_gap_s": 0}
    duration_s = (timestamps_ns[-1] - timestamps_ns[0]) * 1e-9
    ts = np.array(timestamps_ns, dtype=np.float64)
    diffs = np.diff(ts) * 1e-9
    diffs = diffs[diffs > 0]
    if len(diffs) == 0 or duration_s == 0:
        return {"count": n, "duration_s": 0, "hz_eff": 0,
                "hz_median": 0, "max_gap_s": 0}
    hz_eff = (n - 1) / duration_s  # effective rate: total intervals / total time
    hz_median = float(np.median(1.0 / diffs))
    return {
        "count": n,
        "duration_s": duration_s,
        "hz_eff": float(hz_eff),
        "hz_median": hz_median,
        "max_gap_s": float(np.max(diffs)),
    }


# ---------------------------------------------------------------------------
# Bag reader — single pass
# ---------------------------------------------------------------------------

def read_bag(path: Path):
    timestamps: dict[str, list[int]] = defaultdict(list)
    imu_data: list[dict] = []
    gps_data: list[dict] = []
    odom_data: list[dict] = []
    camera_frames: list[bytes] = []  # sampled JPEG bytes
    camera_count = 0
    frame_sample_interval = 30  # save every Nth frame

    with open(path, "rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages():
            if channel is None:
                continue
            topic = channel.topic
            timestamps[topic].append(message.log_time)

            try:
                if topic.endswith("/imu"):
                    imu_data.append(_parse_imu(message.data))
                elif topic.endswith("/gps/fix"):
                    gps_data.append(_parse_gps(message.data))
                elif topic.endswith("/odom"):
                    odom_data.append(_parse_odom(message.data))
                elif topic.endswith("/image/compressed"):
                    camera_count += 1
                    if camera_count % frame_sample_interval == 1:
                        camera_frames.append(_parse_compressed_image(message.data))
            except Exception:
                pass  # skip malformed messages

    return timestamps, imu_data, gps_data, odom_data, camera_frames


# ---------------------------------------------------------------------------
# Pass/fail checks
# ---------------------------------------------------------------------------

CHECKS = [
    ("IMU Hz > 50", lambda stats: any(
        s["hz_eff"] > 50 for t, s in stats.items() if t.endswith("/imu"))),
    ("Camera Hz > 5", lambda stats: any(
        s["hz_eff"] > 5 for t, s in stats.items()
        if t.endswith("/image/compressed") or t.endswith("/image"))),
    ("GPS messages > 0", lambda stats: any(
        s["count"] > 0 for t, s in stats.items() if t.endswith("/gps/fix"))),
    ("Odom messages > 0", lambda stats: any(
        s["count"] > 0 for t, s in stats.items() if t.endswith("/odom"))),
    ("TF messages > 0", lambda stats: any(
        s["count"] > 0 for t, s in stats.items() if t == "/tf")),
    ("IMU max gap < 2s", lambda stats: all(
        s["max_gap_s"] < 2.0 for t, s in stats.items() if t.endswith("/imu"))),
]


# ---------------------------------------------------------------------------
# Report generation
# ---------------------------------------------------------------------------

def generate_report(
    out_dir: Path,
    all_stats: dict[str, dict],
    imu_data: list[dict],
    gps_data: list[dict],
    odom_data: list[dict],
    camera_frames: list[bytes],
):
    out_dir.mkdir(parents=True, exist_ok=True)
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(exist_ok=True)

    # Save sample frames
    for i, jpeg in enumerate(camera_frames):
        (frames_dir / f"frame_{i:04d}.jpg").write_bytes(jpeg)

    # Summary text
    lines = ["CouchVision Bag Verification Report", "=" * 40, ""]
    lines.append(f"{'Topic':<55} {'Count':>6} {'Hz eff':>8} {'Hz med':>8} {'Max gap':>8}")
    lines.append("-" * 90)
    for topic in sorted(all_stats):
        s = all_stats[topic]
        lines.append(
            f"{topic:<55} {s['count']:>6} {s['hz_eff']:>8.1f} {s['hz_median']:>8.1f} "
            f"{s['max_gap_s']:>7.3f}s"
        )
    lines.append("")
    lines.append("Pass/Fail Checks:")
    for name, check_fn in CHECKS:
        passed = check_fn(all_stats)
        lines.append(f"  [{'PASS' if passed else 'FAIL'}] {name}")
    summary_text = "\n".join(lines)
    (out_dir / "summary.txt").write_text(summary_text)
    print(summary_text)

    # PDF report
    pdf_path = out_dir / "report.pdf"
    with PdfPages(str(pdf_path)) as pdf:
        # Page 1: Summary table
        fig, ax = plt.subplots(figsize=(14, 8))
        ax.axis("off")
        ax.set_title("Bag Verification Summary", fontsize=16, fontweight="bold")
        rows = []
        for topic in sorted(all_stats):
            s = all_stats[topic]
            rows.append([
                topic,
                str(s["count"]),
                f"{s['hz_eff']:.1f}",
                f"{s['hz_median']:.1f}",
                f"{s['max_gap_s']:.3f}",
            ])
        if rows:
            table = ax.table(
                cellText=rows,
                colLabels=["Topic", "Count", "Hz (effective)", "Hz (median)", "Max gap (s)"],
                loc="center",
                cellLoc="right",
            )
            table.auto_set_font_size(False)
            table.set_fontsize(8)
            table.scale(1, 1.4)
            for (row, col), cell in table.get_celld().items():
                if row == 0:
                    cell.set_facecolor("#4472C4")
                    cell.set_text_props(color="white", fontweight="bold")
                cell.set_edgecolor("#CCCCCC")
        plt.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        # Page 2: IMU plots
        if imu_data:
            t0 = imu_data[0]["t"]
            t = np.array([s["t"] - t0 for s in imu_data])

            fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
            fig.suptitle("IMU Data", fontsize=14, fontweight="bold")

            ax = axes[0]
            ax.set_title("Orientation (quaternion)")
            for key, label in [("qx", "qx"), ("qy", "qy"), ("qz", "qz"), ("qw", "qw")]:
                ax.plot(t, [s[key] for s in imu_data], label=label, linewidth=0.5)
            ax.legend(loc="upper right")
            ax.set_ylabel("value")

            ax = axes[1]
            ax.set_title("Angular velocity")
            for key, label in [("wx", "wx"), ("wy", "wy"), ("wz", "wz")]:
                ax.plot(t, [s[key] for s in imu_data], label=label, linewidth=0.5)
            ax.legend(loc="upper right")
            ax.set_ylabel("rad/s")

            ax = axes[2]
            ax.set_title("Linear acceleration")
            for key, label in [("ax", "ax"), ("ay", "ay"), ("az", "az")]:
                ax.plot(t, [s[key] for s in imu_data], label=label, linewidth=0.5)
            ax.legend(loc="upper right")
            ax.set_ylabel("m/s²")
            ax.set_xlabel("time (s)")

            plt.tight_layout()
            pdf.savefig(fig)
            plt.close(fig)

        # Page 3: GPS scatter
        if gps_data:
            fig, ax = plt.subplots(figsize=(10, 10))
            fig.suptitle("GPS Fixes", fontsize=14, fontweight="bold")
            lats = [s["lat"] for s in gps_data]
            lons = [s["lon"] for s in gps_data]
            t0 = gps_data[0]["t"]
            colors = [s["t"] - t0 for s in gps_data]
            sc = ax.scatter(lons, lats, c=colors, cmap="viridis", s=10)
            ax.set_xlabel("Longitude")
            ax.set_ylabel("Latitude")
            plt.colorbar(sc, ax=ax, label="time (s)")
            ax.set_aspect("equal")
            plt.tight_layout()
            pdf.savefig(fig)
            plt.close(fig)

        # Page 4: Odometry trail
        if odom_data:
            fig, ax = plt.subplots(figsize=(10, 10))
            fig.suptitle("Odometry Trail", fontsize=14, fontweight="bold")
            xs = [s["x"] for s in odom_data]
            ys = [s["y"] for s in odom_data]
            t0 = odom_data[0]["t"]
            colors = [s["t"] - t0 for s in odom_data]
            sc = ax.scatter(xs, ys, c=colors, cmap="viridis", s=2)
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

        # Page 5: Sample camera frames
        if camera_frames:
            n = min(len(camera_frames), 6)
            cols = 3
            rows = (n + cols - 1) // cols
            fig, axes = plt.subplots(rows, cols, figsize=(14, 4 * rows))
            fig.suptitle("Sample Camera Frames", fontsize=14, fontweight="bold")
            if rows == 1:
                axes = [axes] if cols == 1 else [axes]
            axes_flat = np.array(axes).flatten()
            for i, ax in enumerate(axes_flat):
                if i < n:
                    img = plt.imread(BytesIO(camera_frames[i]), format="jpeg")
                    ax.imshow(img)
                    ax.set_title(f"Frame {i}")
                ax.axis("off")
            plt.tight_layout()
            pdf.savefig(fig)
            plt.close(fig)

        # Page 6: Hz histograms for key topics
        fig, axes = plt.subplots(1, 2, figsize=(14, 5))
        fig.suptitle("Frequency Distribution", fontsize=14, fontweight="bold")

        for ax, suffix, label in [
            (axes[0], "/imu", "IMU"),
            (axes[1], "/image/compressed", "Camera (compressed)"),
        ]:
            for topic, ts_list in sorted(timestamps_cache.items()):
                if topic.endswith(suffix) and len(ts_list) > 2:
                    diffs = np.diff(np.array(ts_list, dtype=np.float64)) * 1e-9
                    hz = 1.0 / diffs[diffs > 0]
                    ax.hist(hz, bins=50, alpha=0.7, label=topic.split("/")[-2] if "/" in topic else topic)
            ax.set_title(f"{label} Hz")
            ax.set_xlabel("Hz")
            ax.set_ylabel("count")
            ax.legend(fontsize=7)

        plt.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

    print(f"\nReport saved to {pdf_path}")
    print(f"Sample frames saved to {frames_dir}/")
    print(f"Summary saved to {out_dir / 'summary.txt'}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

# Module-level cache for timestamps (used by Hz histogram page)
timestamps_cache: dict[str, list[int]] = {}


def main():
    global timestamps_cache

    parser = argparse.ArgumentParser(description="Verify CouchVision bag data quality")
    parser.add_argument("bag", type=Path, help="Path to .mcap bag file")
    parser.add_argument("-o", "--output", type=Path, default=None,
                        help="Output directory (default: verify_output/<bagname>)")
    args = parser.parse_args()

    bag_path = args.bag
    if not bag_path.exists():
        print(f"Error: {bag_path} not found")
        sys.exit(1)

    bag_name = bag_path.stem
    out_dir = args.output or Path("verify_output") / bag_name

    print(f"Reading {bag_path}...")
    timestamps, imu_data, gps_data, odom_data, camera_frames = read_bag(bag_path)
    timestamps_cache = timestamps

    all_stats = {topic: hz_stats(ts) for topic, ts in timestamps.items()}

    print(f"Parsed {sum(s['count'] for s in all_stats.values())} messages across {len(all_stats)} topics")
    print(f"IMU: {len(imu_data)}, GPS: {len(gps_data)}, Odom: {len(odom_data)}, Camera frames saved: {len(camera_frames)}")
    print()

    generate_report(out_dir, all_stats, imu_data, gps_data, odom_data, camera_frames)


if __name__ == "__main__":
    main()
