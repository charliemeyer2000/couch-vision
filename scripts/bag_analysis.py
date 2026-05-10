# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "mcap>=1.1.0",
#     "mcap-ros2-support>=0.5.0",
#     "matplotlib>=3.8.0",
#     "numpy>=1.26.0",
# ]
# ///
"""Analyze Friday (May 8) rosbag data for control diagnostics."""
from __future__ import annotations

import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


# ── bag paths ────────────────────────────────────────────────────────────────
BAGS_DIR = Path(__file__).resolve().parent.parent / "bags"

FRIDAY_BAGS = [
    "2026-05-08_16-17-08",
    "2026-05-08_17-03-18",
    "2026-05-08_17-16-36",
    "2026-05-08_17-25-28",
    "2026-05-08_18-25-49",
]

# Older baseline bag with motor data (24V era)
BASELINE_BAG = "2026-03-17_14-52-17"


@dataclass
class BagData:
    name: str
    # cmd_vel: [(t, linear_x, angular_z), ...]
    cmd_vel: list[tuple[float, float, float]] = field(default_factory=list)
    # wheel_odom: [(t, linear_x, angular_z), ...]
    wheel_odom: list[tuple[float, float, float]] = field(default_factory=list)
    # motor_status: [(t, json_dict), ...]
    motor_status: list[tuple[float, dict]] = field(default_factory=list)
    # teleop_status: [(t, json_dict), ...]
    teleop_status: list[tuple[float, dict]] = field(default_factory=list)
    # motor_config: [(t, json_dict), ...]
    motor_config: list[tuple[float, dict]] = field(default_factory=list)
    # motor_battery: [(t, voltage, current, percentage), ...]
    motor_battery: list[tuple[float, float, float, float]] = field(default_factory=list)


def _ts(msg) -> float:
    """Extract wall-clock seconds from a ROS2 message header or log_time."""
    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return 0.0


def load_bag(bag_name: str) -> BagData:
    bag_dir = BAGS_DIR / bag_name
    mcap_files = list(bag_dir.glob("*.mcap"))
    if not mcap_files:
        # try .db3
        mcap_files = list(bag_dir.glob("*.db3"))
    if not mcap_files:
        print(f"  [SKIP] No mcap/db3 in {bag_dir}")
        return BagData(name=bag_name)

    data = BagData(name=bag_name)
    for mcap_path in mcap_files:
        print(f"  Reading {mcap_path.name} ...")
        try:
            with open(mcap_path, "rb") as f:
                reader = make_reader(f, decoder_factories=[DecoderFactory()])
                for schema, channel, message, decoded in reader.iter_decoded_messages():
                    topic = channel.topic
                    t_sec = message.log_time * 1e-9  # nanoseconds -> seconds

                    if topic == "/cmd_vel":
                        data.cmd_vel.append((
                            t_sec,
                            decoded.linear.x,
                            decoded.angular.z,
                        ))
                    elif topic == "/wheel_odom":
                        data.wheel_odom.append((
                            t_sec,
                            decoded.twist.twist.linear.x,
                            decoded.twist.twist.angular.z,
                        ))
                    elif topic == "/motor/status":
                        try:
                            js = json.loads(decoded.data)
                            data.motor_status.append((t_sec, js))
                        except json.JSONDecodeError:
                            pass
                    elif topic == "/teleop/status":
                        try:
                            js = json.loads(decoded.data)
                            data.teleop_status.append((t_sec, js))
                        except json.JSONDecodeError:
                            pass
                    elif topic == "/motor/config":
                        try:
                            js = json.loads(decoded.data)
                            data.motor_config.append((t_sec, js))
                        except json.JSONDecodeError:
                            pass
                    elif topic == "/motor/battery":
                        data.motor_battery.append((
                            t_sec,
                            decoded.voltage,
                            decoded.current,
                            decoded.percentage,
                        ))
        except Exception as e:
            print(f"  [ERROR] {e}")

    return data


def print_bag_summary(data: BagData) -> None:
    print(f"\n{'='*70}")
    print(f"BAG: {data.name}")
    print(f"{'='*70}")

    # Duration
    all_ts = []
    for series in [data.cmd_vel, data.wheel_odom]:
        if series:
            all_ts.extend([s[0] for s in series])
    for series in [data.motor_status, data.teleop_status]:
        if series:
            all_ts.extend([s[0] for s in series])
    if all_ts:
        dur = max(all_ts) - min(all_ts)
        print(f"Duration: {dur:.1f}s")

    print(f"cmd_vel msgs:      {len(data.cmd_vel)}")
    print(f"wheel_odom msgs:   {len(data.wheel_odom)}")
    print(f"motor_status msgs: {len(data.motor_status)}")
    print(f"teleop_status msgs:{len(data.teleop_status)}")
    print(f"motor_config msgs: {len(data.motor_config)}")
    print(f"motor_battery msgs:{len(data.motor_battery)}")

    # Control method detection
    if data.teleop_status:
        modes = set()
        for _, js in data.teleop_status:
            mode = js.get("mode", "?")
            src = js.get("active_source", js.get("source", "?"))
            modes.add(f"{mode}/{src}")
        print(f"Control modes seen: {modes}")

    # Motor config
    if data.motor_status:
        last = data.motor_status[-1][1]
        print(f"Motor mode:     {last.get('mode', '?')}")
        print(f"Voltage:        {last.get('voltage_input', last.get('master', {}).get('voltage_input', '?'))}V")
        print(f"Max RPM:        {last.get('max_rpm', '?')}")
        print(f"Ramp up:        {last.get('ramp_up_rpm_s', '?')} RPM/s")
        print(f"Ramp down:      {last.get('ramp_down_rpm_s', '?')} RPM/s")
        print(f"Brake current:  {last.get('brake_current', '?')} A")
        print(f"Coast factor:   {last.get('coast_factor', '?')}")
        print(f"Left scale:     {last.get('left_scale', '?')}")
        print(f"Right scale:    {last.get('right_scale', '?')}")
        print(f"Max linear vel: {last.get('max_linear_vel', '?')}")
        print(f"Max angular vel:{last.get('max_angular_vel', '?')}")

        # Check for faults
        for _, js in data.motor_status:
            for side in ["master", "slave"]:
                info = js.get(side, {})
                fc = info.get("fault_code", 0)
                if fc != 0:
                    print(f"  *** FAULT on {side}: code={fc} name={info.get('fault_name', '?')}")

    # Battery voltage
    if data.motor_battery:
        voltages = [v for _, v, _, _ in data.motor_battery]
        print(f"Battery voltage: {min(voltages):.1f}V - {max(voltages):.1f}V (mean {np.mean(voltages):.1f}V)")

    # cmd_vel stats
    if data.cmd_vel:
        lin = [c[1] for c in data.cmd_vel]
        ang = [c[2] for c in data.cmd_vel]
        print(f"\ncmd_vel linear.x:  min={min(lin):.3f}  max={max(lin):.3f}  mean={np.mean(lin):.3f}")
        print(f"cmd_vel angular.z: min={min(ang):.3f}  max={max(ang):.3f}  mean={np.mean(ang):.3f}")

        # Steering bias: when trying to go straight (linear > 0.1, angular ~0),
        # what angular.z is needed?
        straight_attempts = [(l, a) for _, l, a in data.cmd_vel if abs(l) > 0.05]
        if straight_attempts:
            lins, angs = zip(*straight_attempts)
            # Ratio of angular to linear when driving
            ratios = [a / l for l, a in straight_attempts if abs(l) > 0.1]
            if ratios:
                print(f"Angular/linear ratio (when driving): mean={np.mean(ratios):.3f}  std={np.std(ratios):.3f}")

    # wheel_odom stats
    if data.wheel_odom:
        lin = [w[1] for w in data.wheel_odom]
        ang = [w[2] for w in data.wheel_odom]
        print(f"\nwheel_odom linear.x:  min={min(lin):.3f}  max={max(lin):.3f}  mean={np.mean(lin):.3f}")
        print(f"wheel_odom angular.z: min={min(ang):.3f}  max={max(ang):.3f}  mean={np.mean(ang):.3f}")

    # Per-motor RPM analysis
    if data.motor_status:
        m_rpms = []
        s_rpms = []
        m_cmd = []
        s_cmd = []
        for _, js in data.motor_status:
            master = js.get("master", {})
            slave = js.get("slave", {})
            if "rpm" in master:
                m_rpms.append(master["rpm"])
                s_rpms.append(slave.get("rpm", 0))
            if "commanded_erpm" in master:
                m_cmd.append(master["commanded_erpm"])
                s_cmd.append(slave.get("commanded_erpm", 0))

        if m_rpms:
            print(f"\nMaster RPM:     min={min(m_rpms):.0f}  max={max(m_rpms):.0f}  mean={np.mean(m_rpms):.1f}")
            print(f"Slave RPM:      min={min(s_rpms):.0f}  max={max(s_rpms):.0f}  mean={np.mean(s_rpms):.1f}")
            diffs = [m - s for m, s in zip(m_rpms, s_rpms)]
            print(f"RPM diff (M-S): min={min(diffs):.0f}  max={max(diffs):.0f}  mean={np.mean(diffs):.1f}")

        if m_cmd:
            print(f"Master cmd ERPM: min={min(m_cmd)}  max={max(m_cmd)}")
            print(f"Slave cmd ERPM:  min={min(s_cmd)}  max={max(s_cmd)}")


def plot_control_analysis(bags: list[BagData], out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    for data in bags:
        if not data.cmd_vel and not data.wheel_odom:
            continue

        fig, axes = plt.subplots(4, 1, figsize=(16, 14), sharex=True)
        fig.suptitle(f"Control Analysis: {data.name}", fontsize=14, fontweight="bold")

        t0 = min(
            (data.cmd_vel[0][0] if data.cmd_vel else 1e18),
            (data.wheel_odom[0][0] if data.wheel_odom else 1e18),
        )

        # ── Panel 1: cmd_vel linear.x + angular.z ──
        ax = axes[0]
        if data.cmd_vel:
            ts = [c[0] - t0 for c in data.cmd_vel]
            lins = [c[1] for c in data.cmd_vel]
            angs = [c[2] for c in data.cmd_vel]
            ax.plot(ts, lins, "b-", linewidth=0.8, label="linear.x (m/s)", alpha=0.9)
            ax.plot(ts, angs, "r-", linewidth=0.8, label="angular.z (rad/s)", alpha=0.9)
        ax.set_ylabel("cmd_vel")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_title("Commanded velocity")

        # ── Panel 2: wheel_odom linear.x + angular.z ──
        ax = axes[1]
        if data.wheel_odom:
            ts = [w[0] - t0 for w in data.wheel_odom]
            lins = [w[1] for w in data.wheel_odom]
            angs = [w[2] for w in data.wheel_odom]
            ax.plot(ts, lins, "b-", linewidth=0.8, label="linear.x (m/s)", alpha=0.9)
            ax.plot(ts, angs, "r-", linewidth=0.8, label="angular.z (rad/s)", alpha=0.9)
        ax.set_ylabel("wheel_odom")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_title("Actual wheel velocity")

        # ── Panel 3: cmd_vel vs wheel_odom overlay (linear only) ──
        ax = axes[2]
        if data.cmd_vel:
            ts = [c[0] - t0 for c in data.cmd_vel]
            lins = [c[1] for c in data.cmd_vel]
            ax.plot(ts, lins, "b--", linewidth=0.8, label="cmd linear.x", alpha=0.7)
        if data.wheel_odom:
            ts = [w[0] - t0 for w in data.wheel_odom]
            lins = [w[1] for w in data.wheel_odom]
            ax.plot(ts, lins, "b-", linewidth=0.8, label="odom linear.x", alpha=0.9)
        if data.cmd_vel:
            ts = [c[0] - t0 for c in data.cmd_vel]
            angs = [c[2] for c in data.cmd_vel]
            ax.plot(ts, angs, "r--", linewidth=0.8, label="cmd angular.z", alpha=0.7)
        if data.wheel_odom:
            ts = [w[0] - t0 for w in data.wheel_odom]
            angs = [w[2] for w in data.wheel_odom]
            ax.plot(ts, angs, "r-", linewidth=0.8, label="odom angular.z", alpha=0.9)
        ax.set_ylabel("velocity")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_title("Command vs Actual overlay (dashed=cmd, solid=odom)")

        # ── Panel 4: per-motor RPM from status ──
        ax = axes[3]
        if data.motor_status:
            ts = [s[0] - t0 for s in data.motor_status]
            m_rpms = [s[1].get("master", {}).get("rpm", 0) for s in data.motor_status]
            s_rpms = [s[1].get("slave", {}).get("rpm", 0) for s in data.motor_status]
            ax.plot(ts, m_rpms, "g-", linewidth=1, label="master RPM", alpha=0.9)
            ax.plot(ts, s_rpms, "m-", linewidth=1, label="slave RPM", alpha=0.9)
            diffs = [m - s for m, s in zip(m_rpms, s_rpms)]
            ax.plot(ts, diffs, "k--", linewidth=0.8, label="diff (M-S)", alpha=0.6)
        ax.set_ylabel("RPM")
        ax.set_xlabel("Time (s)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_title("Per-motor RPM (master=right, slave=left)")

        plt.tight_layout()
        out_path = out_dir / f"{data.name}_control.png"
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        print(f"  Saved {out_path}")

    # ── Steering bias scatter plot ──
    fig, axes = plt.subplots(1, len(bags), figsize=(5 * len(bags), 5))
    if len(bags) == 1:
        axes = [axes]
    fig.suptitle("Steering Bias: angular.z when driving forward", fontsize=13, fontweight="bold")
    for ax, data in zip(axes, bags):
        if data.cmd_vel:
            fwd = [(l, a) for _, l, a in data.cmd_vel if l > 0.05]
            if fwd:
                lins, angs = zip(*fwd)
                ax.scatter(lins, angs, s=4, alpha=0.4)
                ax.axhline(0, color="k", linewidth=0.5)
                ax.set_xlabel("linear.x (m/s)")
                ax.set_ylabel("angular.z (rad/s)")
        ax.set_title(data.name.split("_", 2)[-1], fontsize=10)
        ax.grid(True, alpha=0.3)
    plt.tight_layout()
    out_path = out_dir / "steering_bias_scatter.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Saved {out_path}")

    # ── Braking analysis: velocity around zero crossings ──
    fig, axes = plt.subplots(len(bags), 1, figsize=(16, 4 * len(bags)))
    if len(bags) == 1:
        axes = [axes]
    fig.suptitle("Braking Jerkiness: wheel_odom velocity + derivative", fontsize=13, fontweight="bold")
    for ax, data in zip(axes, bags):
        if data.wheel_odom and len(data.wheel_odom) > 5:
            ts = np.array([w[0] - data.wheel_odom[0][0] for w in data.wheel_odom])
            lins = np.array([w[1] for w in data.wheel_odom])
            # Acceleration (numerical derivative)
            dt = np.diff(ts)
            dt[dt == 0] = 1e-6
            accel = np.diff(lins) / dt
            ax.plot(ts, lins, "b-", linewidth=0.8, label="linear vel (m/s)")
            ax.plot(ts[1:], accel, "r-", linewidth=0.6, alpha=0.7, label="accel (m/s²)")
            ax.axhline(0, color="k", linewidth=0.5)
            ax.legend(loc="upper right", fontsize=8)
        ax.set_title(data.name, fontsize=10)
        ax.set_ylabel("vel / accel")
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    out_path = out_dir / "braking_analysis.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  Saved {out_path}")

    # ── Two-stick issue: gaps where linear=0 but angular!=0 ──
    for data in bags:
        if not data.cmd_vel or len(data.cmd_vel) < 10:
            continue
        fig, ax = plt.subplots(1, 1, figsize=(16, 5))
        ts = np.array([c[0] - data.cmd_vel[0][0] for c in data.cmd_vel])
        lins = np.array([c[1] for c in data.cmd_vel])
        angs = np.array([c[2] for c in data.cmd_vel])

        ax.plot(ts, lins, "b-", linewidth=0.8, label="linear.x", alpha=0.9)
        ax.plot(ts, angs, "r-", linewidth=0.8, label="angular.z", alpha=0.9)

        # Highlight regions where linear=0 but angular!=0
        steer_only = (np.abs(lins) < 0.02) & (np.abs(angs) > 0.05)
        if np.any(steer_only):
            ax.fill_between(ts, -2, 2, where=steer_only, alpha=0.2, color="orange",
                            label="steer-only (no velocity)")

        ax.set_title(f"Two-Stick Issue: {data.name}", fontsize=12)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("cmd_vel")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-2.5, 2.5)
        plt.tight_layout()
        out_path = out_dir / f"{data.name}_twostick.png"
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        print(f"  Saved {out_path}")


def main() -> None:
    print("Loading Friday bags...")
    bags = []
    for name in FRIDAY_BAGS:
        print(f"\n→ {name}")
        d = load_bag(name)
        bags.append(d)

    # Print summaries
    for d in bags:
        print_bag_summary(d)

    # Check for older baseline bag
    baseline_dir = BAGS_DIR / BASELINE_BAG
    has_baseline = baseline_dir.exists()
    if has_baseline:
        print(f"\n→ Loading baseline bag: {BASELINE_BAG}")
        baseline = load_bag(BASELINE_BAG)
        print_bag_summary(baseline)

    # Generate plots
    out_dir = BAGS_DIR / "analysis"
    print(f"\n{'='*70}")
    print(f"Generating plots to {out_dir} ...")
    print(f"{'='*70}")

    # Only plot bags that have cmd_vel or wheel_odom
    plottable = [d for d in bags if d.cmd_vel or d.wheel_odom]
    if has_baseline and (baseline.cmd_vel or baseline.wheel_odom):
        plottable.append(baseline)
    plot_control_analysis(plottable, out_dir)

    print("\nDone!")


if __name__ == "__main__":
    main()
