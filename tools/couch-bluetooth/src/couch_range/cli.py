from __future__ import annotations

from pathlib import Path
import argparse
import sys

from .analysis import summarize_run, write_summary
from .config import load_config
from .couchvision import fetch_couchvision_status
from .devices import write_inventory
from .distance import build_stations, parse_distances
from .output import prepare_output_dir
from .plotting import create_plots
from .protocol import run_receiver
from .report import generate_report
from .runner import run_couchvision_walk, run_field_test, run_ping_walk
from .simulation import simulate_events, simulate_samples
from .telemetry import (
    DistanceSource,
    analysis_dir,
    raw_dir,
    write_events,
    write_samples,
)


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        match args.command:
            case "inventory":
                return inventory_command(args)
            case "simulate":
                return simulate_command(args)
            case "run":
                return run_command(args)
            case "receiver":
                return receiver_command(args)
            case "ping-walk":
                return ping_walk_command(args)
            case "couchvision-status":
                return couchvision_status_command(args)
            case "couchvision-walk":
                return couchvision_walk_command(args)
            case "analyze":
                return analyze_command(args)
            case "report":
                return report_command(args)
            case _:
                parser.print_help()
                return 2
    except FileExistsError as exc:
        print(f"error: {exc}", file=sys.stderr)
        print("use --force to intentionally overwrite run data", file=sys.stderr)
        return 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="couch-range")
    subparsers = parser.add_subparsers(dest="command", required=True)

    inventory = subparsers.add_parser(
        "inventory", help="collect local device and tailnet inventory"
    )
    inventory.add_argument("--out", type=Path, default=Path("runs/inventory"))

    simulate = subparsers.add_parser("simulate", help="generate a simulated run")
    simulate.add_argument("--out", type=Path, default=Path("runs/simulated"))
    simulate.add_argument("--distances", default=None)
    simulate.add_argument("--hold-seconds", type=float, default=6.0)
    simulate.add_argument("--sample-rate-hz", type=float, default=10.0)
    simulate.add_argument("--force", action="store_true")

    receiver = subparsers.add_parser(
        "receiver", help="run synthetic no-motion ack receiver"
    )
    receiver.add_argument("--bind", default="0.0.0.0")
    receiver.add_argument("--port", type=int, default=8765)

    ping_walk = subparsers.add_parser(
        "ping-walk",
        help="dead-simple laptop-to-target ping walk test with no receiver/controller",
    )
    ping_walk.add_argument("--config", type=Path, default=Path("config.example.toml"))
    ping_walk.add_argument("--out", type=Path, required=True)
    ping_walk.add_argument(
        "--target",
        required=True,
        help="Jetson/Orin hostname or Tailscale IP to ping",
    )
    ping_walk.add_argument(
        "--distances",
        default=None,
        help="comma-separated station distances in meters, for example 0,5,10,20",
    )
    ping_walk.add_argument(
        "--hold-seconds",
        type=float,
        default=None,
        help="seconds to stand at each station",
    )
    ping_walk.add_argument(
        "--sample-rate-hz",
        type=float,
        default=None,
        help="ping attempts per second; default comes from config",
    )
    ping_walk.add_argument(
        "--no-prompt",
        action="store_true",
        help="run through stations without waiting for Enter",
    )
    ping_walk.add_argument("--force", action="store_true")

    couchvision_status = subparsers.add_parser(
        "couchvision-status",
        help="check CouchVision relay /status without sending motion commands",
    )
    couchvision_status.add_argument(
        "--relay",
        default="http://127.0.0.1:4200",
        help="CouchVision relay base URL",
    )
    couchvision_status.add_argument("--timeout-seconds", type=float, default=1.0)

    couchvision_walk = subparsers.add_parser(
        "couchvision-walk",
        help="station-based no-motion CouchVision /status heartbeat walk test",
    )
    couchvision_walk.add_argument(
        "--config", type=Path, default=Path("config.example.toml")
    )
    couchvision_walk.add_argument("--out", type=Path, required=True)
    couchvision_walk.add_argument(
        "--relay",
        default="http://127.0.0.1:4200",
        help="CouchVision relay base URL",
    )
    couchvision_walk.add_argument(
        "--distances",
        default=None,
        help="comma-separated station distances in meters, for example 0,5,10,20",
    )
    couchvision_walk.add_argument("--hold-seconds", type=float, default=None)
    couchvision_walk.add_argument("--sample-rate-hz", type=float, default=None)
    couchvision_walk.add_argument("--timeout-seconds", type=float, default=1.0)
    couchvision_walk.add_argument(
        "--no-prompt",
        action="store_true",
        help="run through stations without waiting for Enter",
    )
    couchvision_walk.add_argument("--force", action="store_true")

    run = subparsers.add_parser("run", help="run a real station-based walk-away test")
    run.add_argument("--config", type=Path, default=Path("config.example.toml"))
    run.add_argument("--out", type=Path, required=True)
    run.add_argument("--target", required=True)
    run.add_argument("--port", type=int, default=8765)
    run.add_argument("--distances", default=None)
    run.add_argument("--hold-seconds", type=float, default=None)
    run.add_argument("--adapter-name", default="")
    run.add_argument("--adapter-address", default="")
    run.add_argument("--bluetooth-peer", default="")
    run.add_argument("--no-prompt", action="store_true")
    run.add_argument("--force", action="store_true")

    analyze = subparsers.add_parser(
        "analyze", help="summarize raw samples and create charts"
    )
    analyze.add_argument("run_dir", type=Path)
    analyze.add_argument("--config", type=Path, default=Path("config.example.toml"))

    report = subparsers.add_parser(
        "report", help="generate Markdown report from summary and charts"
    )
    report.add_argument("run_dir", type=Path)
    report.add_argument("--config", type=Path, default=Path("config.example.toml"))

    return parser


def inventory_command(args: argparse.Namespace) -> int:
    path = write_inventory(args.out)
    print(f"wrote {path}")
    return 0


def simulate_command(args: argparse.Namespace) -> int:
    out_dir = prepare_output_dir(args.out, force=args.force)
    distances = parse_distances(args.distances, [0, 5, 10, 20, 30, 50, 60])
    samples = simulate_samples(distances, args.hold_seconds, args.sample_rate_hz)
    write_samples(out_dir, samples)
    write_events(out_dir, simulate_events(samples, distances, args.hold_seconds))
    print(f"wrote {raw_dir(out_dir) / 'samples.csv'}")
    print(f"wrote {raw_dir(out_dir) / 'samples.jsonl'}")
    print(f"wrote {raw_dir(out_dir) / 'events.jsonl'}")
    return 0


def run_command(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    distances = parse_distances(args.distances, config.run.distances_m)
    hold_seconds = (
        args.hold_seconds if args.hold_seconds is not None else config.run.hold_seconds
    )
    stations = build_stations(
        distances, hold_seconds, DistanceSource.MANUAL_MARKER.value
    )
    prepare_output_dir(args.out, force=args.force)
    run_field_test(
        args.out,
        config,
        stations,
        args.target,
        args.port,
        adapter_name=args.adapter_name,
        adapter_address=args.adapter_address,
        bluetooth_peer=args.bluetooth_peer,
        no_prompt=args.no_prompt,
    )
    print(f"wrote {raw_dir(args.out) / 'samples.csv'}")
    return 0


def ping_walk_command(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    if args.sample_rate_hz is not None:
        config = config.with_sample_rate(args.sample_rate_hz)
    distances = parse_distances(args.distances, config.run.distances_m)
    hold_seconds = (
        args.hold_seconds if args.hold_seconds is not None else config.run.hold_seconds
    )
    stations = build_stations(
        distances, hold_seconds, DistanceSource.MANUAL_MARKER.value
    )
    prepare_output_dir(args.out, force=args.force)
    run_ping_walk(args.out, config, stations, args.target, no_prompt=args.no_prompt)
    print(f"wrote {raw_dir(args.out) / 'samples.csv'}")
    print(f"next: ./couch-range analyze {args.out} && ./couch-range report {args.out}")
    return 0


def couchvision_status_command(args: argparse.Namespace) -> int:
    status = fetch_couchvision_status(args.relay, timeout_s=args.timeout_seconds)
    if status.error:
        print(f"relay: {args.relay}")
        print(f"error: {status.error}")
        return 1
    print(f"relay: {args.relay}")
    print(f"connected: {status.connected}")
    print(f"heartbeat_rtt_ms: {_format_optional(status.heartbeat_rtt_ms)}")
    print(f"relay_latency_ms: {_format_optional(status.relay_latency_ms)}")
    print(f"http_request_ms: {_format_optional(status.request_latency_ms)}")
    print(f"writes_per_sec: {_format_optional(status.writes_per_sec)}")
    if status.heartbeat_rtt_ms is None or status.heartbeat_rtt_ms <= 0:
        print("warning: /status responded, but no heartbeat pong RTT is available")
    return 0


def couchvision_walk_command(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    if args.sample_rate_hz is not None:
        config = config.with_sample_rate(args.sample_rate_hz)
    distances = parse_distances(args.distances, config.run.distances_m)
    hold_seconds = (
        args.hold_seconds if args.hold_seconds is not None else config.run.hold_seconds
    )
    stations = build_stations(
        distances, hold_seconds, DistanceSource.MANUAL_MARKER.value
    )
    prepare_output_dir(args.out, force=args.force)
    run_couchvision_walk(
        args.out,
        config,
        stations,
        args.relay,
        no_prompt=args.no_prompt,
        status_timeout_s=args.timeout_seconds,
    )
    print(f"wrote {raw_dir(args.out) / 'samples.csv'}")
    print(f"next: ./couch-range analyze {args.out} && ./couch-range report {args.out}")
    return 0


def receiver_command(args: argparse.Namespace) -> int:
    print(f"listening on {args.bind}:{args.port}; synthetic no-motion commands only")
    try:
        run_receiver(args.bind, args.port)
    except KeyboardInterrupt:
        print("receiver stopped")
    return 0


def analyze_command(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    summaries = summarize_run(args.run_dir, config.thresholds)
    summary_path = write_summary(args.run_dir, summaries)
    charts = create_plots(args.run_dir, summaries)
    print(f"wrote {summary_path}")
    for chart in charts:
        print(f"wrote {chart}")
    return 0


def report_command(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    if (
        not (analysis_dir(args.run_dir) / "summary.json").exists()
        and not (args.run_dir / "summary.json").exists()
    ):
        summaries = summarize_run(args.run_dir, config.thresholds)
        write_summary(args.run_dir, summaries)
        create_plots(args.run_dir, summaries)
    path = generate_report(args.run_dir, config.thresholds)
    print(f"wrote {path}")
    return 0


def _format_optional(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.1f}"


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
