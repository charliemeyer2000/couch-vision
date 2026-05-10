from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
import subprocess
import time

from .bluetooth import bluetooth_rssi
from .config import AppConfig
from .controller import best_controller_name
from .couchvision import CouchVisionStatus, fetch_couchvision_status
from .distance import DistanceStation
from .protocol import send_command
from .telemetry import (
    DistanceSource,
    Event,
    Sample,
    append_event,
    append_sample,
    local_hostname,
    new_run_id,
    raw_dir,
    utc_now,
)

TAILSCALE_RTT_RE = re.compile(r"\bin\s+([0-9.]+)ms\b")


@dataclass(frozen=True, slots=True)
class PingResult:
    ok: bool
    latency_ms: float | None
    error: str


def run_field_test(
    out_dir: Path,
    config: AppConfig,
    stations: list[DistanceStation],
    target: str,
    port: int,
    adapter_name: str = "",
    adapter_address: str = "",
    bluetooth_peer: str = "",
    no_prompt: bool = False,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "raw").mkdir(exist_ok=True)
    csv_path = raw_dir(out_dir) / "samples.csv"
    jsonl_path = raw_dir(out_dir) / "samples.jsonl"
    events_csv_path = raw_dir(out_dir) / "events.csv"
    events_jsonl_path = raw_dir(out_dir) / "events.jsonl"
    csv_path.unlink(missing_ok=True)
    jsonl_path.unlink(missing_ok=True)
    events_csv_path.unlink(missing_ok=True)
    events_jsonl_path.unlink(missing_ok=True)

    run_id = new_run_id("field")
    hostname = local_hostname()
    controller = best_controller_name()
    sequence = 0
    elapsed_start = time.perf_counter()

    for station in stations:
        if not no_prompt:
            input(
                f"Stand at {station.meters:g} m, then press Enter to collect {station.hold_seconds:g} seconds..."
            )
        append_event(
            events_csv_path,
            events_jsonl_path,
            Event(
                record_type="event",
                event_id=f"station-{station.meters:g}-start",
                run_id=run_id,
                event_type="station_start",
                event_severity="info",
                timestamp=utc_now(),
                elapsed_s=time.perf_counter() - elapsed_start,
                declared_distance_m=station.meters,
                distance_source=station.source,
                note=f"station started at {station.meters:g} m",
            ),
        )
        deadline = time.perf_counter() + station.hold_seconds
        sent = 0
        acked = 0
        timeouts = 0
        reconnects = 0
        drops = 0
        interval = 1 / config.run.sample_rate_hz
        while time.perf_counter() < deadline:
            sequence += 1
            sent += 1
            result = send_command(
                target, port, sequence, config.run.timeout_ms, run_id=run_id
            )
            if result.ok:
                acked += 1
            else:
                timeouts += 1
            loss = (timeouts / sent) * 100 if sent else 100.0
            sample = Sample(
                run_id=run_id,
                device_role="laptop_logger",
                laptop_hostname=hostname,
                orin_hostname=target,
                adapter_name=adapter_name,
                adapter_address=adapter_address,
                controller_name=controller,
                sequence=sequence,
                timestamp=utc_now(),
                elapsed_s=time.perf_counter() - elapsed_start,
                declared_distance_m=station.meters,
                distance_source=station.source or DistanceSource.MANUAL_MARKER.value,
                gps_latitude=None,
                gps_longitude=None,
                geolocation_accuracy_m=None,
                rssi_dbm=bluetooth_rssi(bluetooth_peer) if bluetooth_peer else None,
                ping_rtt_ms=ping_rtt_ms(target),
                command_latency_ms=result.latency_ms,
                command_sent_count=sent,
                command_ack_count=acked,
                command_timeout_count=timeouts,
                packet_loss_percent=loss,
                reconnect_events=reconnects,
                drop_events=drops,
                error=result.error,
                notes=config.run.notes,
            )
            append_sample(csv_path, jsonl_path, sample)
            sleep_for = max(
                0.0, interval - ((time.perf_counter() - elapsed_start) % interval)
            )
            time.sleep(min(sleep_for, interval))
        append_event(
            events_csv_path,
            events_jsonl_path,
            Event(
                record_type="event",
                event_id=f"station-{station.meters:g}-end",
                run_id=run_id,
                event_type="station_end",
                event_severity="info",
                timestamp=utc_now(),
                elapsed_s=time.perf_counter() - elapsed_start,
                declared_distance_m=station.meters,
                distance_source=station.source,
                note=f"station ended at {station.meters:g} m",
            ),
        )


def run_ping_walk(
    out_dir: Path,
    config: AppConfig,
    stations: list[DistanceStation],
    target: str,
    no_prompt: bool = False,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "raw").mkdir(exist_ok=True)
    csv_path = raw_dir(out_dir) / "samples.csv"
    jsonl_path = raw_dir(out_dir) / "samples.jsonl"
    events_csv_path = raw_dir(out_dir) / "events.csv"
    events_jsonl_path = raw_dir(out_dir) / "events.jsonl"
    csv_path.unlink(missing_ok=True)
    jsonl_path.unlink(missing_ok=True)
    events_csv_path.unlink(missing_ok=True)
    events_jsonl_path.unlink(missing_ok=True)

    run_id = new_run_id("ping")
    hostname = local_hostname()
    sequence = 0
    elapsed_start = time.perf_counter()
    previous_ok: bool | None = None
    reconnects = 0
    drops = 0

    for station in stations:
        if not no_prompt:
            input(
                f"Stand at {station.meters:g} m, then press Enter to ping {target} for {station.hold_seconds:g} seconds..."
            )
        append_event(
            events_csv_path,
            events_jsonl_path,
            Event(
                record_type="event",
                event_id=f"station-{station.meters:g}-start",
                run_id=run_id,
                event_type="station_start",
                event_severity="info",
                timestamp=utc_now(),
                elapsed_s=time.perf_counter() - elapsed_start,
                declared_distance_m=station.meters,
                distance_source=station.source,
                note=f"ping station started at {station.meters:g} m",
            ),
        )
        deadline = time.perf_counter() + station.hold_seconds
        sent = 0
        acked = 0
        timeouts = 0
        interval = 1 / config.run.sample_rate_hz
        while time.perf_counter() < deadline:
            sequence += 1
            sent += 1
            result = tailscale_ping_once(target, timeout_ms=config.run.timeout_ms)
            if result.ok:
                acked += 1
                if previous_ok is False:
                    reconnects += 1
                previous_ok = True
            else:
                timeouts += 1
                if previous_ok is True:
                    drops += 1
                previous_ok = False
            loss = (timeouts / sent) * 100 if sent else 100.0
            sample = Sample(
                run_id=run_id,
                device_role="laptop_ping_probe",
                laptop_hostname=hostname,
                orin_hostname=target,
                adapter_name="",
                adapter_address="",
                controller_name="not used for ping-only diagnostic",
                sequence=sequence,
                timestamp=utc_now(),
                elapsed_s=time.perf_counter() - elapsed_start,
                declared_distance_m=station.meters,
                distance_source=station.source or DistanceSource.MANUAL_MARKER.value,
                gps_latitude=None,
                gps_longitude=None,
                geolocation_accuracy_m=None,
                rssi_dbm=None,
                ping_rtt_ms=result.latency_ms,
                command_latency_ms=result.latency_ms,
                command_sent_count=sent,
                command_ack_count=acked,
                command_timeout_count=timeouts,
                packet_loss_percent=loss,
                reconnect_events=reconnects,
                drop_events=drops,
                error=result.error,
                notes="ping-only diagnostic; no controller, receiver, Bluetooth command path, or couch motion",
            )
            append_sample(csv_path, jsonl_path, sample)
            sleep_for = max(
                0.0, interval - ((time.perf_counter() - elapsed_start) % interval)
            )
            time.sleep(min(sleep_for, interval))
        append_event(
            events_csv_path,
            events_jsonl_path,
            Event(
                record_type="event",
                event_id=f"station-{station.meters:g}-end",
                run_id=run_id,
                event_type="station_end",
                event_severity="info",
                timestamp=utc_now(),
                elapsed_s=time.perf_counter() - elapsed_start,
                declared_distance_m=station.meters,
                distance_source=station.source,
                note=f"ping station ended at {station.meters:g} m",
            ),
        )


def run_couchvision_walk(
    out_dir: Path,
    config: AppConfig,
    stations: list[DistanceStation],
    relay_url: str,
    no_prompt: bool = False,
    status_timeout_s: float = 1.0,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "raw").mkdir(exist_ok=True)
    csv_path = raw_dir(out_dir) / "samples.csv"
    jsonl_path = raw_dir(out_dir) / "samples.jsonl"
    events_csv_path = raw_dir(out_dir) / "events.csv"
    events_jsonl_path = raw_dir(out_dir) / "events.jsonl"
    csv_path.unlink(missing_ok=True)
    jsonl_path.unlink(missing_ok=True)
    events_csv_path.unlink(missing_ok=True)
    events_jsonl_path.unlink(missing_ok=True)

    run_id = new_run_id("couchvision")
    hostname = local_hostname()
    sequence = 0
    elapsed_start = time.perf_counter()
    previous_ok: bool | None = None
    reconnects = 0
    drops = 0

    for station in stations:
        if not no_prompt:
            input(
                f"Stand at {station.meters:g} m, then press Enter to poll CouchVision /status for {station.hold_seconds:g} seconds..."
            )
        append_event(
            events_csv_path,
            events_jsonl_path,
            Event(
                record_type="event",
                event_id=f"station-{station.meters:g}-start",
                run_id=run_id,
                event_type="station_start",
                event_severity="info",
                timestamp=utc_now(),
                elapsed_s=time.perf_counter() - elapsed_start,
                declared_distance_m=station.meters,
                distance_source=station.source,
                note=f"CouchVision /status station started at {station.meters:g} m",
            ),
        )
        deadline = time.perf_counter() + station.hold_seconds
        sent = 0
        acked = 0
        timeouts = 0
        interval = 1 / config.run.sample_rate_hz
        while time.perf_counter() < deadline:
            sequence += 1
            sent += 1
            status = fetch_couchvision_status(relay_url, timeout_s=status_timeout_s)
            heartbeat_rtt_ms = status.heartbeat_rtt_ms
            ok = (
                status.error == ""
                and status.connected
                and heartbeat_rtt_ms is not None
                and heartbeat_rtt_ms > 0
            )
            if ok:
                acked += 1
                if previous_ok is False:
                    reconnects += 1
                previous_ok = True
            else:
                timeouts += 1
                if previous_ok is True:
                    drops += 1
                previous_ok = False
            loss = (timeouts / sent) * 100 if sent else 100.0
            error = status.error
            if not error and not ok:
                error = _couchvision_status_error(status)
            notes = (
                "CouchVision /status no-motion probe; "
                f"http_request_ms={_format_optional(status.request_latency_ms)}; "
                f"relay_latency_ms={_format_optional(status.relay_latency_ms)}; "
                f"writes_per_sec={_format_optional(status.writes_per_sec)}; "
                f"connected={status.connected}"
            )
            sample = Sample(
                run_id=run_id,
                device_role="couchvision_status_probe",
                laptop_hostname=hostname,
                orin_hostname=relay_url,
                adapter_name="CouchVision BLE relay",
                adapter_address="",
                controller_name="not used for no-motion CouchVision status probe",
                sequence=sequence,
                timestamp=utc_now(),
                elapsed_s=time.perf_counter() - elapsed_start,
                declared_distance_m=station.meters,
                distance_source=station.source or DistanceSource.MANUAL_MARKER.value,
                gps_latitude=None,
                gps_longitude=None,
                geolocation_accuracy_m=None,
                rssi_dbm=None,
                ping_rtt_ms=heartbeat_rtt_ms,
                command_latency_ms=heartbeat_rtt_ms if ok else None,
                command_sent_count=sent,
                command_ack_count=acked,
                command_timeout_count=timeouts,
                packet_loss_percent=loss,
                reconnect_events=reconnects,
                drop_events=drops,
                error=error,
                notes=notes,
            )
            append_sample(csv_path, jsonl_path, sample)
            sleep_for = max(
                0.0, interval - ((time.perf_counter() - elapsed_start) % interval)
            )
            time.sleep(min(sleep_for, interval))
        append_event(
            events_csv_path,
            events_jsonl_path,
            Event(
                record_type="event",
                event_id=f"station-{station.meters:g}-end",
                run_id=run_id,
                event_type="station_end",
                event_severity="info",
                timestamp=utc_now(),
                elapsed_s=time.perf_counter() - elapsed_start,
                declared_distance_m=station.meters,
                distance_source=station.source,
                note=f"CouchVision /status station ended at {station.meters:g} m",
            ),
        )


def _format_optional(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.1f}"


def _couchvision_status_error(status: CouchVisionStatus) -> str:
    if not status.connected:
        return "CouchVision relay reports disconnected"
    return "missing CouchVision heartbeat pong RTT"


def ping_rtt_ms(target: str) -> float | None:
    try:
        completed = subprocess.run(
            ["ping", "-c", "1", "-W", "1", target],
            check=False,
            capture_output=True,
            text=True,
            timeout=2,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if completed.returncode != 0:
        return None
    marker = "time="
    for part in completed.stdout.split():
        if part.startswith(marker):
            return float(part.removeprefix(marker))
    return None


def tailscale_ping_once(target: str, timeout_ms: int) -> PingResult:
    timeout_s = max(1, int(round(timeout_ms / 1000)))
    try:
        completed = subprocess.run(
            ["tailscale", "ping", "--timeout", f"{timeout_s}s", "--c", "1", target],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s + 2,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return PingResult(False, None, str(exc))
    output = "\n".join(part for part in [completed.stdout, completed.stderr] if part)
    match = TAILSCALE_RTT_RE.search(output)
    if completed.returncode == 0 and match is not None:
        return PingResult(True, float(match.group(1)), "")
    error = (
        output.strip().splitlines()[-1] if output.strip() else "tailscale ping failed"
    )
    return PingResult(False, None, error)
