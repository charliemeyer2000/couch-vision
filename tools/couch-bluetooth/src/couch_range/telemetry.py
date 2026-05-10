from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from enum import StrEnum
from pathlib import Path
import csv
import json
import socket
import uuid


class DistanceSource(StrEnum):
    MANUAL_MARKER = "manual_marker"
    TIMED_STATION = "timed_station"
    EVENT_MARKER = "event_marker"
    GEOLOCATION = "geolocation"
    SIMULATION = "simulation"


class ReliabilityBand(StrEnum):
    RELIABLE = "reliable"
    MARGINAL = "marginal"
    UNSAFE = "unsafe"
    INSUFFICIENT_DATA = "insufficient_data"
    NO_DATA = "no_data"


CSV_FIELDS = [
    "run_id",
    "device_role",
    "laptop_hostname",
    "orin_hostname",
    "adapter_name",
    "adapter_address",
    "controller_name",
    "sequence",
    "timestamp",
    "elapsed_s",
    "declared_distance_m",
    "distance_source",
    "gps_latitude",
    "gps_longitude",
    "geolocation_accuracy_m",
    "rssi_dbm",
    "ping_rtt_ms",
    "command_latency_ms",
    "command_sent_count",
    "command_ack_count",
    "command_timeout_count",
    "packet_loss_percent",
    "reconnect_events",
    "drop_events",
    "error",
    "notes",
]

EVENT_FIELDS = [
    "record_type",
    "event_id",
    "run_id",
    "event_type",
    "event_severity",
    "timestamp",
    "elapsed_s",
    "declared_distance_m",
    "distance_source",
    "note",
]


@dataclass(slots=True)
class Sample:
    run_id: str
    device_role: str
    laptop_hostname: str
    orin_hostname: str
    adapter_name: str
    adapter_address: str
    controller_name: str
    sequence: int
    timestamp: str
    elapsed_s: float
    declared_distance_m: float
    distance_source: str
    gps_latitude: float | None
    gps_longitude: float | None
    geolocation_accuracy_m: float | None
    rssi_dbm: float | None
    ping_rtt_ms: float | None
    command_latency_ms: float | None
    command_sent_count: int
    command_ack_count: int
    command_timeout_count: int
    packet_loss_percent: float
    reconnect_events: int
    drop_events: int
    error: str
    notes: str


@dataclass(slots=True)
class Event:
    record_type: str
    event_id: str
    run_id: str
    event_type: str
    event_severity: str
    timestamp: str
    elapsed_s: float
    declared_distance_m: float | None
    distance_source: str
    note: str


def new_run_id(prefix: str = "run") -> str:
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    return f"{prefix}-{stamp}-{uuid.uuid4().hex[:8]}"


def utc_now() -> str:
    return datetime.now(UTC).isoformat()


def local_hostname() -> str:
    return socket.gethostname()


def ensure_run_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    (path / "raw").mkdir(exist_ok=True)
    (path / "analysis").mkdir(exist_ok=True)
    (path / "charts").mkdir(exist_ok=True)
    return path


def raw_dir(run_dir: Path) -> Path:
    return run_dir / "raw"


def analysis_dir(run_dir: Path) -> Path:
    return run_dir / "analysis"


def sample_to_row(sample: Sample) -> dict[str, str | int | float | None]:
    return asdict(sample)


def append_sample(csv_path: Path, jsonl_path: Path, sample: Sample) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    jsonl_path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not csv_path.exists()
    with csv_path.open("a", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        if write_header:
            writer.writeheader()
        writer.writerow(sample_to_row(sample))
    with jsonl_path.open("a") as handle:
        handle.write(json.dumps(sample_to_row(sample), sort_keys=True) + "\n")


def append_event(csv_path: Path, jsonl_path: Path, event: Event) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    jsonl_path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not csv_path.exists()
    row = asdict(event)
    with csv_path.open("a", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=EVENT_FIELDS)
        if write_header:
            writer.writeheader()
        writer.writerow(row)
    with jsonl_path.open("a") as handle:
        handle.write(json.dumps(row, sort_keys=True) + "\n")


def write_samples(run_dir: Path, samples: list[Sample]) -> None:
    ensure_run_dir(run_dir)
    csv_path = raw_dir(run_dir) / "samples.csv"
    jsonl_path = raw_dir(run_dir) / "samples.jsonl"
    if csv_path.exists():
        csv_path.unlink()
    if jsonl_path.exists():
        jsonl_path.unlink()
    for sample in samples:
        append_sample(csv_path, jsonl_path, sample)


def write_events(run_dir: Path, events: list[Event]) -> None:
    ensure_run_dir(run_dir)
    csv_path = raw_dir(run_dir) / "events.csv"
    jsonl_path = raw_dir(run_dir) / "events.jsonl"
    if csv_path.exists():
        csv_path.unlink()
    if jsonl_path.exists():
        jsonl_path.unlink()
    for event in events:
        append_event(csv_path, jsonl_path, event)


def read_samples(run_dir: Path) -> list[Sample]:
    csv_path = raw_dir(run_dir) / "samples.csv"
    if not csv_path.exists():
        csv_path = run_dir / "samples.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"missing {csv_path}")
    samples: list[Sample] = []
    with csv_path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            samples.append(_row_to_sample(row))
    return samples


def _none_if_blank(value: str | None) -> str | None:
    if value is None or value == "":
        return None
    return value


def _float_or_none(value: str | None) -> float | None:
    value = _none_if_blank(value)
    if value is None:
        return None
    return float(value)


def _row_to_sample(row: dict[str, str]) -> Sample:
    return Sample(
        run_id=row["run_id"],
        device_role=row["device_role"],
        laptop_hostname=row["laptop_hostname"],
        orin_hostname=row["orin_hostname"],
        adapter_name=row["adapter_name"],
        adapter_address=row["adapter_address"],
        controller_name=row["controller_name"],
        sequence=int(row["sequence"]),
        timestamp=row["timestamp"],
        elapsed_s=float(row["elapsed_s"]),
        declared_distance_m=float(row["declared_distance_m"]),
        distance_source=row["distance_source"],
        gps_latitude=_float_or_none(row.get("gps_latitude")),
        gps_longitude=_float_or_none(row.get("gps_longitude")),
        geolocation_accuracy_m=_float_or_none(row.get("geolocation_accuracy_m")),
        rssi_dbm=_float_or_none(row.get("rssi_dbm")),
        ping_rtt_ms=_float_or_none(row.get("ping_rtt_ms")),
        command_latency_ms=_float_or_none(row.get("command_latency_ms")),
        command_sent_count=int(row["command_sent_count"]),
        command_ack_count=int(row["command_ack_count"]),
        command_timeout_count=int(row["command_timeout_count"]),
        packet_loss_percent=float(row["packet_loss_percent"]),
        reconnect_events=int(row["reconnect_events"]),
        drop_events=int(row["drop_events"]),
        error=row["error"],
        notes=row["notes"],
    )
