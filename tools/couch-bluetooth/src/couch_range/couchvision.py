from __future__ import annotations

from dataclasses import dataclass
from typing import Any
import json
import time
import urllib.error
import urllib.parse
import urllib.request


# Must match /home/barrett/dev/couch-vision/scripts/ble_relay.py.
SERVICE_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567890"
CMD_VEL_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567891"
E_STOP_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567892"
HEARTBEAT_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567893"
PONG_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567894"

DEVICE_NAME = "CouchVision-BLE"
HTTP_PORT = 4200

STATUS_PATH = "/status"


@dataclass(frozen=True, slots=True)
class CouchVisionStatus:
    connected: bool
    relay_latency_ms: float | None
    heartbeat_rtt_ms: float | None
    writes_per_sec: float | None
    request_latency_ms: float | None
    raw: dict[str, Any]
    error: str


_urlopen = urllib.request.urlopen


def fetch_couchvision_status(base_url: str, timeout_s: float) -> CouchVisionStatus:
    request_latency_ms: float | None = None
    raw: dict[str, Any] = {}

    if timeout_s <= 0:
        return _status_error("timeout_s must be greater than 0")

    try:
        url = _status_url(base_url)
    except ValueError as exc:
        return _status_error(str(exc))

    request = urllib.request.Request(
        url,
        headers={"Accept": "application/json"},
        method="GET",
    )
    started = time.perf_counter()
    try:
        with _urlopen(request, timeout=timeout_s) as response:
            status_code = _response_status_code(response)
            if status_code is not None and status_code >= 400:
                raise ValueError(f"HTTP status {status_code}")
            body = response.read()
        request_latency_ms = (time.perf_counter() - started) * 1000
        decoded = json.loads(body.decode("utf-8"))
        if not isinstance(decoded, dict):
            raise ValueError("status response must be a JSON object")
        raw = decoded
        return _parse_status(raw, request_latency_ms)
    except (
        OSError,
        TimeoutError,
        UnicodeDecodeError,
        ValueError,
        urllib.error.URLError,
    ) as exc:
        request_latency_ms = (time.perf_counter() - started) * 1000
        return _status_error(str(exc), request_latency_ms, raw)


def _status_url(base_url: str) -> str:
    candidate = base_url.strip()
    if not candidate:
        candidate = f"127.0.0.1:{HTTP_PORT}"
    if "://" not in candidate:
        candidate = f"http://{candidate}"

    parsed = urllib.parse.urlsplit(candidate)
    scheme = parsed.scheme.lower()
    if scheme not in {"http", "https"}:
        raise ValueError(f"unsupported CouchVision status URL scheme: {parsed.scheme}")
    if not parsed.hostname:
        raise ValueError("CouchVision status URL is missing a host")

    port = _parsed_port(parsed)
    netloc = _netloc(parsed.hostname, port or HTTP_PORT)
    path = _status_path(parsed.path)
    return urllib.parse.urlunsplit((scheme, netloc, path, "", ""))


def _parsed_port(parsed: urllib.parse.SplitResult) -> int | None:
    try:
        return parsed.port
    except ValueError as exc:
        raise ValueError(f"invalid CouchVision status URL port: {exc}") from exc


def _netloc(hostname: str, port: int) -> str:
    host = hostname
    if ":" in host and not host.startswith("["):
        host = f"[{host}]"
    return f"{host}:{port}"


def _status_path(path: str) -> str:
    normalized = "/" + path.strip("/")
    if normalized == "/":
        return STATUS_PATH
    if normalized.endswith(STATUS_PATH):
        return normalized
    return f"{normalized}{STATUS_PATH}"


def _parse_status(raw: dict[str, Any], request_latency_ms: float) -> CouchVisionStatus:
    connected = raw.get("connected")
    if not isinstance(connected, bool):
        raise ValueError("status.connected must be a boolean")

    return CouchVisionStatus(
        connected=connected,
        relay_latency_ms=_float_or_none(raw, "latency_ms"),
        heartbeat_rtt_ms=_float_or_none(raw, "rtt_ms"),
        writes_per_sec=_float_or_none(raw, "writes_per_sec"),
        request_latency_ms=request_latency_ms,
        raw=raw,
        error="",
    )


def _float_or_none(raw: dict[str, Any], key: str) -> float | None:
    value = raw.get(key)
    if value is None:
        return None
    if isinstance(value, bool):
        raise ValueError(f"status.{key} must be numeric")
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"status.{key} must be numeric") from exc


def _response_status_code(response: Any) -> int | None:
    status = getattr(response, "status", None)
    if status is None:
        getcode = getattr(response, "getcode", None)
        if getcode is not None:
            status = getcode()
    if status is None:
        return None
    return int(status)


def _status_error(
    error: str,
    request_latency_ms: float | None = None,
    raw: dict[str, Any] | None = None,
) -> CouchVisionStatus:
    return CouchVisionStatus(
        connected=False,
        relay_latency_ms=None,
        heartbeat_rtt_ms=None,
        writes_per_sec=None,
        request_latency_ms=request_latency_ms,
        raw=raw or {},
        error=error,
    )
