from __future__ import annotations

from typing import Any
import urllib.error
import urllib.request

from couch_range import couchvision
from couch_range.couchvision import fetch_couchvision_status


class FakeResponse:
    def __init__(self, payload: bytes, status: int = 200) -> None:
        self._payload = payload
        self.status = status

    def __enter__(self) -> "FakeResponse":
        return self

    def __exit__(self, *args: object) -> None:
        return None

    def read(self) -> bytes:
        return self._payload


def test_constants_match_couchvision_ble_relay() -> None:
    assert couchvision.SERVICE_UUID == "a1b2c3d4-e5f6-7890-abcd-ef1234567890"
    assert couchvision.CMD_VEL_UUID == "a1b2c3d4-e5f6-7890-abcd-ef1234567891"
    assert couchvision.E_STOP_UUID == "a1b2c3d4-e5f6-7890-abcd-ef1234567892"
    assert couchvision.HEARTBEAT_UUID == "a1b2c3d4-e5f6-7890-abcd-ef1234567893"
    assert couchvision.PONG_UUID == "a1b2c3d4-e5f6-7890-abcd-ef1234567894"
    assert couchvision.DEVICE_NAME == "CouchVision-BLE"
    assert couchvision.HTTP_PORT == 4200


def test_fetch_status_parses_relay_payload(monkeypatch: Any) -> None:
    seen: list[tuple[str, float, str]] = []

    def opener(request: urllib.request.Request, timeout: float) -> FakeResponse:
        seen.append((request.full_url, timeout, request.get_method()))
        return FakeResponse(
            b'{"connected": true, "latency_ms": 1.25, "rtt_ms": 4.5, '
            b'"writes_per_sec": 12.0}'
        )

    monkeypatch.setattr(couchvision, "_urlopen", opener)

    status = fetch_couchvision_status("127.0.0.1", 2.5)

    assert seen == [("http://127.0.0.1:4200/status", 2.5, "GET")]
    assert status.connected is True
    assert status.relay_latency_ms == 1.25
    assert status.heartbeat_rtt_ms == 4.5
    assert status.writes_per_sec == 12.0
    assert status.request_latency_ms is not None
    assert status.raw == {
        "connected": True,
        "latency_ms": 1.25,
        "rtt_ms": 4.5,
        "writes_per_sec": 12.0,
    }
    assert status.error == ""


def test_fetch_status_does_not_duplicate_status_path(monkeypatch: Any) -> None:
    seen: list[str] = []

    def opener(request: urllib.request.Request, timeout: float) -> FakeResponse:
        seen.append(request.full_url)
        return FakeResponse(
            b'{"connected": false, "latency_ms": 0, "rtt_ms": 0, '
            b'"writes_per_sec": 0}'
        )

    monkeypatch.setattr(couchvision, "_urlopen", opener)

    status = fetch_couchvision_status("http://couch.local:4210/status/", 1.0)

    assert seen == ["http://couch.local:4210/status"]
    assert status.connected is False
    assert status.error == ""


def test_fetch_status_appends_status_to_base_path(monkeypatch: Any) -> None:
    seen: list[str] = []

    def opener(request: urllib.request.Request, timeout: float) -> FakeResponse:
        seen.append(request.full_url)
        return FakeResponse(b'{"connected": true}')

    monkeypatch.setattr(couchvision, "_urlopen", opener)

    status = fetch_couchvision_status("couch.local:4210/relay", 1.0)

    assert seen == ["http://couch.local:4210/relay/status"]
    assert status.connected is True
    assert status.error == ""


def test_fetch_status_returns_error_on_network_failure(monkeypatch: Any) -> None:
    def opener(request: urllib.request.Request, timeout: float) -> FakeResponse:
        raise urllib.error.URLError("connection refused")

    monkeypatch.setattr(couchvision, "_urlopen", opener)

    status = fetch_couchvision_status("http://127.0.0.1:4200", 0.5)

    assert status.connected is False
    assert status.relay_latency_ms is None
    assert status.heartbeat_rtt_ms is None
    assert status.writes_per_sec is None
    assert status.request_latency_ms is not None
    assert status.raw == {}
    assert "connection refused" in status.error


def test_fetch_status_preserves_raw_payload_on_parse_error(
    monkeypatch: Any,
) -> None:
    def opener(request: urllib.request.Request, timeout: float) -> FakeResponse:
        return FakeResponse(b'{"connected": "yes", "latency_ms": "slow"}')

    monkeypatch.setattr(couchvision, "_urlopen", opener)

    status = fetch_couchvision_status("http://127.0.0.1:4200", 1.0)

    assert status.connected is False
    assert status.raw == {"connected": "yes", "latency_ms": "slow"}
    assert status.error == "status.connected must be a boolean"
