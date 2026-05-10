from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import json
import socket
import socketserver
import threading
import time


@dataclass(frozen=True, slots=True)
class AckResult:
    ok: bool
    latency_ms: float | None
    error: str


SAFE_COMMAND_KINDS = {"noop", "controller_sample", "link_check"}


def send_command(
    host: str, port: int, sequence: int, timeout_ms: int, run_id: str = ""
) -> AckResult:
    payload = {
        "schema_version": "1",
        "type": "synthetic_control",
        "phase": "synthetic_ack",
        "run_id": run_id,
        "command_id": f"{run_id or 'run'}-{sequence}",
        "sequence": sequence,
        "sent_monotonic": time.monotonic(),
        "command_kind": "noop",
        "requires_actuation": False,
        "payload": {"requested_action": "none"},
    }
    started = time.perf_counter()
    try:
        with socket.create_connection((host, port), timeout=timeout_ms / 1000) as sock:
            sock.settimeout(timeout_ms / 1000)
            sock.sendall((json.dumps(payload, sort_keys=True) + "\n").encode())
            data = _readline(sock)
    except OSError as exc:
        return AckResult(False, None, str(exc))
    latency_ms = (time.perf_counter() - started) * 1000
    try:
        ack = json.loads(data)
    except json.JSONDecodeError as exc:
        return AckResult(False, latency_ms, f"bad ack: {exc}")
    if ack.get("type") != "ack" or ack.get("sequence") != sequence:
        return AckResult(False, latency_ms, f"unexpected ack: {ack}")
    return AckResult(True, latency_ms, "")


def _readline(sock: socket.socket) -> str:
    chunks: list[bytes] = []
    while True:
        chunk = sock.recv(1)
        if not chunk:
            break
        if chunk == b"\n":
            break
        chunks.append(chunk)
    return b"".join(chunks).decode()


class _AckHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        line = self.rfile.readline().decode().strip()
        try:
            message = json.loads(line)
            command_kind = str(message.get("command_kind", ""))
            if (
                message.get("requires_actuation") is not False
                or command_kind not in SAFE_COMMAND_KINDS
            ):
                raise ValueError("unsafe or unsupported command")
            sequence = int(message.get("sequence", -1))
            ack = {
                "schema_version": "1",
                "type": "ack",
                "run_id": message.get("run_id", ""),
                "command_id": message.get("command_id", ""),
                "sequence": sequence,
                "received_monotonic": time.monotonic(),
                "ack_status": "ok",
                "requires_actuation": False,
            }
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            ack = {"type": "error", "error": str(exc), "no_motion": True}
        self.wfile.write((json.dumps(ack, sort_keys=True) + "\n").encode())


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


def run_receiver(bind: str, port: int) -> None:
    with ThreadedTCPServer((bind, port), _AckHandler) as server:
        server.serve_forever()


def start_receiver_thread(
    bind: str = "127.0.0.1", port: int = 0
) -> tuple[ThreadedTCPServer, threading.Thread, int]:
    server = ThreadedTCPServer((bind, port), _AckHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    actual_port = int(server.server_address[1])
    return server, thread, actual_port


def write_receiver_note(path: Path, bind: str, port: int) -> None:
    path.write_text(
        "\n".join(
            [
                "# Receiver",
                "",
                f"Listening on `{bind}:{port}`.",
                "This receiver only acknowledges synthetic no-motion commands.",
                "",
            ]
        )
    )
