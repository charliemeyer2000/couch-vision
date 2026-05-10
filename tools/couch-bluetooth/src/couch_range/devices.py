from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
import json
import shutil
import socket
import subprocess


@dataclass(slots=True)
class CommandResult:
    command: list[str]
    available: bool
    returncode: int | None
    stdout: str
    stderr: str
    error: str


@dataclass(slots=True)
class Inventory:
    hostname: str
    commands: dict[str, CommandResult]
    input_devices: list[str]
    notes: list[str]


def command_exists(command: str) -> bool:
    return shutil.which(command) is not None


def _subprocess_text(value: str | bytes | None) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode(errors="replace").strip()
    return value.strip()


def run_command(command: list[str], timeout_s: float = 5.0) -> CommandResult:
    if not command or not command_exists(command[0]):
        return CommandResult(
            command,
            False,
            None,
            "",
            "",
            f"{command[0] if command else 'command'} not found",
        )
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired as exc:
        return CommandResult(
            command,
            True,
            None,
            _subprocess_text(exc.stdout),
            _subprocess_text(exc.stderr),
            "timed out",
        )
    except OSError as exc:
        return CommandResult(command, True, None, "", "", str(exc))
    return CommandResult(
        command,
        True,
        completed.returncode,
        completed.stdout.strip(),
        completed.stderr.strip(),
        "",
    )


def run_shell(
    label: str, script: str, timeout_s: float = 5.0
) -> tuple[str, CommandResult]:
    result = subprocess.run(
        ["/bin/sh", "-c", script],
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout_s,
    )
    return label, CommandResult(
        ["/bin/sh", "-c", script],
        True,
        result.returncode,
        result.stdout.strip(),
        result.stderr.strip(),
        "",
    )


def collect_inventory() -> Inventory:
    commands: dict[str, CommandResult] = {}
    command_sets: dict[str, list[str]] = {
        "hostname": ["hostname"],
        "ip_addr": ["ip", "addr"],
        "tailscale_status": ["tailscale", "status"],
        "tailscale_status_json": ["tailscale", "status", "--json"],
        "bluetoothctl_show": ["bluetoothctl", "show"],
        "bluetoothctl_devices": ["bluetoothctl", "devices"],
        "btmgmt_info": ["btmgmt", "info"],
        "hciconfig": ["hciconfig", "-a"],
        "lsusb": ["lsusb"],
        "rfkill": ["rfkill", "list"],
        "evtest_version": ["evtest", "--version"],
    }
    for label, command in command_sets.items():
        commands[label] = run_command(command)

    if Path("/bin/sh").exists():
        try:
            label, result = run_shell(
                "dmesg_bluetooth",
                "dmesg 2>/dev/null | grep -i bluetooth | tail -n 80",
                timeout_s=5.0,
            )
            commands[label] = result
        except (OSError, subprocess.TimeoutExpired) as exc:
            commands["dmesg_bluetooth"] = CommandResult(
                ["/bin/sh", "-c", "dmesg | grep -i bluetooth"],
                True,
                None,
                "",
                "",
                str(exc),
            )

    input_devices = (
        sorted(str(path) for path in Path("/dev/input").glob("*"))
        if Path("/dev/input").exists()
        else []
    )
    notes = []
    for label, result in commands.items():
        if not result.available:
            notes.append(f"{label}: command missing")
        elif result.returncode not in (0, None):
            notes.append(f"{label}: exited {result.returncode}")
        elif result.error:
            notes.append(f"{label}: {result.error}")

    return Inventory(socket.gethostname(), commands, input_devices, notes)


def write_inventory(out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    inventory = collect_inventory()
    path = out_dir / "inventory.json"
    path.write_text(json.dumps(asdict(inventory), indent=2, sort_keys=True) + "\n")
    markdown = out_dir / "inventory.md"
    markdown.write_text(render_inventory_markdown(inventory))
    return path


def render_inventory_markdown(inventory: Inventory) -> str:
    lines = [
        "# Device Inventory",
        "",
        f"- Hostname: `{inventory.hostname}`",
        f"- Input devices found: {len(inventory.input_devices)}",
        "",
        "## Input Devices",
        "",
    ]
    if inventory.input_devices:
        lines.extend(f"- `{item}`" for item in inventory.input_devices)
    else:
        lines.append("- none found")
    lines.extend(["", "## Command Probes", ""])
    for label, result in inventory.commands.items():
        status = "missing" if not result.available else f"exit {result.returncode}"
        lines.append(f"### {label} ({status})")
        lines.append("")
        if result.error:
            lines.append(f"Error: `{result.error}`")
            lines.append("")
        output = result.stdout or result.stderr
        if output:
            lines.append("```text")
            lines.append(output[:4000])
            lines.append("```")
            lines.append("")
    if inventory.notes:
        lines.extend(["## Notes", ""])
        lines.extend(f"- {note}" for note in inventory.notes)
        lines.append("")
    return "\n".join(lines)
