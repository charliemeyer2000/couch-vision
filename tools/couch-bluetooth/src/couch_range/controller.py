from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True)
class ControllerDevice:
    path: str
    name: str


def list_input_devices() -> list[ControllerDevice]:
    devices: list[ControllerDevice] = []
    root = Path("/dev/input")
    if not root.exists():
        return devices
    for event_path in sorted(root.glob("event*")):
        name = _read_device_name(event_path)
        devices.append(ControllerDevice(str(event_path), name))
    return devices


def best_controller_name() -> str:
    names = [
        device.name
        for device in list_input_devices()
        if _looks_like_controller(device.name)
    ]
    if names:
        return names[0]
    devices = list_input_devices()
    return devices[0].name if devices else ""


def _looks_like_controller(name: str) -> bool:
    lowered = name.lower()
    return any(
        token in lowered for token in ["xbox", "controller", "gamepad", "joystick"]
    )


def _read_device_name(event_path: Path) -> str:
    name_path = Path("/sys/class/input") / event_path.name / "device/name"
    try:
        return name_path.read_text().strip()
    except OSError:
        return event_path.name
