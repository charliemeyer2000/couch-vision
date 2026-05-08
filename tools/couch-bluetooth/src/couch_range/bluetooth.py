from __future__ import annotations

import re
import subprocess


RSSI_RE = re.compile(r"RSSI:\s*(-?\d+)")


def bluetooth_rssi(address: str, timeout_s: float = 3.0) -> float | None:
    if not address:
        return None
    try:
        completed = subprocess.run(
            ["bluetoothctl", "info", address],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    match = RSSI_RE.search(completed.stdout)
    if match is None:
        return None
    return float(match.group(1))
