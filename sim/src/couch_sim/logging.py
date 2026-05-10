"""Logging helpers for simulation runs."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


@dataclass(slots=True)
class RunLogger:
    """Collect and write simulation samples."""

    metadata: dict[str, Any]
    samples: list[dict[str, Any]] = field(default_factory=list)

    def add_sample(self, sample: dict[str, Any]) -> None:
        """Append a sampled telemetry point."""
        self.samples.append(sample)

    def write(self, output_path: Path | None) -> Path:
        """Write the run log to disk and return the path."""
        if output_path is None:
            timestamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
            output_path = Path("output/logs") / f"{self.metadata['scenario']}_{self.metadata['mode']}_{timestamp}.json"
        output_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "metadata": self.metadata,
            "sample_count": len(self.samples),
            "samples": self.samples,
        }
        output_path.write_text(json.dumps(payload, indent=2))
        return output_path
