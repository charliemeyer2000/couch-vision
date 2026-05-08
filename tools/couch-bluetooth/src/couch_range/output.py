from __future__ import annotations

from pathlib import Path

from .telemetry import ensure_run_dir

RUN_DATA_PATHS = (
    Path("raw/samples.csv"),
    Path("raw/samples.jsonl"),
    Path("report.md"),
    Path("analysis/summary.json"),
)


def prepare_output_dir(path: Path, force: bool = False) -> Path:
    existing_run_data = [
        run_path for run_path in RUN_DATA_PATHS if (path / run_path).exists()
    ]
    if existing_run_data and not force:
        found = ", ".join(str(run_path) for run_path in existing_run_data)
        raise FileExistsError(
            f"refusing to use output directory with existing run data: {found}"
        )
    return ensure_run_dir(path)
