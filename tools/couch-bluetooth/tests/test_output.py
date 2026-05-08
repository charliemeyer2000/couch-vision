from pathlib import Path

import pytest

from couch_range.output import RUN_DATA_PATHS, prepare_output_dir


def test_prepare_output_dir_creates_fresh_dir(tmp_path: Path) -> None:
    run_dir = tmp_path / "run"

    prepared = prepare_output_dir(run_dir)

    assert prepared == run_dir
    assert run_dir.is_dir()
    assert (run_dir / "raw").is_dir()
    assert (run_dir / "analysis").is_dir()
    assert (run_dir / "charts").is_dir()


def test_prepare_output_dir_allows_existing_non_run_dir(tmp_path: Path) -> None:
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    note_path = run_dir / "notes.txt"
    note_path.write_text("not run data\n")

    prepared = prepare_output_dir(run_dir)

    assert prepared == run_dir
    assert note_path.read_text() == "not run data\n"
    assert (run_dir / "raw").is_dir()
    assert (run_dir / "analysis").is_dir()
    assert (run_dir / "charts").is_dir()


@pytest.mark.parametrize("run_data_path", RUN_DATA_PATHS)
def test_prepare_output_dir_refuses_existing_run_data(
    tmp_path: Path, run_data_path: Path
) -> None:
    run_dir = tmp_path / "run"
    existing_path = run_dir / run_data_path
    existing_path.parent.mkdir(parents=True, exist_ok=True)
    existing_path.write_text("existing data\n")

    with pytest.raises(FileExistsError, match=str(run_data_path)):
        prepare_output_dir(run_dir)


@pytest.mark.parametrize("run_data_path", RUN_DATA_PATHS)
def test_prepare_output_dir_allows_existing_run_data_with_force(
    tmp_path: Path, run_data_path: Path
) -> None:
    run_dir = tmp_path / "run"
    existing_path = run_dir / run_data_path
    existing_path.parent.mkdir(parents=True, exist_ok=True)
    existing_path.write_text("existing data\n")

    prepared = prepare_output_dir(run_dir, force=True)

    assert prepared == run_dir
    assert existing_path.read_text() == "existing data\n"
    assert (run_dir / "raw").is_dir()
    assert (run_dir / "analysis").is_dir()
    assert (run_dir / "charts").is_dir()
