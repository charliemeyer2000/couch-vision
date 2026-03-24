from __future__ import annotations

from couch_perception.differential_drive import twist_to_erpm


def test_twist_to_erpm_scales_both_wheels_when_saturated() -> None:
    master_erpm, slave_erpm = twist_to_erpm(
        1.0,
        0.05,
        wheel_radius=0.0415,
        wheel_separation=0.5,
        gear_ratio=1,
        pole_pairs=1,
        max_erpm=100,
        invert_master=False,
        invert_slave=False,
    )

    assert abs(master_erpm) <= 100
    assert abs(slave_erpm) <= 100
    assert abs(master_erpm) != abs(slave_erpm)


def test_twist_to_erpm_preserves_direction_without_saturation() -> None:
    master_erpm, slave_erpm = twist_to_erpm(
        0.05,
        0.1,
        wheel_radius=0.0415,
        wheel_separation=0.5,
        gear_ratio=1,
        pole_pairs=1,
        max_erpm=1000,
        invert_master=False,
        invert_slave=False,
    )

    assert master_erpm > slave_erpm
    assert master_erpm > 0
    assert slave_erpm > 0
