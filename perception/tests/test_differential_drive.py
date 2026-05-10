from __future__ import annotations

import math

import pytest

from couch_perception.differential_drive import slew_cmd_vel, twist_to_erpm


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


# ── slew_cmd_vel ──────────────────────────────────────────────────────────────


def test_slew_passthrough_when_target_already_reached() -> None:
    assert slew_cmd_vel(1.0, 1.0, accel=4.0, decel=4.0, dt=0.05) == 1.0


def test_slew_dt_zero_holds_current() -> None:
    assert slew_cmd_vel(0.5, 1.0, accel=4.0, decel=4.0, dt=0.0) == 0.5


def test_slew_accel_within_step() -> None:
    """Step is small enough that we land exactly on target."""
    # delta = 0.1, accel*dt = 4.0 * 0.05 = 0.2 → snap to target
    assert slew_cmd_vel(0.0, 0.1, accel=4.0, decel=4.0, dt=0.05) == 0.1


def test_slew_accel_clipped_to_rate() -> None:
    """Step is bigger than accel*dt — clipped at the rate limit."""
    # accel*dt = 0.2, target requires 1.0
    assert math.isclose(
        slew_cmd_vel(0.0, 1.0, accel=4.0, decel=4.0, dt=0.05), 0.2
    )


def test_slew_decel_uses_decel_rate() -> None:
    """Heading toward zero from the same sign uses decel rate."""
    # current 1.0 → target 0.5, delta = -0.5 (heading toward zero)
    # decel*dt = 2.0 * 0.05 = 0.1
    assert math.isclose(
        slew_cmd_vel(1.0, 0.5, accel=4.0, decel=2.0, dt=0.05), 0.9
    )


def test_slew_asymmetric_rates() -> None:
    """Accel rate must NOT be used when actually decelerating, and vice versa."""
    # Strong accel, weak decel: braking should be the slow one.
    fast_accel = slew_cmd_vel(0.0, 1.0, accel=10.0, decel=1.0, dt=0.1)
    slow_decel = slew_cmd_vel(1.0, 0.0, accel=10.0, decel=1.0, dt=0.1)
    assert math.isclose(fast_accel, 1.0)  # 10*0.1 = 1.0, snap to target
    assert math.isclose(slow_decel, 0.9)  # 1.0 - 1*0.1


def test_slew_sign_flip_within_one_step() -> None:
    """+0.1 → -0.5 with rates that allow both phases in one dt."""
    # decel from 0.1 to 0 takes 0.1/2.0 = 0.05s. Remaining dt = 0.05s.
    # accel from 0 toward -0.5 at rate 4.0 → step = 4*0.05 = 0.2
    out = slew_cmd_vel(0.1, -0.5, accel=4.0, decel=2.0, dt=0.10)
    assert math.isclose(out, -0.2, abs_tol=1e-9)


def test_slew_sign_flip_blocked_by_decel() -> None:
    """If we can't even reach zero in one dt, we just decel toward it."""
    # decel*dt = 1.0*0.05 = 0.05, current 1.0 → 0.95 (not crossing zero)
    out = slew_cmd_vel(1.0, -1.0, accel=10.0, decel=1.0, dt=0.05)
    assert math.isclose(out, 0.95)


@pytest.mark.parametrize(
    "current,target",
    [(0.0, 0.0), (1.0, 1.0), (-0.5, -0.5)],
)
def test_slew_no_op_when_already_there(current: float, target: float) -> None:
    assert slew_cmd_vel(current, target, accel=4.0, decel=4.0, dt=0.05) == target


def test_slew_negative_rate_is_passthrough() -> None:
    """rate <= 0 means no limit — used by bypass_ramp upstream."""
    assert slew_cmd_vel(0.0, 1.0, accel=0.0, decel=4.0, dt=0.05) == 1.0
