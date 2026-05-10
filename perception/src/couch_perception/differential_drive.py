"""Pure differential-drive helpers.

Kept free of ROS imports so the kinematics can be unit tested in isolation.
"""

from __future__ import annotations

import math


def slew_cmd_vel(
    current: float,
    target: float,
    accel: float,
    decel: float,
    dt: float,
) -> float:
    """Slew-rate limit ``current`` toward ``target`` over a step ``dt``.

    Acceleration is "moving away from zero" or "increasing magnitude in the
    same sign", deceleration is "moving toward zero". A target on the opposite
    side of zero is handled in two phases within a single step: decelerate to
    zero at ``decel``, then accelerate toward the target at ``accel``, all
    within the same ``dt`` budget. ``accel <= 0`` or ``decel <= 0`` disables
    the corresponding limit (passthrough).
    """
    if dt <= 0.0:
        return current
    delta = target - current
    if delta == 0.0:
        return target

    # Pick rate by quadrant: same sign / heading-away-from-zero = accel,
    # opposite-sign / heading-toward-zero = decel.
    if current == 0.0 or (delta > 0) == (current > 0):
        rate = accel
    else:
        rate = decel

    if rate <= 0.0:
        return target

    # Sign change: decel from |current| to 0, then accel from 0 toward target,
    # sharing the dt budget. Lets a single step both stop and reverse cleanly.
    if (current > 0.0 and target < 0.0) or (current < 0.0 and target > 0.0):
        if decel <= 0.0:
            time_to_zero = 0.0
        else:
            time_to_zero = abs(current) / decel
        if time_to_zero >= dt:
            step = decel * dt
            return current - step if current > 0.0 else current + step
        remaining = dt - time_to_zero
        if accel <= 0.0:
            return target
        step = accel * remaining
        if step >= abs(target):
            return target
        return step if target > 0.0 else -step

    max_step = rate * dt
    if abs(delta) <= max_step:
        return target
    return current + (max_step if delta > 0.0 else -max_step)


def twist_to_erpm(
    linear_x: float,
    angular_z: float,
    wheel_radius: float,
    wheel_separation: float,
    gear_ratio: int,
    pole_pairs: int,
    max_erpm: int,
    *,
    invert_master: bool = False,
    invert_slave: bool = True,
    left_scale: float = 1.0,
    right_scale: float = 1.0,
) -> tuple[int, int]:
    """Convert a differential-drive Twist to motor ERPM targets.

    The left/right wheel requests are scaled together when saturated so the
    commanded turn radius is preserved instead of flattening to a straight line.
    Returns (master_erpm, slave_erpm), where master is the right wheel.

    left_scale / right_scale trim asymmetric drive response — bump the slow
    side up (e.g. left_scale=1.05) when the rover veers toward the fast side.
    """
    v_left = (linear_x - (angular_z * wheel_separation / 2.0)) * left_scale
    v_right = (linear_x + (angular_z * wheel_separation / 2.0)) * right_scale

    def to_erpm(v: float) -> int:
        wheel_rpm = v / (2.0 * math.pi * wheel_radius) * 60.0
        return int(round(wheel_rpm * gear_ratio * pole_pairs))

    left_erpm = to_erpm(v_left)
    right_erpm = to_erpm(v_right)

    max_mag = max(abs(left_erpm), abs(right_erpm))
    if max_mag > max_erpm > 0:
        scale = max_erpm / max_mag
        left_erpm = int(round(left_erpm * scale))
        right_erpm = int(round(right_erpm * scale))

    master_erpm = -right_erpm if invert_master else right_erpm
    slave_erpm = -left_erpm if invert_slave else left_erpm
    return master_erpm, slave_erpm
