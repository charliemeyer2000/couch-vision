"""Pure differential-drive helpers.

Kept free of ROS imports so the kinematics can be unit tested in isolation.
"""

from __future__ import annotations

import math


def twist_to_erpm(
    linear_x: float,
    angular_z: float,
    wheel_radius: float,
    wheel_separation: float,
    gear_ratio: int,
    pole_pairs: int,
    max_erpm: int,
    *,
    invert_master: bool = True,
    invert_slave: bool = False,
) -> tuple[int, int]:
    """Convert a differential-drive Twist to motor ERPM targets.

    The left/right wheel requests are scaled together when saturated so the
    commanded turn radius is preserved instead of flattening to a straight line.
    Returns (master_erpm, slave_erpm), where master is the right wheel.
    """
    v_left = linear_x - (angular_z * wheel_separation / 2.0)
    v_right = linear_x + (angular_z * wheel_separation / 2.0)

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
