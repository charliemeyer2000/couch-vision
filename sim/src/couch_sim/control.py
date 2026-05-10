"""Control logic and telemetry helpers for the couch simulator."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from .config import VehicleConfig

DriveMode = Literal["raw", "assisted"]


@dataclass(slots=True)
class Command:
    """Normalized user intent and resolved linear/angular targets."""

    forward: float
    turn: float
    linear_mps: float
    angular_rps: float


@dataclass(slots=True)
class Telemetry:
    """Vehicle state inputs needed by the assist controller."""

    x: float
    y: float
    yaw: float
    pitch: float
    roll: float
    forward_speed: float
    lateral_speed: float
    yaw_rate: float
    left_wheel_speed: float
    right_wheel_speed: float


@dataclass(slots=True)
class DriveOutputs:
    """Final per-wheel targets and assist bookkeeping."""

    left_target_speed: float
    right_target_speed: float
    angular_correction: float
    heading_reference: float | None
    heading_error: float


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a scalar to a closed interval."""
    return max(lower, min(upper, value))


def wrap_angle(angle: float) -> float:
    """Wrap radians into [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def make_command(forward: float, turn: float, vehicle: VehicleConfig) -> Command:
    """Scale normalized user input into linear/angular targets."""
    fwd = clamp(forward, -1.0, 1.0)
    yaw = clamp(turn, -1.0, 1.0)
    spin_in_place = abs(yaw) > vehicle.spin_in_place_turn_threshold
    linear_mps = 0.0 if spin_in_place else fwd * vehicle.max_linear_speed
    angular_scale = vehicle.spin_in_place_angular_scale if spin_in_place else 1.0
    return Command(
        forward=fwd,
        turn=yaw,
        linear_mps=linear_mps,
        angular_rps=yaw * vehicle.max_angular_speed * angular_scale,
    )


def wheel_targets_from_command(command: Command, vehicle: VehicleConfig) -> tuple[float, float]:
    """Convert body-frame targets into rear wheel angular velocities."""
    half_track = vehicle.wheel_separation / 2.0
    left_linear = command.linear_mps - command.angular_rps * half_track
    right_linear = command.linear_mps + command.angular_rps * half_track
    return left_linear / vehicle.wheel_radius, right_linear / vehicle.wheel_radius


@dataclass(slots=True)
class AssistedMixer:
    """Optional heading-hold helper for cross-slope teleop."""

    vehicle: VehicleConfig
    heading_reference: float | None = None

    def reset(self) -> None:
        """Clear heading hold state."""
        self.heading_reference = None

    def mix(self, mode: DriveMode, command: Command, telemetry: Telemetry) -> DriveOutputs:
        """Compute wheel speed targets for the chosen mode."""
        angular_correction = 0.0
        heading_error = 0.0

        if mode == "assisted":
            should_hold_heading = (
                abs(command.turn) < self.vehicle.assist_turn_deadband
                and abs(command.linear_mps) > self.vehicle.assist_activation_speed
            )
            if should_hold_heading:
                if self.heading_reference is None:
                    self.heading_reference = telemetry.yaw
                heading_error = wrap_angle(self.heading_reference - telemetry.yaw)
                angular_correction = (
                    self.vehicle.assist_heading_gain * heading_error
                    - self.vehicle.assist_yaw_rate_gain * telemetry.yaw_rate
                    - self.vehicle.assist_lateral_gain * telemetry.lateral_speed
                )
            else:
                self.heading_reference = telemetry.yaw if abs(command.linear_mps) > 0.02 else None
        else:
            self.heading_reference = None

        corrected = Command(
            forward=command.forward,
            turn=command.turn,
            linear_mps=command.linear_mps,
            angular_rps=clamp(
                command.angular_rps + angular_correction,
                -self.vehicle.max_angular_speed * 1.5,
                self.vehicle.max_angular_speed * 1.5,
            ),
        )
        left_target, right_target = wheel_targets_from_command(corrected, self.vehicle)
        return DriveOutputs(
            left_target_speed=left_target,
            right_target_speed=right_target,
            angular_correction=angular_correction,
            heading_reference=self.heading_reference,
            heading_error=heading_error,
        )
