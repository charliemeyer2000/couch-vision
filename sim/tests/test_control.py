import math

from couch_sim.config import load_vehicle
from couch_sim.app import _available_wheel_torque, _spin_torque_pair, _wheel_peak_torque
from couch_sim.control import AssistedMixer, Telemetry, make_command, wheel_targets_from_command


def _telemetry(yaw: float = 0.0, lateral_speed: float = 0.0, yaw_rate: float = 0.0) -> Telemetry:
    return Telemetry(
        x=0.0,
        y=0.0,
        yaw=yaw,
        pitch=0.0,
        roll=0.0,
        forward_speed=0.5,
        lateral_speed=lateral_speed,
        yaw_rate=yaw_rate,
        left_wheel_speed=0.0,
        right_wheel_speed=0.0,
    )


def _wheel_telemetry(left_speed: float, right_speed: float) -> Telemetry:
    telemetry = _telemetry()
    telemetry.left_wheel_speed = left_speed
    telemetry.right_wheel_speed = right_speed
    return telemetry


def test_rotation_in_place_targets_are_opposite() -> None:
    vehicle = load_vehicle()
    command = make_command(0.0, 1.0, vehicle)
    left, right = wheel_targets_from_command(command, vehicle)
    assert left < 0 < right
    assert math.isclose(abs(left), abs(right), rel_tol=1e-6)


def test_assist_adds_countersteer_when_heading_drifts() -> None:
    vehicle = load_vehicle()
    mixer = AssistedMixer(vehicle)
    command = make_command(0.6, 0.0, vehicle)
    initial = mixer.mix("assisted", command, _telemetry())
    drifted = mixer.mix("assisted", command, _telemetry(yaw=0.15, lateral_speed=0.2, yaw_rate=0.05))
    assert initial.heading_reference is not None
    assert drifted.heading_error < 0
    assert drifted.angular_correction < 0


def test_raw_mode_does_not_latch_heading() -> None:
    vehicle = load_vehicle()
    mixer = AssistedMixer(vehicle)
    command = make_command(0.4, 0.0, vehicle)
    outputs = mixer.mix("raw", command, _telemetry(yaw=0.25))
    assert outputs.heading_reference is None
    assert outputs.angular_correction == 0.0


def test_torque_model_drops_with_speed() -> None:
    vehicle = load_vehicle()
    peak = _wheel_peak_torque(vehicle)
    medium = _available_wheel_torque(vehicle, 8.0)
    fast = _available_wheel_torque(vehicle, 40.0)
    assert peak > 0.0
    assert peak >= medium >= fast


def test_spin_torque_is_coupled_equal_and_opposite() -> None:
    vehicle = load_vehicle()
    left_torque, right_torque, left_limit, right_limit = _spin_torque_pair(
        vehicle,
        left_target_speed=-12.0,
        right_target_speed=12.0,
        telemetry=_wheel_telemetry(left_speed=-30.0, right_speed=8.0),
    )
    assert math.isclose(abs(left_torque), abs(right_torque), rel_tol=1e-9)
    assert left_torque == -right_torque
    assert left_limit == right_limit
    assert abs(left_torque) > 8.0


def test_turn_input_snaps_to_spin_in_place_when_forward_is_small() -> None:
    vehicle = load_vehicle()
    command = make_command(0.1, 1.0, vehicle)
    assert command.linear_mps == 0.0
    assert command.angular_rps > vehicle.max_angular_speed


def test_any_real_turn_overrides_forward_into_spin_mode() -> None:
    vehicle = load_vehicle()
    command = make_command(0.9, -0.3, vehicle)
    assert command.linear_mps == 0.0
    assert command.angular_rps < 0.0
