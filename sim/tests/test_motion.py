import mujoco
import math

from couch_sim.app import RuntimeState, _apply_drive, _body_telemetry, _build_bundle
from couch_sim.check import acceptance_errors, run_headless_commands, scripted_spin_commands
from couch_sim.config import load_scenario, load_vehicle
from couch_sim.control import AssistedMixer


def test_forward_command_moves_couch_on_flat_ground() -> None:
    vehicle = load_vehicle()
    scenario = load_scenario("flat_default")
    bundle = _build_bundle(vehicle, scenario, enable_rendering=False)
    runtime = RuntimeState(scenario_name=scenario.name, mode="raw", vehicle=vehicle)
    mixer = AssistedMixer(vehicle)

    start_x = _body_telemetry(bundle, vehicle).x
    for _ in range(500):
        _apply_drive(bundle, runtime, mixer, 0.8, 0.0)
        mujoco.mj_step(bundle.model, bundle.data)

    end = _body_telemetry(bundle, vehicle)
    assert end.x > start_x + 0.2
    assert abs(end.y) < 0.4


def test_couch_rests_tilted_back_on_flat_ground() -> None:
    vehicle = load_vehicle()
    scenario = load_scenario("flat_default")
    bundle = _build_bundle(vehicle, scenario, enable_rendering=False)

    for _ in range(300):
        mujoco.mj_step(bundle.model, bundle.data)

    telemetry = _body_telemetry(bundle, vehicle)
    assert telemetry.pitch < -0.015


def test_spin_turn_reaches_90_degrees_quickly_without_torque_dropout() -> None:
    vehicle = load_vehicle()
    scenario = load_scenario("flat_default")
    bundle = _build_bundle(vehicle, scenario, enable_rendering=False)
    runtime = RuntimeState(scenario_name=scenario.name, mode="raw", vehicle=vehicle)
    mixer = AssistedMixer(vehicle)

    reached_90_at: float | None = None
    min_torque_before_90 = float("inf")
    max_traction_force_before_90 = 0.0
    max_balance_error = 0.0
    for _ in range(500):
        debug = _apply_drive(bundle, runtime, mixer, 0.0, 1.0)
        left_torque = float(debug["left_torque"])
        right_torque = float(debug["right_torque"])
        left_traction = float(debug["left_traction_force"])
        right_traction = float(debug["right_traction_force"])
        min_torque_before_90 = min(min_torque_before_90, abs(left_torque), abs(right_torque))
        max_traction_force_before_90 = max(
            max_traction_force_before_90,
            abs(left_traction),
            abs(right_traction),
        )
        max_balance_error = max(max_balance_error, abs(left_torque + right_torque))
        mujoco.mj_step(bundle.model, bundle.data)

        telemetry = _body_telemetry(bundle, vehicle)
        if abs(telemetry.yaw) >= math.pi / 2.0:
            reached_90_at = float(bundle.data.time)
            break

    assert reached_90_at is not None
    assert reached_90_at <= 1.5
    assert min_torque_before_90 >= 18.0
    assert max_traction_force_before_90 >= 20.0
    assert max_balance_error <= 1e-6


def test_spin_turn_keeps_chassis_stable_and_contacts_bounded() -> None:
    metrics = run_headless_commands(scripted_spin_commands(duration_seconds=2.0))

    assert metrics.time_to_90_degrees is not None
    assert metrics.time_to_90_degrees <= 1.5
    assert metrics.max_chassis_z - metrics.min_chassis_z < 0.04
    assert max(metrics.contacts.max_airborne_seconds.values()) <= 0.25


def test_acceptance_gate_accepts_nominal_metrics() -> None:
    simulation = run_headless_commands(scripted_spin_commands(duration_seconds=2.0))
    metrics = type(
        "BagMetrics",
        (),
        {
            "pure_turn_count": 10,
            "sim_to_real_yaw_rate_ratio": 1.0,
            "simulation": simulation,
        },
    )()

    assert acceptance_errors(metrics) == []
