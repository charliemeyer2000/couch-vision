"""Headless simulation diagnostics and bag-response checks."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import mujoco
import numpy as np

from .app import RuntimeState, _apply_drive, _body_telemetry, _build_bundle
from .config import load_scenario, load_vehicle, simulator_root
from .control import AssistedMixer, clamp

SIM_DT = 0.005
DEFAULT_BAG = simulator_root().parent / "bags/2026-05-08_18-25-49_0.mcap"


@dataclass(slots=True)
class CommandSample:
    """A normalized headless command sample."""

    time: float
    forward: float
    turn: float


@dataclass(slots=True)
class ContactMetrics:
    """Contact stability summary for support geoms."""

    duty_cycle: dict[str, float]
    max_airborne_seconds: dict[str, float]


@dataclass(slots=True)
class SimulationMetrics:
    """Headless run metrics used by tests and the CLI."""

    duration_seconds: float
    final_yaw_degrees: float
    max_abs_yaw_rate: float
    p90_abs_yaw_rate: float
    max_chassis_z: float
    min_chassis_z: float
    max_abs_roll_degrees: float
    max_abs_pitch_degrees: float
    time_to_90_degrees: float | None
    contacts: ContactMetrics


@dataclass(slots=True)
class BagTurnMetrics:
    """Real-bag turn envelope from /cmd_vel and /wheel_odom."""

    command_count: int
    odom_count: int
    pure_turn_count: int
    real_p90_abs_yaw_rate: float
    sim_p90_abs_yaw_rate: float
    sim_to_real_yaw_rate_ratio: float
    simulation: SimulationMetrics


def _contact_touching(bundle: object) -> dict[str, bool]:
    assert hasattr(bundle, "data")
    assert hasattr(bundle, "contact_geom_ids")
    touching = {name: False for name in bundle.contact_geom_ids}
    for contact_idx in range(bundle.data.ncon):
        contact = bundle.data.contact[contact_idx]
        for name, geom_id in bundle.contact_geom_ids.items():
            if contact.geom1 == geom_id or contact.geom2 == geom_id:
                touching[name] = True
    return touching


def _normalize_cmd_vel(linear_x: float, angular_z: float) -> tuple[float, float]:
    forward = clamp(linear_x / 3.0, -1.0, 1.0)
    turn = clamp(angular_z / 1.0, -1.0, 1.0)
    return forward, turn


def scripted_spin_commands(duration_seconds: float = 3.0, turn: float = 1.0) -> list[CommandSample]:
    """Build a full-turn command sequence."""
    steps = int(duration_seconds / SIM_DT)
    return [CommandSample(time=idx * SIM_DT, forward=0.0, turn=turn) for idx in range(steps)]


def run_headless_commands(
    commands: Iterable[CommandSample],
    *,
    scenario_name: str = "flat_default",
    mode: str = "raw",
) -> SimulationMetrics:
    """Run deterministic headless commands and return stability metrics."""
    vehicle = load_vehicle()
    scenario = load_scenario(scenario_name)
    bundle = _build_bundle(vehicle, scenario, enable_rendering=False)
    runtime = RuntimeState(scenario_name=scenario.name, mode=mode, vehicle=vehicle)
    mixer = AssistedMixer(vehicle)

    command_list = list(commands)
    if not command_list:
        raise ValueError("At least one command sample is required")

    contact_counts = {name: 0 for name in bundle.contact_geom_ids}
    airborne_streaks = {name: 0 for name in bundle.contact_geom_ids}
    max_airborne_streaks = {name: 0 for name in bundle.contact_geom_ids}
    max_chassis_z = -math.inf
    min_chassis_z = math.inf
    max_abs_roll = 0.0
    max_abs_pitch = 0.0
    max_abs_yaw_rate = 0.0
    abs_yaw_rates: list[float] = []
    time_to_90: float | None = None

    for sample in command_list:
        _apply_drive(bundle, runtime, mixer, sample.forward, sample.turn)
        mujoco.mj_step(bundle.model, bundle.data)

        touching = _contact_touching(bundle)
        for name, is_touching in touching.items():
            if is_touching:
                contact_counts[name] += 1
                airborne_streaks[name] = 0
            else:
                airborne_streaks[name] += 1
                max_airborne_streaks[name] = max(max_airborne_streaks[name], airborne_streaks[name])

        telemetry = _body_telemetry(bundle, vehicle)
        chassis_z = float(bundle.data.xpos[bundle.chassis_body_id][2])
        max_chassis_z = max(max_chassis_z, chassis_z)
        min_chassis_z = min(min_chassis_z, chassis_z)
        max_abs_roll = max(max_abs_roll, abs(math.degrees(telemetry.roll)))
        max_abs_pitch = max(max_abs_pitch, abs(math.degrees(telemetry.pitch)))
        abs_yaw_rate = abs(telemetry.yaw_rate)
        abs_yaw_rates.append(abs_yaw_rate)
        max_abs_yaw_rate = max(max_abs_yaw_rate, abs_yaw_rate)
        if time_to_90 is None and abs(telemetry.yaw) >= math.pi / 2.0:
            time_to_90 = float(bundle.data.time)

    final = _body_telemetry(bundle, vehicle)
    sample_count = len(command_list)
    return SimulationMetrics(
        duration_seconds=float(bundle.data.time),
        final_yaw_degrees=math.degrees(final.yaw),
        max_abs_yaw_rate=max_abs_yaw_rate,
        p90_abs_yaw_rate=float(np.percentile(np.array(abs_yaw_rates), 90)),
        max_chassis_z=max_chassis_z,
        min_chassis_z=min_chassis_z,
        max_abs_roll_degrees=max_abs_roll,
        max_abs_pitch_degrees=max_abs_pitch,
        time_to_90_degrees=time_to_90,
        contacts=ContactMetrics(
            duty_cycle={name: count / sample_count for name, count in contact_counts.items()},
            max_airborne_seconds={name: streak * SIM_DT for name, streak in max_airborne_streaks.items()},
        ),
    )


def _read_bag_turn_data(bag_path: Path) -> tuple[list[tuple[float, float, float]], list[tuple[float, float]]]:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    cmd_vel: list[tuple[float, float, float]] = []
    wheel_odom: list[tuple[float, float]] = []
    with bag_path.open("rb") as bag_file:
        reader = make_reader(bag_file, decoder_factories=[DecoderFactory()])
        for _schema, channel, message, decoded in reader.iter_decoded_messages():
            timestamp = message.log_time / 1e9
            if channel.topic == "/cmd_vel":
                cmd_vel.append((timestamp, float(decoded.linear.x), float(decoded.angular.z)))
            elif channel.topic == "/wheel_odom":
                wheel_odom.append((timestamp, float(decoded.twist.twist.angular.z)))
    return cmd_vel, wheel_odom


def _bag_commands(cmd_vel: list[tuple[float, float, float]], max_duration: float) -> list[CommandSample]:
    if not cmd_vel:
        raise ValueError("No /cmd_vel samples found in bag")
    ordered = sorted(cmd_vel, key=lambda sample: sample[0])
    start = next(
        (timestamp for timestamp, linear_x, angular_z in ordered if abs(linear_x) < 0.05 and abs(angular_z) > 0.2),
        ordered[0][0],
    )
    samples: list[CommandSample] = []
    command_idx = 0
    step_count = int(max_duration / SIM_DT)
    for step_idx in range(step_count):
        elapsed = step_idx * SIM_DT
        target_time = start + elapsed
        while command_idx < len(ordered) - 1 and ordered[command_idx + 1][0] <= target_time:
            command_idx += 1
        _timestamp, linear_x, angular_z = ordered[command_idx]
        forward, turn = _normalize_cmd_vel(linear_x, angular_z)
        samples.append(CommandSample(time=elapsed, forward=forward, turn=turn))
    return samples


def compare_bag_turn_response(
    bag_path: Path = DEFAULT_BAG,
    *,
    scenario_name: str = "flat_default",
    max_duration: float = 12.0,
) -> BagTurnMetrics:
    """Replay bag commands and compare yaw-rate envelope against wheel odom."""
    cmd_vel, wheel_odom = _read_bag_turn_data(bag_path)
    commands = _bag_commands(cmd_vel, max_duration)
    simulation = run_headless_commands(commands, scenario_name=scenario_name)

    ordered_odom = sorted(wheel_odom, key=lambda sample: sample[0])
    odom_start = next(
        (timestamp for timestamp, linear_x, angular_z in sorted(cmd_vel, key=lambda sample: sample[0]) if abs(linear_x) < 0.05 and abs(angular_z) > 0.2),
        ordered_odom[0][0] if ordered_odom else 0.0,
    )
    real_yaw_rates = np.array(
        [yaw_rate for timestamp, yaw_rate in ordered_odom if odom_start <= timestamp <= odom_start + max_duration]
    )
    if real_yaw_rates.size == 0:
        raise ValueError("No /wheel_odom samples found in bag")
    pure_turn_count = sum(1 for _timestamp, linear_x, angular_z in cmd_vel if abs(linear_x) < 0.05 and abs(angular_z) > 0.2)
    real_p90 = float(np.percentile(np.abs(real_yaw_rates), 90))
    sim_p90 = simulation.p90_abs_yaw_rate
    ratio = sim_p90 / real_p90 if real_p90 > 1e-9 else math.inf
    return BagTurnMetrics(
        command_count=len(cmd_vel),
        odom_count=len(wheel_odom),
        pure_turn_count=pure_turn_count,
        real_p90_abs_yaw_rate=real_p90,
        sim_p90_abs_yaw_rate=sim_p90,
        sim_to_real_yaw_rate_ratio=ratio,
        simulation=simulation,
    )


def acceptance_errors(metrics: BagTurnMetrics) -> list[str]:
    """Return gate failures for the headless bag check."""
    errors: list[str] = []
    if metrics.pure_turn_count == 0:
        errors.append("bag has no pure-turn /cmd_vel windows")
    if not 0.25 <= metrics.sim_to_real_yaw_rate_ratio <= 2.5:
        errors.append(f"sim/real yaw-rate ratio {metrics.sim_to_real_yaw_rate_ratio:.2f} outside [0.25, 2.5]")
    if metrics.simulation.max_chassis_z - metrics.simulation.min_chassis_z >= 0.04:
        errors.append("chassis vertical travel exceeded 0.04 m")
    max_airborne = max(metrics.simulation.contacts.max_airborne_seconds.values())
    if max_airborne > 0.25:
        errors.append(f"wheel/caster airborne streak {max_airborne:.3f}s exceeded 0.250s")
    return errors


def _json_default(value: object) -> object:
    if hasattr(value, "__dataclass_fields__"):
        return asdict(value)
    raise TypeError(f"Cannot JSON encode {type(value).__name__}")


def main() -> None:
    """CLI entrypoint for headless sim checks."""
    parser = argparse.ArgumentParser(description="Run headless MuJoCo couch diagnostics")
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--scenario", default="flat_default")
    parser.add_argument("--max-duration", type=float, default=12.0)
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()

    metrics = compare_bag_turn_response(args.bag, scenario_name=args.scenario, max_duration=args.max_duration)
    errors = acceptance_errors(metrics)
    payload = json.dumps(
        {
            "passed": not errors,
            "errors": errors,
            "metrics": metrics,
        },
        default=_json_default,
        indent=2,
    )
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(payload)
    print(payload)
    if errors:
        sys.exit(1)


if __name__ == "__main__":
    main()
