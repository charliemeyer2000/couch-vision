"""Standalone MuJoCo couch teleop sandbox."""

from __future__ import annotations

import argparse
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "hide")
import pygame

from .config import ScenarioConfig, VehicleConfig, available_scenarios, load_scenario, load_vehicle
from .control import AssistedMixer, DriveMode, Telemetry, clamp, make_command
from .logging import RunLogger
from .model import ModelHandles, build_mjcf

WINDOW_WIDTH = 1400
WINDOW_HEIGHT = 860
PANEL_WIDTH = 320
SCENE_WIDTH = WINDOW_WIDTH - PANEL_WIDTH
SCENE_HEIGHT = WINDOW_HEIGHT
RENDER_FPS = 60
SIM_DT = 0.005

TUNABLES: list[tuple[str, float]] = [
    ("rear_drive_left_gain", 0.01),
    ("rear_drive_right_gain", 0.01),
    ("rear_drive_left_slip", 0.01),
    ("rear_drive_right_slip", 0.01),
    ("caster_damping", 0.05),
    ("caster_friction", 0.05),
    ("caster_trail", 0.005),
    ("caster_contact_radius_extra", 0.001),
    ("caster_contact_margin", 0.002),
    ("caster_contact_solref_time", 0.002),
    ("com_offset_y", 0.01),
    ("assist_heading_gain", 0.1),
    ("assist_lateral_gain", 0.1),
    ("assist_yaw_rate_gain", 0.05),
]

SCENARIO_HOTKEYS = {
    pygame.K_1: "flat_default",
    pygame.K_2: "cross_slope",
    pygame.K_3: "uphill_downhill",
    pygame.K_4: "bumpy_washboard",
    pygame.K_5: "split_friction",
    pygame.K_6: "intersecting_grades",
}


@dataclass(slots=True)
class RuntimeState:
    """Mutable runtime state for the application loop."""

    scenario_name: str
    mode: DriveMode
    vehicle: VehicleConfig
    selected_tunable: int = 0
    analog_active: bool = False
    analog_forward: float = 0.0
    analog_turn: float = 0.0
    camera_orbit_active: bool = False
    camera_last_mouse_pos: tuple[int, int] | None = None


@dataclass(slots=True)
class SimBundle:
    """MuJoCo model, data, and lookup handles."""

    model: mujoco.MjModel
    data: mujoco.MjData
    handles: ModelHandles
    chassis_body_id: int
    left_joint_id: int
    right_joint_id: int
    left_actuator_id: int
    right_actuator_id: int
    steering_joint_ids: tuple[int, int]
    contact_geom_ids: dict[str, int]
    renderer: Any | None = None
    camera: Any | None = None


def _create_camera(vehicle: VehicleConfig, body_id: int) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = body_id
    camera.distance = vehicle.camera_distance
    camera.azimuth = vehicle.camera_azimuth_deg
    camera.elevation = vehicle.camera_elevation_deg
    camera.lookat[:] = (0.0, 0.0, vehicle.com_height)
    return camera


def _build_bundle(vehicle: VehicleConfig, scenario: ScenarioConfig, enable_rendering: bool) -> SimBundle:
    model = mujoco.MjModel.from_xml_string(build_mjcf(vehicle, scenario))
    data = mujoco.MjData(model)
    handles = ModelHandles()
    chassis_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, handles.chassis_body)
    left_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, handles.left_wheel_joint)
    right_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, handles.right_wheel_joint)
    left_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "rear_left_motor")
    right_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "rear_right_motor")
    steering_joint_ids = tuple(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in handles.steering_joints
    )
    contact_geom_ids = {
        name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name in handles.contact_geoms
    }

    renderer = None
    camera = None
    if enable_rendering:
        renderer = mujoco.Renderer(model, width=SCENE_WIDTH, height=SCENE_HEIGHT)
        camera = _create_camera(vehicle, chassis_body_id)
    return SimBundle(
        model=model,
        data=data,
        handles=handles,
        chassis_body_id=chassis_body_id,
        left_joint_id=left_joint_id,
        right_joint_id=right_joint_id,
        left_actuator_id=left_actuator_id,
        right_actuator_id=right_actuator_id,
        steering_joint_ids=steering_joint_ids,
        contact_geom_ids=contact_geom_ids,
        renderer=renderer,
        camera=camera,
    )


def _reset_bundle(bundle: SimBundle) -> None:
    mujoco.mj_resetData(bundle.model, bundle.data)
    mujoco.mj_forward(bundle.model, bundle.data)


def _vehicle_copy(vehicle: VehicleConfig) -> VehicleConfig:
    return VehicleConfig(**vehicle.to_dict())


def _quat_to_euler_xyz(quat: np.ndarray) -> tuple[float, float, float]:
    """Convert a MuJoCo quaternion `[w, x, y, z]` to XYZ Euler radians."""
    w, x, y, z = [float(value) for value in quat]

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _body_telemetry(bundle: SimBundle, vehicle: VehicleConfig) -> Telemetry:
    data = bundle.data
    body_id = bundle.chassis_body_id
    x, y, z = data.xpos[body_id]
    quat = data.xquat[body_id]
    roll, pitch, yaw = _quat_to_euler_xyz(quat)
    body_cvel = data.cvel[body_id]
    local_rotation = data.xmat[body_id].reshape(3, 3)
    world_linear = np.array(data.qvel[:3], dtype=float)
    body_linear = local_rotation.T @ world_linear
    yaw_rate = float(body_cvel[2])
    left_speed = float(data.qvel[bundle.model.jnt_dofadr[bundle.left_joint_id]])
    right_speed = float(data.qvel[bundle.model.jnt_dofadr[bundle.right_joint_id]])
    return Telemetry(
        x=float(x),
        y=float(y),
        yaw=float(yaw),
        pitch=float(pitch),
        roll=float(roll),
        forward_speed=float(body_linear[0]),
        lateral_speed=float(body_linear[1]),
        yaw_rate=yaw_rate,
        left_wheel_speed=left_speed,
        right_wheel_speed=right_speed,
    )


def _input_from_keyboard(keys: pygame.key.ScancodeWrapper) -> tuple[float, float]:
    forward = 0.0
    turn = 0.0
    if keys[pygame.K_w]:
        forward += 1.0
    if keys[pygame.K_s]:
        forward -= 1.0
    if keys[pygame.K_a]:
        turn += 1.0
    if keys[pygame.K_d]:
        turn -= 1.0
    return clamp(forward, -1.0, 1.0), clamp(turn, -1.0, 1.0)


def _joystick_rect() -> pygame.Rect:
    return pygame.Rect(SCENE_WIDTH + 70, 520, 180, 180)


def _scene_rect() -> pygame.Rect:
    return pygame.Rect(0, 0, SCENE_WIDTH, SCENE_HEIGHT)


def _update_analog_from_mouse(runtime: RuntimeState, mouse_pos: tuple[int, int]) -> None:
    rect = _joystick_rect()
    nx = clamp((mouse_pos[0] - rect.centerx) / (rect.width / 2.0), -1.0, 1.0)
    ny = clamp((rect.centery - mouse_pos[1]) / (rect.height / 2.0), -1.0, 1.0)
    runtime.analog_turn = -nx
    runtime.analog_forward = ny


def _start_camera_orbit(runtime: RuntimeState, mouse_pos: tuple[int, int]) -> None:
    """Start dragging the scene camera."""
    runtime.camera_orbit_active = True
    runtime.camera_last_mouse_pos = mouse_pos


def _update_camera_orbit(bundle: SimBundle, runtime: RuntimeState, mouse_pos: tuple[int, int]) -> None:
    """Orbit the render camera from a scene mouse drag."""
    if bundle.camera is None or runtime.camera_last_mouse_pos is None:
        runtime.camera_last_mouse_pos = mouse_pos
        return

    last_x, last_y = runtime.camera_last_mouse_pos
    dx = mouse_pos[0] - last_x
    dy = mouse_pos[1] - last_y
    bundle.camera.azimuth += dx * 0.35
    bundle.camera.elevation = clamp(bundle.camera.elevation + dy * 0.25, -80.0, -5.0)
    runtime.camera_last_mouse_pos = mouse_pos


def _zoom_camera(bundle: SimBundle, scroll_y: int) -> None:
    """Zoom the render camera with the mouse wheel."""
    if bundle.camera is None:
        return
    scale = 0.9 if scroll_y > 0 else 1.1
    bundle.camera.distance = clamp(bundle.camera.distance * scale, 1.2, 8.0)


def _motor_torque_constant(vehicle: VehicleConfig) -> float:
    """Return motor torque constant in N m / A from Kv in RPM/V."""
    return 60.0 / (2.0 * math.pi * vehicle.motor_kv)


def _wheel_peak_torque(vehicle: VehicleConfig) -> float:
    """Return the low-speed wheel torque limit per motor."""
    per_motor_supply_current = vehicle.supply_current_limit_total / 2.0
    estimated_phase_current = per_motor_supply_current * vehicle.phase_current_multiplier
    shaft_torque = _motor_torque_constant(vehicle) * estimated_phase_current
    return shaft_torque * vehicle.gear_ratio * vehicle.drivetrain_efficiency * vehicle.torque_power_scale


def _wheel_no_load_speed(vehicle: VehicleConfig) -> float:
    """Return the approximate no-load wheel angular speed in rad/s."""
    motor_no_load_rpm = vehicle.motor_kv * vehicle.supply_voltage
    motor_no_load_rads = motor_no_load_rpm * (2.0 * math.pi / 60.0)
    return motor_no_load_rads / vehicle.gear_ratio


def _available_wheel_torque(vehicle: VehicleConfig, wheel_speed: float) -> float:
    """Return the speed-limited available wheel torque per side."""
    base_torque = _wheel_peak_torque(vehicle)
    no_load_speed = max(_wheel_no_load_speed(vehicle), 1e-6)
    speed_fraction = min(abs(wheel_speed) / no_load_speed, 1.0)
    back_emf_limited = base_torque * (1.0 - speed_fraction)

    per_motor_power = (
        vehicle.supply_voltage
        * (vehicle.supply_current_limit_total / 2.0)
        * vehicle.drivetrain_efficiency
        * vehicle.torque_power_scale
    )
    if abs(wheel_speed) < 0.5:
        power_limited = base_torque
    else:
        power_limited = per_motor_power / abs(wheel_speed)

    return max(0.2, min(vehicle.max_wheel_torque, back_emf_limited, power_limited))


def _spin_torque_pair(
    vehicle: VehicleConfig,
    left_target_speed: float,
    right_target_speed: float,
    telemetry: Telemetry,
) -> tuple[float, float, float, float]:
    """Couple left/right torque for zero-radius turns.

    Spin mode is intentionally torque-commanded rather than wheel-speed
    controlled. The speed loop chatters badly during counter-rotation because
    the caster/contact model lets one wheel overshoot before the other.
    """
    left_sign = -1.0 if left_target_speed < 0.0 else 1.0
    right_sign = -1.0 if right_target_speed < 0.0 else 1.0
    shared_limit = min(vehicle.max_wheel_torque, _wheel_peak_torque(vehicle))
    target_speed = (abs(left_target_speed) + abs(right_target_speed)) / 2.0
    target_torque = vehicle.spin_torque_nm * min(1.0, target_speed / max(vehicle.max_wheel_speed, 1e-6))
    effort = clamp(target_torque, 0.0, shared_limit)
    return left_sign * effort, right_sign * effort, shared_limit, shared_limit


def _apply_drive_traction(
    bundle: SimBundle,
    vehicle: VehicleConfig,
    left_target_speed: float,
    right_target_speed: float,
    telemetry: Telemetry,
) -> tuple[float, float, float]:
    """Apply drive force through the planted rear tire patches."""
    normal_per_drive_wheel = vehicle.mass_total * 9.81 * vehicle.drive_normal_load_fraction / 2.0
    traction_limit = vehicle.drive_traction_friction * normal_per_drive_wheel
    force_gain = vehicle.mass_total * vehicle.drive_traction_force_scale * 5.0
    half_track = vehicle.wheel_separation / 2.0
    left_target_linear = left_target_speed * vehicle.wheel_radius
    right_target_linear = right_target_speed * vehicle.wheel_radius
    left_actual_linear = telemetry.forward_speed - telemetry.yaw_rate * half_track
    right_actual_linear = telemetry.forward_speed + telemetry.yaw_rate * half_track
    left_force_x = clamp((left_target_linear - left_actual_linear) * force_gain, -traction_limit, traction_limit)
    right_force_x = clamp((right_target_linear - right_actual_linear) * force_gain, -traction_limit, traction_limit)

    body_id = bundle.chassis_body_id
    body_pos = bundle.data.xpos[body_id]
    body_rot = bundle.data.xmat[body_id].reshape(3, 3)
    rear_x = -vehicle.wheelbase / 2.0
    contact_z = -(vehicle.frame_height / 2.0 + vehicle.frame_ground_clearance)

    for side_sign, force_x in ((1.0, left_force_x), (-1.0, right_force_x)):
        local_point = np.array([rear_x, side_sign * vehicle.wheel_separation / 2.0, contact_z], dtype=float)
        world_point = body_pos + body_rot @ local_point
        world_force = body_rot @ np.array([force_x, 0.0, 0.0], dtype=float)
        mujoco.mj_applyFT(
            bundle.model,
            bundle.data,
            world_force,
            np.zeros(3, dtype=float),
            world_point,
            body_id,
            bundle.data.qfrc_applied,
        )

    return left_force_x, right_force_x, traction_limit


def _apply_drive(bundle: SimBundle, runtime: RuntimeState, mixer: AssistedMixer, forward: float, turn: float) -> dict[str, float]:
    bundle.data.qfrc_applied[:] = 0.0
    telemetry = _body_telemetry(bundle, runtime.vehicle)
    command = make_command(forward, turn, runtime.vehicle)
    outputs = mixer.mix(runtime.mode, command, telemetry)

    left_target_speed = clamp(
        outputs.left_target_speed,
        -runtime.vehicle.max_wheel_speed,
        runtime.vehicle.max_wheel_speed,
    )
    right_target_speed = clamp(
        outputs.right_target_speed,
        -runtime.vehicle.max_wheel_speed,
        runtime.vehicle.max_wheel_speed,
    )

    is_spin = command.linear_mps == 0.0 and abs(command.angular_rps) > 0.0
    if is_spin:
        left_torque, right_torque, left_torque_limit, right_torque_limit = _spin_torque_pair(
            runtime.vehicle,
            left_target_speed,
            right_target_speed,
            telemetry,
        )
    else:
        left_error = left_target_speed - telemetry.left_wheel_speed
        right_error = right_target_speed - telemetry.right_wheel_speed
        left_target_linear = left_target_speed * runtime.vehicle.wheel_radius
        right_target_linear = right_target_speed * runtime.vehicle.wheel_radius

        left_slip_factor = max(
            0.45,
            1.0 - runtime.vehicle.rear_drive_left_slip * abs(left_target_linear - telemetry.forward_speed),
        )
        right_slip_factor = max(
            0.45,
            1.0 - runtime.vehicle.rear_drive_right_slip * abs(right_target_linear - telemetry.forward_speed),
        )

        left_torque = runtime.vehicle.rear_drive_left_gain * left_slip_factor * (
            runtime.vehicle.wheel_speed_kp * left_error - runtime.vehicle.wheel_speed_kd * telemetry.left_wheel_speed
        )
        right_torque = runtime.vehicle.rear_drive_right_gain * right_slip_factor * (
            runtime.vehicle.wheel_speed_kp * right_error - runtime.vehicle.wheel_speed_kd * telemetry.right_wheel_speed
        )

        left_torque_limit = _available_wheel_torque(runtime.vehicle, telemetry.left_wheel_speed)
        right_torque_limit = _available_wheel_torque(runtime.vehicle, telemetry.right_wheel_speed)

        left_torque = clamp(left_torque, -left_torque_limit, left_torque_limit)
        right_torque = clamp(right_torque, -right_torque_limit, right_torque_limit)

    left_traction_force, right_traction_force, traction_limit = _apply_drive_traction(
        bundle,
        runtime.vehicle,
        left_target_speed,
        right_target_speed,
        telemetry,
    )

    bundle.data.ctrl[bundle.left_actuator_id] = left_torque
    bundle.data.ctrl[bundle.right_actuator_id] = right_torque
    for steering_joint_id in bundle.steering_joint_ids:
        dof = bundle.model.jnt_dofadr[steering_joint_id]
        bundle.data.qfrc_applied[dof] = -runtime.vehicle.caster_damping * bundle.data.qvel[dof]

    return {
        "command_linear": command.linear_mps,
        "command_angular": command.angular_rps,
        "assist_angular": outputs.angular_correction,
        "heading_error": outputs.heading_error,
        "left_target_speed": left_target_speed,
        "right_target_speed": right_target_speed,
        "left_torque": left_torque,
        "right_torque": right_torque,
        "left_torque_limit": left_torque_limit,
        "right_torque_limit": right_torque_limit,
        "left_traction_force": left_traction_force,
        "right_traction_force": right_traction_force,
        "traction_force_limit": traction_limit,
    }


def _render_scene(bundle: SimBundle) -> pygame.Surface:
    assert bundle.renderer is not None
    assert bundle.camera is not None
    bundle.renderer.update_scene(bundle.data, camera=bundle.camera)
    pixels = bundle.renderer.render()
    return pygame.image.frombuffer(pixels.tobytes(), (SCENE_WIDTH, SCENE_HEIGHT), "RGB")


def _draw_overlay(
    screen: pygame.Surface,
    runtime: RuntimeState,
    scenario: ScenarioConfig,
    telemetry: Telemetry,
    drive_debug: dict[str, float],
    font: pygame.font.Font,
    small_font: pygame.font.Font,
) -> None:
    panel_rect = pygame.Rect(SCENE_WIDTH, 0, PANEL_WIDTH, WINDOW_HEIGHT)
    pygame.draw.rect(screen, (19, 23, 29), panel_rect)
    pygame.draw.line(screen, (54, 63, 77), (SCENE_WIDTH, 0), (SCENE_WIDTH, WINDOW_HEIGHT), 1)

    lines = [
        f"Scenario: {scenario.name}",
        f"Mode: {runtime.mode}",
        f"Input: fwd={drive_debug['input_forward']:+.2f} turn={drive_debug['input_turn']:+.2f}",
        f"Pose: x={telemetry.x:+.2f} y={telemetry.y:+.2f}",
        f"Attitude: roll={math.degrees(telemetry.roll):+.1f} pitch={math.degrees(telemetry.pitch):+.1f} yaw={math.degrees(telemetry.yaw):+.1f}",
        f"Speeds: fwd={telemetry.forward_speed:+.2f} lat={telemetry.lateral_speed:+.2f} yawdot={telemetry.yaw_rate:+.2f}",
        f"Cmd: v={drive_debug['command_linear']:+.2f} w={drive_debug['command_angular']:+.2f}",
        f"Assist: {drive_debug['assist_angular']:+.2f} rad/s",
        f"Heading err: {drive_debug['heading_error']:+.2f} rad",
        f"Wheel tgt: L={drive_debug['left_target_speed']:+.2f} R={drive_debug['right_target_speed']:+.2f}",
        f"Wheel tq: L={drive_debug['left_torque']:+.1f} R={drive_debug['right_torque']:+.1f}",
        f"Rollover margin: {drive_debug['rollover_margin_deg']:.1f} deg",
        "",
        "Controls:",
        "W/A/S/D drive",
        "Mouse drag joystick",
        "Drag scene to orbit",
        "Mouse wheel zoom",
        "Tab raw/assisted",
        "1-6 scenario",
        "R reset",
        ", / . select tuning",
        "[ / ] adjust tuning",
    ]
    y = 24
    for line in lines:
        if line == "":
            y += 10
            continue
        rendered = font.render(line, True, (226, 232, 240))
        screen.blit(rendered, (SCENE_WIDTH + 18, y))
        y += 22

    tunable_name, tunable_step = TUNABLES[runtime.selected_tunable]
    tuning_text = small_font.render(
        f"Tuning: {tunable_name} = {getattr(runtime.vehicle, tunable_name):.3f} (step {tunable_step})",
        True,
        (125, 211, 252),
    )
    screen.blit(tuning_text, (SCENE_WIDTH + 18, 470))

    rect = _joystick_rect()
    pygame.draw.rect(screen, (15, 18, 24), rect, border_radius=12)
    pygame.draw.rect(screen, (70, 84, 102), rect, width=1, border_radius=12)
    pygame.draw.line(screen, (52, 61, 74), (rect.centerx, rect.top + 8), (rect.centerx, rect.bottom - 8))
    pygame.draw.line(screen, (52, 61, 74), (rect.left + 8, rect.centery), (rect.right - 8, rect.centery))
    dot_x = rect.centerx - runtime.analog_turn * (rect.width / 2.0 - 10)
    dot_y = rect.centery - runtime.analog_forward * (rect.height / 2.0 - 10)
    pygame.draw.circle(screen, (34, 197, 94), (int(dot_x), int(dot_y)), 11)
    screen.blit(small_font.render("FWD", True, (148, 163, 184)), (rect.centerx - 14, rect.top + 6))
    screen.blit(small_font.render("REV", True, (148, 163, 184)), (rect.centerx - 14, rect.bottom - 20))


def _rollover_margin(vehicle: VehicleConfig, telemetry: Telemetry) -> float:
    return max(0.0, vehicle.rollover_roll_limit_deg - abs(math.degrees(telemetry.roll)))


def _reload_bundle(runtime: RuntimeState, enable_rendering: bool) -> tuple[ScenarioConfig, SimBundle, AssistedMixer]:
    scenario = load_scenario(runtime.scenario_name)
    bundle = _build_bundle(runtime.vehicle, scenario, enable_rendering=enable_rendering)
    mixer = AssistedMixer(runtime.vehicle)
    _reset_bundle(bundle)
    return scenario, bundle, mixer


def run_app(args: argparse.Namespace) -> Path:
    """Run the simulator and return the written log path."""
    scenario_names = available_scenarios()
    scenario_name = args.scenario or scenario_names[0]
    if scenario_name not in scenario_names:
        raise ValueError(f"Unknown scenario {scenario_name!r}; choose from {scenario_names}")

    base_vehicle = load_vehicle(args.vehicle)
    runtime = RuntimeState(
        scenario_name=scenario_name,
        mode=args.mode,
        vehicle=_vehicle_copy(base_vehicle),
    )

    enable_rendering = not args.headless
    if enable_rendering:
        pygame.init()
        pygame.display.set_caption("Couch MuJoCo Sandbox")
        screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        font = pygame.font.SysFont("Menlo", 18)
        small_font = pygame.font.SysFont("Menlo", 14)
        clock = pygame.time.Clock()
    else:
        screen = None
        font = None
        small_font = None
        clock = None

    scenario, bundle, mixer = _reload_bundle(runtime, enable_rendering=enable_rendering)
    logger = RunLogger(
        metadata={
            "vehicle": runtime.vehicle.name,
            "scenario": scenario.name,
            "mode": runtime.mode,
            "generated_at": time.time(),
        }
    )

    running = True
    step_count = 0
    pending_reset = False

    while running:
        if enable_rendering:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_TAB:
                        runtime.mode = "assisted" if runtime.mode == "raw" else "raw"
                        mixer.reset()
                        logger.metadata["mode"] = runtime.mode
                    elif event.key == pygame.K_SPACE:
                        runtime.analog_active = False
                        runtime.camera_orbit_active = False
                        runtime.camera_last_mouse_pos = None
                        runtime.analog_forward = 0.0
                        runtime.analog_turn = 0.0
                    elif event.key == pygame.K_r:
                        pending_reset = True
                    elif event.key in SCENARIO_HOTKEYS:
                        runtime.scenario_name = SCENARIO_HOTKEYS[event.key]
                        scenario, bundle, mixer = _reload_bundle(runtime, enable_rendering=True)
                        logger.metadata["scenario"] = scenario.name
                    elif event.key == pygame.K_COMMA:
                        runtime.selected_tunable = (runtime.selected_tunable - 1) % len(TUNABLES)
                    elif event.key == pygame.K_PERIOD:
                        runtime.selected_tunable = (runtime.selected_tunable + 1) % len(TUNABLES)
                    elif event.key in (pygame.K_LEFTBRACKET, pygame.K_RIGHTBRACKET):
                        field_name, step = TUNABLES[runtime.selected_tunable]
                        direction = -1.0 if event.key == pygame.K_LEFTBRACKET else 1.0
                        setattr(runtime.vehicle, field_name, getattr(runtime.vehicle, field_name) + direction * step)
                        scenario, bundle, mixer = _reload_bundle(runtime, enable_rendering=True)
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and _joystick_rect().collidepoint(event.pos):
                    runtime.analog_active = True
                    runtime.camera_orbit_active = False
                    runtime.camera_last_mouse_pos = None
                    _update_analog_from_mouse(runtime, event.pos)
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and _scene_rect().collidepoint(event.pos):
                    runtime.analog_active = False
                    runtime.analog_forward = 0.0
                    runtime.analog_turn = 0.0
                    _start_camera_orbit(runtime, event.pos)
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    runtime.analog_active = False
                    runtime.camera_orbit_active = False
                    runtime.camera_last_mouse_pos = None
                    runtime.analog_forward = 0.0
                    runtime.analog_turn = 0.0
                elif event.type == pygame.MOUSEWHEEL:
                    _zoom_camera(bundle, event.y)
                elif event.type == pygame.MOUSEMOTION:
                    if runtime.camera_orbit_active:
                        _update_camera_orbit(bundle, runtime, event.pos)
                    elif runtime.analog_active:
                        _update_analog_from_mouse(runtime, event.pos)

        if pending_reset:
            _reset_bundle(bundle)
            mixer.reset()
            pending_reset = False

        if runtime.analog_active:
            forward, turn = runtime.analog_forward, runtime.analog_turn
        elif enable_rendering:
            forward, turn = _input_from_keyboard(pygame.key.get_pressed())
        else:
            forward, turn = 0.0, 0.0

        drive_debug = _apply_drive(bundle, runtime, mixer, forward, turn)
        drive_debug["input_forward"] = forward
        drive_debug["input_turn"] = turn
        mujoco.mj_step(bundle.model, bundle.data)
        telemetry = _body_telemetry(bundle, runtime.vehicle)
        drive_debug["rollover_margin_deg"] = _rollover_margin(runtime.vehicle, telemetry)

        logger.add_sample(
            {
                "time": float(bundle.data.time),
                "x": telemetry.x,
                "y": telemetry.y,
                "yaw": telemetry.yaw,
                "pitch": telemetry.pitch,
                "roll": telemetry.roll,
                "forward_speed": telemetry.forward_speed,
                "lateral_speed": telemetry.lateral_speed,
                "yaw_rate": telemetry.yaw_rate,
                "input_forward": forward,
                "input_turn": turn,
                "command_linear": drive_debug["command_linear"],
                "command_angular": drive_debug["command_angular"],
                "assist_angular": drive_debug["assist_angular"],
                "heading_error": drive_debug["heading_error"],
                "left_target_speed": drive_debug["left_target_speed"],
                "right_target_speed": drive_debug["right_target_speed"],
                "left_torque": drive_debug["left_torque"],
                "right_torque": drive_debug["right_torque"],
                "rollover_margin_deg": drive_debug["rollover_margin_deg"],
            }
        )

        step_count += 1
        if args.headless and step_count >= args.steps:
            running = False

        if enable_rendering and screen is not None and font is not None and small_font is not None and clock is not None:
            scene_surface = _render_scene(bundle)
            screen.blit(scene_surface, (0, 0))
            _draw_overlay(screen, runtime, scenario, telemetry, drive_debug, font, small_font)
            pygame.display.flip()
            clock.tick(RENDER_FPS)

    if enable_rendering:
        pygame.quit()

    return logger.write(args.log_path)


def main() -> None:
    """CLI entrypoint for the simulator."""
    parser = argparse.ArgumentParser(description="Standalone MuJoCo couch teleop sandbox")
    parser.add_argument("--scenario", help="Scenario name from sim/config/scenarios")
    parser.add_argument("--vehicle", type=Path, help="Vehicle YAML override")
    parser.add_argument("--mode", choices=["raw", "assisted"], default="raw")
    parser.add_argument("--headless", action="store_true", help="Run physics only without opening a window")
    parser.add_argument("--steps", type=int, default=900, help="Headless simulation step count")
    parser.add_argument("--log-path", type=Path, help="Explicit JSON output path")
    args = parser.parse_args()

    log_path = run_app(args)
    print(log_path)


if __name__ == "__main__":
    main()
