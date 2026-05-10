"""Configuration loading for the couch simulator."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(slots=True)
class VehicleConfig:
    """Tunable physical parameters for the couch vehicle."""

    name: str
    mass_total: float
    frame_mass: float
    rider_mass: float
    rider_radius_x: float
    rider_radius_y: float
    rider_radius_z: float
    com_offset_x: float
    com_offset_y: float
    com_height: float
    chassis_length: float
    chassis_width: float
    frame_height: float
    frame_ground_clearance: float
    initial_spawn_clearance: float
    backrest_height: float
    seat_thickness: float
    wheel_radius: float
    wheel_width: float
    wheel_separation: float
    wheelbase: float
    caster_radius: float
    caster_width: float
    caster_separation: float
    caster_fork_drop: float
    front_frame_ground_clearance: float
    motor_kv: float
    supply_voltage: float
    supply_current_limit_total: float
    gear_ratio: float
    drivetrain_efficiency: float
    phase_current_multiplier: float
    torque_power_scale: float
    rear_drive_left_gain: float
    rear_drive_right_gain: float
    rear_drive_left_slip: float
    rear_drive_right_slip: float
    drive_traction_friction: float
    drive_normal_load_fraction: float
    drive_traction_force_scale: float
    max_wheel_torque: float
    max_wheel_speed: float
    wheel_speed_kp: float
    wheel_speed_kd: float
    spin_torque_nm: float
    max_linear_speed: float
    max_angular_speed: float
    spin_in_place_turn_threshold: float
    spin_in_place_angular_scale: float
    frame_flex_stiffness: float
    frame_flex_damping: float
    frame_flex_range_deg: float
    caster_trail: float
    caster_damping: float
    caster_friction: float
    caster_contact_radius_extra: float
    caster_contact_margin: float
    caster_contact_solref_time: float
    caster_contact_solref_damping: float
    caster_contact_solimp_min: float
    caster_contact_solimp_max: float
    caster_contact_solimp_width: float
    rollover_roll_limit_deg: float
    assist_heading_gain: float
    assist_lateral_gain: float
    assist_yaw_rate_gain: float
    assist_turn_deadband: float
    assist_activation_speed: float
    camera_distance: float
    camera_azimuth_deg: float
    camera_elevation_deg: float

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable view."""
        return asdict(self)


@dataclass(slots=True)
class TerrainPlaneConfig:
    """One explicit terrain slab in a scenario."""

    name: str
    pos: list[float]
    size: list[float]
    slope_pitch_deg: float
    slope_roll_deg: float
    yaw_deg: float
    friction: list[float]
    rgba: str | None = None

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable view."""
        return asdict(self)


@dataclass(slots=True)
class ScenarioConfig:
    """Terrain and environment settings for a simulation run."""

    name: str
    description: str
    slope_pitch_deg: float
    slope_roll_deg: float
    ground_friction: list[float]
    left_ground_friction: list[float] | None
    right_ground_friction: list[float] | None
    bump_height: float
    bump_spacing: float
    bump_count: int
    bump_width: float
    bump_length: float
    surface_length: float
    surface_width: float
    terrain_planes: list[TerrainPlaneConfig] | None = None

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable view."""
        return asdict(self)


def simulator_root() -> Path:
    """Return the `sim/` project root."""
    return Path(__file__).resolve().parents[2]


def _read_yaml(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping in {path}")
    return data


def load_vehicle(path: Path | None = None) -> VehicleConfig:
    """Load a vehicle profile from YAML."""
    vehicle_path = path or simulator_root() / "config/vehicle/default.yaml"
    return VehicleConfig(**_read_yaml(vehicle_path))


def load_scenario(name: str, path: Path | None = None) -> ScenarioConfig:
    """Load a named scenario or an explicit YAML path."""
    scenario_path = path or simulator_root() / "config/scenarios" / f"{name}.yaml"
    data = _read_yaml(scenario_path)
    terrain_planes = data.get("terrain_planes")
    if terrain_planes is not None:
        if not isinstance(terrain_planes, list):
            raise ValueError(f"Expected terrain_planes list in {scenario_path}")
        data["terrain_planes"] = [TerrainPlaneConfig(**plane) for plane in terrain_planes]
    return ScenarioConfig(**data)


def available_scenarios() -> list[str]:
    """List scenario names from the config directory."""
    scenario_dir = simulator_root() / "config/scenarios"
    return sorted(item.stem for item in scenario_dir.glob("*.yaml"))
