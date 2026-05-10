"""MJCF generation for the couch simulator."""

from __future__ import annotations

import math
from dataclasses import dataclass
from xml.etree import ElementTree as ET

from .config import ScenarioConfig, VehicleConfig

DEFAULT_OFFSCREEN_WIDTH = 1600
DEFAULT_OFFSCREEN_HEIGHT = 1200


def _euler_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float = 0.0) -> tuple[float, float, float, float]:
    """Convert XYZ Euler degrees to a quaternion tuple."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, y, z


@dataclass(slots=True)
class ModelHandles:
    """Names used by the runtime to locate joints and bodies."""

    chassis_body: str = "chassis"
    left_wheel_joint: str = "rear_left_wheel"
    right_wheel_joint: str = "rear_right_wheel"
    contact_geoms: tuple[str, ...] = (
        "rear_left_wheel_collision",
        "rear_right_wheel_collision",
        "front_left_caster_collision",
        "front_right_caster_collision",
    )
    steering_joints: tuple[str, str] = ("front_left_steer", "front_right_steer")


def _fmt_triplet(values: tuple[float, float, float]) -> str:
    return " ".join(f"{value:.6f}" for value in values)


def _fmt_quat(values: tuple[float, float, float, float]) -> str:
    return " ".join(f"{value:.6f}" for value in values)


def _box_inertia(mass: float, half_x: float, half_y: float, half_z: float) -> tuple[float, float, float]:
    """Return box inertia around local x/y/z axes for MuJoCo diaginertia."""
    full_x = 2.0 * half_x
    full_y = 2.0 * half_y
    full_z = 2.0 * half_z
    ixx = mass * (full_y * full_y + full_z * full_z) / 12.0
    iyy = mass * (full_x * full_x + full_z * full_z) / 12.0
    izz = mass * (full_x * full_x + full_y * full_y) / 12.0
    return ixx, iyy, izz


def build_mjcf(vehicle: VehicleConfig, scenario: ScenarioConfig) -> str:
    """Generate an MJCF string for the requested vehicle and terrain."""
    frame_half = vehicle.frame_height / 2.0
    ride_chassis_z = frame_half + vehicle.frame_ground_clearance
    spawn_chassis_z = ride_chassis_z + vehicle.initial_spawn_clearance
    rear_x = -vehicle.wheelbase / 2.0
    front_x = vehicle.wheelbase / 2.0
    wheel_y = vehicle.wheel_separation / 2.0
    caster_y = vehicle.caster_separation / 2.0
    ground_half_z = 0.1
    slope_quat = _euler_to_quat(scenario.slope_roll_deg, scenario.slope_pitch_deg)
    wheel_quat = (0.707107, 0.707107, 0.0, 0.0)
    front_caster_center_local_z = vehicle.caster_radius - (
        vehicle.front_frame_ground_clearance + frame_half
    )
    flex_range = f"{-vehicle.frame_flex_range_deg:.6f} {vehicle.frame_flex_range_deg:.6f}"
    chassis_inertia = _box_inertia(
        vehicle.frame_mass,
        vehicle.chassis_length / 2.0,
        vehicle.chassis_width / 2.0,
        frame_half,
    )
    rider_inertia = _box_inertia(
        vehicle.rider_mass,
        vehicle.rider_radius_x,
        vehicle.rider_radius_y,
        vehicle.rider_radius_z,
    )

    root = ET.Element("mujoco", model="couch_sim")
    ET.SubElement(root, "compiler", angle="degree", coordinate="local")
    ET.SubElement(
        root,
        "option",
        timestep="0.005",
        gravity="0 0 -9.81",
        iterations="120",
        ls_iterations="40",
        integrator="implicitfast",
        viscosity="0",
        cone="elliptic",
    )
    ET.SubElement(
        root,
        "size",
        nconmax="256",
        njmax="512",
    )
    visual = ET.SubElement(root, "visual")
    ET.SubElement(
        visual,
        "global",
        offwidth=str(DEFAULT_OFFSCREEN_WIDTH),
        offheight=str(DEFAULT_OFFSCREEN_HEIGHT),
    )

    default = ET.SubElement(root, "default")
    ET.SubElement(default, "joint", damping="0.2", armature="0.01")
    ET.SubElement(default, "geom", condim="6", solref="0.006 1", solimp="0.92 0.99 0.002")
    ET.SubElement(default, "motor", ctrllimited="true")

    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker", rgb1="0.35 0.37 0.4", rgb2="0.18 0.2 0.23", width="512", height="512")
    ET.SubElement(asset, "material", name="ground_mat", texture="grid", texrepeat="6 6", reflectance="0.04")
    ET.SubElement(asset, "material", name="frame_mat", rgba="0.42 0.3 0.18 1")
    ET.SubElement(asset, "material", name="cushion_mat", rgba="0.1 0.27 0.52 1")
    ET.SubElement(asset, "material", name="wheel_mat", rgba="0.08 0.08 0.08 1")
    ET.SubElement(asset, "material", name="caster_mat", rgba="0.2 0.2 0.22 1")

    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(worldbody, "light", pos="0 0 6", dir="0 0 -1", diffuse="1 1 1", specular="0.2 0.2 0.2")

    if scenario.terrain_planes:
        for plane in scenario.terrain_planes:
            plane_quat = _euler_to_quat(plane.slope_roll_deg, plane.slope_pitch_deg, plane.yaw_deg)
            terrain = ET.SubElement(
                worldbody,
                "geom",
                name=plane.name,
                type="box",
                pos=_fmt_triplet(tuple(plane.pos)),
                size=_fmt_triplet(tuple(plane.size)),
                quat=_fmt_quat(plane_quat),
                material="ground_mat",
                friction=_fmt_triplet(tuple(plane.friction)),
            )
            if plane.rgba:
                terrain.set("rgba", plane.rgba)
    elif scenario.left_ground_friction and scenario.right_ground_friction:
        left_ground = ET.SubElement(
            worldbody,
            "geom",
            name="ground_left",
            type="box",
            pos=_fmt_triplet((0.0, scenario.surface_width / 4.0, -ground_half_z)),
            size=_fmt_triplet((scenario.surface_length / 2.0, scenario.surface_width / 4.0, ground_half_z)),
            quat=_fmt_quat(slope_quat),
            material="ground_mat",
            friction=_fmt_triplet(tuple(scenario.left_ground_friction)),
        )
        right_ground = ET.SubElement(
            worldbody,
            "geom",
            name="ground_right",
            type="box",
            pos=_fmt_triplet((0.0, -scenario.surface_width / 4.0, -ground_half_z)),
            size=_fmt_triplet((scenario.surface_length / 2.0, scenario.surface_width / 4.0, ground_half_z)),
            quat=_fmt_quat(slope_quat),
            material="ground_mat",
            friction=_fmt_triplet(tuple(scenario.right_ground_friction)),
        )
        left_ground.set("rgba", "0.55 0.56 0.58 1")
        right_ground.set("rgba", "0.32 0.34 0.36 1")
    else:
        ET.SubElement(
            worldbody,
            "geom",
            name="ground",
            type="box",
            pos=_fmt_triplet((0.0, 0.0, -ground_half_z)),
            size=_fmt_triplet((scenario.surface_length / 2.0, scenario.surface_width / 2.0, ground_half_z)),
            quat=_fmt_quat(slope_quat),
            material="ground_mat",
            friction=_fmt_triplet(tuple(scenario.ground_friction)),
        )

    if scenario.bump_count > 0 and scenario.bump_height > 0.0:
        start_x = -scenario.surface_length / 3.0
        for idx in range(scenario.bump_count):
            bump_x = start_x + idx * scenario.bump_spacing
            ET.SubElement(
                worldbody,
                "geom",
                name=f"bump_{idx}",
                type="box",
                pos=_fmt_triplet((bump_x, 0.0, scenario.bump_height / 2.0 - 0.005)),
                size=_fmt_triplet((scenario.bump_width / 2.0, scenario.bump_length / 2.0, scenario.bump_height / 2.0)),
                quat=_fmt_quat(slope_quat),
                friction=_fmt_triplet(tuple(scenario.ground_friction)),
                rgba="0.46 0.42 0.4 1",
            )

    chassis = ET.SubElement(worldbody, "body", name="chassis", pos=_fmt_triplet((0.0, 0.0, spawn_chassis_z)))
    ET.SubElement(chassis, "freejoint", name="couch_free")
    ET.SubElement(
        chassis,
        "inertial",
        pos="0 0 0",
        mass=f"{vehicle.frame_mass:.6f}",
        diaginertia=_fmt_triplet(chassis_inertia),
    )
    ET.SubElement(
        chassis,
        "geom",
        name="frame",
        type="box",
        pos=_fmt_triplet((0.0, 0.0, 0.0)),
        size=_fmt_triplet((vehicle.chassis_length / 2.0, vehicle.chassis_width / 2.0, frame_half)),
        material="frame_mat",
        contype="0",
        conaffinity="0",
    )
    front_frame = ET.SubElement(
        chassis,
        "body",
        name="front_frame_flex",
        pos=_fmt_triplet((front_x, 0.0, 0.0)),
    )
    ET.SubElement(
        front_frame,
        "joint",
        name="frame_flex_pitch",
        type="hinge",
        axis="0 1 0",
        limited="true",
        range=flex_range,
        stiffness=f"{vehicle.frame_flex_stiffness:.6f}",
        damping=f"{vehicle.frame_flex_damping:.6f}",
    )
    ET.SubElement(
        front_frame,
        "joint",
        name="frame_flex_roll",
        type="hinge",
        axis="1 0 0",
        limited="true",
        range=flex_range,
        stiffness=f"{vehicle.frame_flex_stiffness:.6f}",
        damping=f"{vehicle.frame_flex_damping:.6f}",
    )
    ET.SubElement(
        front_frame,
        "geom",
        name="front_crossmember",
        type="box",
        pos="0 0 0",
        size=_fmt_triplet((0.06, vehicle.chassis_width / 2.0, frame_half * 0.85)),
        material="frame_mat",
        density="260",
        contype="0",
        conaffinity="0",
    )
    ET.SubElement(
        chassis,
        "geom",
        name="seat",
        type="box",
        pos=_fmt_triplet((0.08, 0.0, frame_half + vehicle.seat_thickness / 2.0)),
        size=_fmt_triplet((vehicle.chassis_length * 0.44, vehicle.chassis_width * 0.48, vehicle.seat_thickness / 2.0)),
        material="cushion_mat",
        density="85",
        contype="0",
        conaffinity="0",
    )
    rider = ET.SubElement(
        chassis,
        "body",
        name="central_rider_mass",
        pos=_fmt_triplet((vehicle.com_offset_x, vehicle.com_offset_y, vehicle.com_height - ride_chassis_z)),
    )
    ET.SubElement(
        rider,
        "inertial",
        pos="0 0 0",
        mass=f"{vehicle.rider_mass:.6f}",
        diaginertia=_fmt_triplet(rider_inertia),
    )
    ET.SubElement(
        rider,
        "geom",
        name="rider_mass_visual",
        type="ellipsoid",
        pos="0 0 0",
        size=_fmt_triplet((vehicle.rider_radius_x, vehicle.rider_radius_y, vehicle.rider_radius_z)),
        rgba="0.18 0.33 0.56 0.28",
        contype="0",
        conaffinity="0",
    )
    ET.SubElement(
        chassis,
        "geom",
        name="backrest",
        type="box",
        pos=_fmt_triplet((-vehicle.chassis_length * 0.3, 0.0, frame_half + vehicle.backrest_height / 2.0)),
        size=_fmt_triplet((vehicle.chassis_length * 0.08, vehicle.chassis_width * 0.48, vehicle.backrest_height / 2.0)),
        material="cushion_mat",
        density="70",
        contype="0",
        conaffinity="0",
    )
    ET.SubElement(chassis, "site", name="camera_target", pos=_fmt_triplet((0.0, 0.0, 0.45)), size="0.02")

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        ET.SubElement(
            chassis,
            "geom",
            name=f"rear_{side_name}_tire_patch",
            type="capsule",
            fromto=(
                f"{rear_x:.6f} {side_sign * (wheel_y - vehicle.wheel_width / 2.0):.6f} "
                f"{-(ride_chassis_z - 0.018):.6f} "
                f"{rear_x:.6f} {side_sign * (wheel_y + vehicle.wheel_width / 2.0):.6f} "
                f"{-(ride_chassis_z - 0.018):.6f}"
            ),
            size="0.030000",
            rgba="0 0 0 0",
            friction="0.03 0.004 0.001",
            solref="0.018 1.5",
            solimp="0.70 0.96 0.020",
            margin="0.080",
            contype="0",
            conaffinity="0",
        )
        wheel_body = ET.SubElement(
            chassis,
            "body",
            name=f"rear_{side_name}_wheel_body",
            pos=_fmt_triplet((rear_x, side_sign * wheel_y, vehicle.wheel_radius - ride_chassis_z)),
        )
        ET.SubElement(
            wheel_body,
            "inertial",
            pos="0 0 0",
            mass="1.400000",
            diaginertia="0.003600 0.006600 0.003600",
        )
        ET.SubElement(
            wheel_body,
            "joint",
            name=f"rear_{side_name}_wheel",
            type="hinge",
            axis="0 1 0",
            damping="0.12",
            armature="0.015",
        )
        ET.SubElement(
            wheel_body,
            "geom",
            name=f"rear_{side_name}_wheel_geom",
            type="cylinder",
            size=f"{vehicle.wheel_radius:.6f} {vehicle.wheel_width / 2.0:.6f}",
            quat=_fmt_quat(wheel_quat),
            material="wheel_mat",
            contype="0",
            conaffinity="0",
        )
        ET.SubElement(
            wheel_body,
            "geom",
            name=f"rear_{side_name}_wheel_collision",
            type="capsule",
            fromto=f"0 {-vehicle.wheel_width / 2.0:.6f} 0 0 {vehicle.wheel_width / 2.0:.6f} 0",
            size=f"{vehicle.wheel_radius + 0.010000:.6f}",
            rgba="0 0 0 0",
            friction="1.35 0.12 0.01",
            solref="0.014 1.4",
            solimp="0.76 0.97 0.012",
            margin="0.020",
        )

        caster_body = ET.SubElement(
            front_frame,
            "body",
            name=f"front_{side_name}_caster_body",
            pos=_fmt_triplet(
                (
                    0.0,
                    side_sign * caster_y,
                    front_caster_center_local_z + vehicle.caster_fork_drop,
                )
            ),
        )
        ET.SubElement(
            caster_body,
            "joint",
            name=f"front_{side_name}_steer",
            type="hinge",
            axis="0 0 1",
            damping=f"{vehicle.caster_damping:.6f}",
            frictionloss=f"{vehicle.caster_friction:.6f}",
        )
        ET.SubElement(
            caster_body,
            "geom",
            name=f"front_{side_name}_fork",
            type="capsule",
            fromto=f"0 0 0 0 0 {-vehicle.caster_fork_drop:.6f}",
            size="0.015",
            material="caster_mat",
            density="400",
            contype="0",
            conaffinity="0",
        )
        caster_wheel = ET.SubElement(
            caster_body,
            "body",
            name=f"front_{side_name}_caster_wheel_body",
            pos=_fmt_triplet((-vehicle.caster_trail, 0.0, -vehicle.caster_fork_drop)),
        )
        ET.SubElement(
            caster_wheel,
            "inertial",
            pos="0 0 0",
            mass="0.500000",
            diaginertia="0.000450 0.000750 0.000450",
        )
        ET.SubElement(
            caster_wheel,
            "joint",
            name=f"front_{side_name}_caster_spin",
            type="hinge",
            axis="0 1 0",
            damping="0.08",
            armature="0.004",
        )
        ET.SubElement(
            caster_wheel,
            "geom",
            name=f"front_{side_name}_caster_wheel_geom",
            type="cylinder",
            size=f"{vehicle.caster_radius:.6f} {vehicle.caster_width / 2.0:.6f}",
            quat=_fmt_quat(wheel_quat),
            material="caster_mat",
            contype="0",
            conaffinity="0",
        )
        ET.SubElement(
            caster_wheel,
            "geom",
            name=f"front_{side_name}_caster_collision",
            type="sphere",
            size=f"{vehicle.caster_radius + vehicle.caster_contact_radius_extra:.6f}",
            rgba="0 0 0 0",
            friction="0.08 0.01 0.001",
            solref=f"{vehicle.caster_contact_solref_time:.6f} {vehicle.caster_contact_solref_damping:.6f}",
            solimp=(
                f"{vehicle.caster_contact_solimp_min:.6f} "
                f"{vehicle.caster_contact_solimp_max:.6f} "
                f"{vehicle.caster_contact_solimp_width:.6f}"
            ),
            margin=f"{vehicle.caster_contact_margin:.6f}",
        )

    actuator = ET.SubElement(root, "actuator")
    ET.SubElement(
        actuator,
        "motor",
        name="rear_left_motor",
        joint="rear_left_wheel",
        ctrlrange=f"{-vehicle.max_wheel_torque:.6f} {vehicle.max_wheel_torque:.6f}",
    )
    ET.SubElement(
        actuator,
        "motor",
        name="rear_right_motor",
        joint="rear_right_wheel",
        ctrlrange=f"{-vehicle.max_wheel_torque:.6f} {vehicle.max_wheel_torque:.6f}",
    )

    return ET.tostring(root, encoding="unicode")
