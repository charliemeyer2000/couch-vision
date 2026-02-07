"""Helpers for loading fixed frame relationships from the phone mount URDF."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

_DEFAULT_URDF = Path(__file__).resolve().parent.parent.parent / "config" / "phone_mount.urdf"


@dataclass(frozen=True)
class MountFrameModel:
    """Resolved transform relationship between base_link and imu link."""

    urdf_path: Path
    translation_base_from_imu_m: np.ndarray
    rotation_base_from_imu: np.ndarray
    base_forward_axis_in_imu: np.ndarray
    base_up_axis_in_imu: np.ndarray


def _parse_origin(joint: ET.Element) -> tuple[np.ndarray, np.ndarray]:
    origin = joint.find("origin")
    if origin is None:
        return np.zeros(3, dtype=np.float64), np.eye(3, dtype=np.float64)

    xyz_str = origin.attrib.get("xyz", "0 0 0")
    rpy_str = origin.attrib.get("rpy", "0 0 0")
    xyz = np.array([float(v) for v in xyz_str.split()], dtype=np.float64)
    rpy = np.array([float(v) for v in rpy_str.split()], dtype=np.float64)
    rot = Rotation.from_euler("xyz", rpy).as_matrix()
    return xyz, rot


def load_mount_frame_model(
    urdf_path: str | Path | None = None,
    base_link: str = "base_link",
    imu_link: str = "imu",
) -> MountFrameModel:
    """Load the mount frame model from URDF and resolve base<-imu transform."""
    path = Path(urdf_path) if urdf_path is not None else _DEFAULT_URDF
    root = ET.parse(path).getroot()

    # child -> (parent, translation parent->child, rotation parent->child)
    child_to_joint: dict[str, tuple[str, np.ndarray, np.ndarray]] = {}
    for joint in root.findall("joint"):
        parent_el = joint.find("parent")
        child_el = joint.find("child")
        if parent_el is None or child_el is None:
            continue
        parent = parent_el.attrib["link"]
        child = child_el.attrib["link"]
        t_parent_child, r_parent_child = _parse_origin(joint)
        child_to_joint[child] = (parent, t_parent_child, r_parent_child)

    # Build chain by walking imu -> ... -> base (child->parent lookup), then
    # compose parent_from_child transforms in forward order (base -> imu).
    chain: list[tuple[np.ndarray, np.ndarray]] = []
    curr = imu_link
    visited: set[str] = set()
    while curr != base_link:
        if curr in visited:
            raise ValueError(f"Cycle detected while resolving {imu_link}->{base_link}")
        visited.add(curr)
        if curr not in child_to_joint:
            raise ValueError(f"No joint chain from {imu_link} to {base_link} in {path}")

        parent, t_parent_child, r_parent_child = child_to_joint[curr]
        chain.append((t_parent_child, r_parent_child))
        curr = parent

    t_base_from_imu = np.zeros(3, dtype=np.float64)
    r_base_from_imu = np.eye(3, dtype=np.float64)
    for t_parent_child, r_parent_child in reversed(chain):
        t_base_from_imu = t_base_from_imu + r_base_from_imu @ t_parent_child
        r_base_from_imu = r_base_from_imu @ r_parent_child

    base_forward = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    base_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    r_imu_from_base = r_base_from_imu.T

    return MountFrameModel(
        urdf_path=path,
        translation_base_from_imu_m=t_base_from_imu,
        rotation_base_from_imu=r_base_from_imu,
        base_forward_axis_in_imu=r_imu_from_base @ base_forward,
        base_up_axis_in_imu=r_imu_from_base @ base_up,
    )
