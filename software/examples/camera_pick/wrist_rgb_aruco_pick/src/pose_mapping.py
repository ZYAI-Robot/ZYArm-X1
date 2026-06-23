from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from config_loader import MappingConfig, Pose6D


Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]


@dataclass(frozen=True)
class PickPlan:
    target_x_mm: float
    target_y_mm: float
    safe_z_mm: float
    grasp_z_mm: float
    target_rx_deg: float
    target_ry_deg: float
    target_rz_deg: float
    delta_camera_xy_mm: tuple[float, float]
    delta_base_xy_mm: tuple[float, float]


class TargetOffsetError(RuntimeError):
    """Raised when a camera-to-base target offset is unsafe or invalid."""


def map_marker_to_target_xy(
    marker_tvec_mm: Sequence[float],
    *,
    observe_pose: Pose6D,
    mapping: MappingConfig,
) -> tuple[float, float, tuple[float, float], tuple[float, float]]:
    if len(marker_tvec_mm) < 2:
        raise TargetOffsetError("marker_tvec_mm must contain at least x/y")
    camera_x = _finite(marker_tvec_mm[0], "marker_tvec_mm[0]")
    camera_y = _finite(marker_tvec_mm[1], "marker_tvec_mm[1]")
    reference_camera_xy = mapping.reference_camera_xy_mm or (0.0, 0.0)
    delta_camera = (
        camera_x - reference_camera_xy[0],
        camera_y - reference_camera_xy[1],
    )
    if mapping.camera_to_tool is None:
        matrix = mapping.camera_xy_to_base_xy
        if matrix is None:
            raise TargetOffsetError("mapping.camera_xy_to_base_xy is required when camera_to_tool is not configured")
        delta_base = (
            matrix[0][0] * delta_camera[0] + matrix[0][1] * delta_camera[1],
            matrix[1][0] * delta_camera[0] + matrix[1][1] * delta_camera[1],
        )
    else:
        if len(marker_tvec_mm) < 3:
            raise TargetOffsetError("marker_tvec_mm must contain x/y/z when camera_to_tool is configured")
        camera_z = _finite(marker_tvec_mm[2], "marker_tvec_mm[2]")
        marker_base = _opencv_marker_to_base(
            (camera_x, camera_y, camera_z),
            observe_pose=observe_pose,
            mapping=mapping,
        )
        delta_base = (
            marker_base[0] - observe_pose.x_mm,
            marker_base[1] - observe_pose.y_mm,
        )
    _validate_offset(delta_base, mapping.max_xy_offset_mm)
    if mapping.camera_to_tool is None:
        target_x = observe_pose.x_mm + delta_base[0] + mapping.grasp_offset_base_xy_mm[0]
        target_y = observe_pose.y_mm + delta_base[1] + mapping.grasp_offset_base_xy_mm[1]
    else:
        target_x = marker_base[0] + mapping.grasp_offset_base_xy_mm[0]
        target_y = marker_base[1] + mapping.grasp_offset_base_xy_mm[1]
    return target_x, target_y, delta_camera, delta_base


def _opencv_marker_to_base(
    marker_tvec_mm: Vector3,
    *,
    observe_pose: Pose6D,
    mapping: MappingConfig,
) -> Vector3:
    if mapping.camera_to_tool is None or mapping.opencv_to_camera_rotation is None:
        raise TargetOffsetError("camera_to_tool and opencv_to_camera are required for 3D mapping")
    marker_camera = _matrix_vector(mapping.opencv_to_camera_rotation, marker_tvec_mm)
    tool_from_camera = _rotation_from_euler_deg(mapping.camera_to_tool.rotation_deg)
    marker_tool = _vector_add(
        mapping.camera_to_tool.translation_mm,
        _matrix_vector(tool_from_camera, marker_camera),
    )
    base_from_tool = _rotation_from_euler_deg(
        (observe_pose.rx_deg, observe_pose.ry_deg, observe_pose.rz_deg)
    )
    return _vector_add(
        (observe_pose.x_mm, observe_pose.y_mm, observe_pose.z_mm),
        _matrix_vector(base_from_tool, marker_tool),
    )


def _rotation_from_euler_deg(rotation_deg: Vector3) -> Matrix3:
    rx, ry, rz = rotation_deg
    return _matrix_multiply(
        _rotation_z(rz),
        _matrix_multiply(_rotation_y(ry), _rotation_x(rx)),
    )


def _rotation_x(angle_deg: float) -> Matrix3:
    angle_rad = math.radians(angle_deg)
    cos_angle = math.cos(angle_rad)
    sin_angle = math.sin(angle_rad)
    return (
        (1.0, 0.0, 0.0),
        (0.0, cos_angle, -sin_angle),
        (0.0, sin_angle, cos_angle),
    )


def _rotation_y(angle_deg: float) -> Matrix3:
    angle_rad = math.radians(angle_deg)
    cos_angle = math.cos(angle_rad)
    sin_angle = math.sin(angle_rad)
    return (
        (cos_angle, 0.0, sin_angle),
        (0.0, 1.0, 0.0),
        (-sin_angle, 0.0, cos_angle),
    )


def _rotation_z(angle_deg: float) -> Matrix3:
    angle_rad = math.radians(angle_deg)
    cos_angle = math.cos(angle_rad)
    sin_angle = math.sin(angle_rad)
    return (
        (cos_angle, -sin_angle, 0.0),
        (sin_angle, cos_angle, 0.0),
        (0.0, 0.0, 1.0),
    )


def _matrix_multiply(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def _matrix_vector(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def _vector_add(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[0] + right[0],
        left[1] + right[1],
        left[2] + right[2],
    )


def _validate_offset(delta_base_xy: tuple[float, float], max_xy_offset_mm: float) -> None:
    max_component = max(abs(delta_base_xy[0]), abs(delta_base_xy[1]))
    distance = math.hypot(delta_base_xy[0], delta_base_xy[1])
    if max_component > max_xy_offset_mm or distance > max_xy_offset_mm:
        raise TargetOffsetError(
            "target offset exceeds mapping.max_xy_offset_mm: "
            f"delta_base_xy=({delta_base_xy[0]:.3f}, {delta_base_xy[1]:.3f}), "
            f"distance={distance:.3f}, max={max_xy_offset_mm:.3f}"
        )


def _finite(value: object, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise TargetOffsetError(f"{name} must be finite") from exc
    if not math.isfinite(result):
        raise TargetOffsetError(f"{name} must be finite")
    return result
