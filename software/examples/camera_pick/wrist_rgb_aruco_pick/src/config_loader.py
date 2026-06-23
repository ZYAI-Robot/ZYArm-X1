from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

try:
    import yaml
except ModuleNotFoundError as exc:  # pragma: no cover - depends on local env
    yaml = None
    _YAML_IMPORT_ERROR = exc
else:
    _YAML_IMPORT_ERROR = None


Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]


@dataclass(frozen=True)
class Pose6D:
    x_mm: float
    y_mm: float
    z_mm: float
    rx_deg: float
    ry_deg: float
    rz_deg: float

    def as_move_ik_args(self) -> tuple[float, float, float, float, float, float]:
        return (self.x_mm, self.y_mm, self.z_mm, self.rx_deg, self.ry_deg, self.rz_deg)


@dataclass(frozen=True)
class MarkerConfig:
    dictionary: str
    marker_id: int
    size_mm: float
    max_reprojection_error_px: float


@dataclass(frozen=True)
class CameraToToolConfig:
    translation_mm: Vector3
    rotation_deg: Vector3


@dataclass(frozen=True)
class MappingConfig:
    reference_camera_xy_mm: tuple[float, float] | None
    camera_xy_to_base_xy: tuple[tuple[float, float], tuple[float, float]] | None
    grasp_offset_base_xy_mm: tuple[float, float]
    max_xy_offset_mm: float
    camera_to_tool: CameraToToolConfig | None = None
    opencv_to_camera_rotation: Matrix3 | None = None


@dataclass(frozen=True)
class TaskConfig:
    safe_z_mm: float
    grasp_z_mm: float
    grasp_pause_s: float


@dataclass(frozen=True)
class LoopConfig:
    max_cycles: int | None
    wait_for_user: bool


@dataclass(frozen=True)
class SafetyConfig:
    require_marker_usable: bool


@dataclass(frozen=True)
class WristArucoConfig:
    camera: dict[str, Any]
    marker: MarkerConfig
    observe_pose: Pose6D
    place_pose: Pose6D
    mapping: MappingConfig
    task: TaskConfig
    loop: LoopConfig
    safety: SafetyConfig


def load_config(path: Path) -> WristArucoConfig:
    if yaml is None:
        raise RuntimeError(
            "Missing dependency PyYAML. Install it with `python -m pip install PyYAML`."
        ) from _YAML_IMPORT_ERROR
    if not path.is_file():
        raise FileNotFoundError(f"Config file not found: {path}")
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, Mapping):
        raise ValueError("Config root must be a mapping")
    return parse_config(raw)


def parse_config(raw: Mapping[str, Any]) -> WristArucoConfig:
    camera = _mapping(raw, "camera")
    _validate_camera(camera)
    marker = _marker_config(_mapping(raw, "marker"))
    observe_pose = _pose(_mapping(raw, "observe_pose"), "observe_pose")
    place_pose = _pose(_mapping(raw, "place_pose"), "place_pose")
    mapping = _mapping_config(_mapping(raw, "mapping"))
    task = _task_config(_mapping(raw, "task"))
    loop = _loop_config(_optional_mapping(raw, "loop"))
    safety = _safety_config(_optional_mapping(raw, "safety"))
    return WristArucoConfig(
        camera=dict(camera),
        marker=marker,
        observe_pose=observe_pose,
        place_pose=place_pose,
        mapping=mapping,
        task=task,
        loop=loop,
        safety=safety,
    )


def _validate_camera(camera: Mapping[str, Any]) -> None:
    for key in ("width", "height", "camera_matrix", "dist_coeffs"):
        if key not in camera:
            raise ValueError(f"camera.{key} is required")
    width = _positive_number(camera["width"], "camera.width")
    height = _positive_number(camera["height"], "camera.height")
    if int(width) <= 0 or int(height) <= 0:
        raise ValueError("camera.width and camera.height must be positive")
    _matrix(camera["camera_matrix"], "camera.camera_matrix", 3, 3)
    dist_coeffs = camera["dist_coeffs"]
    if not isinstance(dist_coeffs, Sequence) or isinstance(dist_coeffs, (str, bytes)):
        raise ValueError("camera.dist_coeffs must be a finite numeric sequence")
    if len(dist_coeffs) not in (4, 5, 8, 12, 14):
        raise ValueError("camera.dist_coeffs must contain 4, 5, 8, 12, or 14 values")
    for index, value in enumerate(dist_coeffs):
        _finite_number(value, f"camera.dist_coeffs[{index}]")
    if "fps" in camera:
        _positive_number(camera["fps"], "camera.fps")
    if "fourcc" in camera:
        fourcc = camera["fourcc"]
        if not isinstance(fourcc, str) or len(fourcc) != 4 or not fourcc.isascii():
            raise ValueError("camera.fourcc must be a four-character ASCII string")
    if "warmup_frames" in camera:
        warmup_frames = _int(camera["warmup_frames"], "camera.warmup_frames")
        if warmup_frames <= 0:
            raise ValueError("camera.warmup_frames must be positive")
    if "exposure" in camera:
        _finite_number(camera["exposure"], "camera.exposure")
    if "buffersize" in camera:
        buffersize = _int(camera["buffersize"], "camera.buffersize")
        if buffersize <= 0:
            raise ValueError("camera.buffersize must be positive")


def _marker_config(raw: Mapping[str, Any]) -> MarkerConfig:
    dictionary = str(_required(raw, "dictionary", "marker.dictionary"))
    marker_id = _int(raw.get("id"), "marker.id")
    if marker_id < 0:
        raise ValueError("marker.id must be non-negative")
    return MarkerConfig(
        dictionary=dictionary,
        marker_id=marker_id,
        size_mm=_positive_number(_required(raw, "size_mm", "marker.size_mm"), "marker.size_mm"),
        max_reprojection_error_px=_positive_number(
            raw.get("max_reprojection_error_px", 4.0),
            "marker.max_reprojection_error_px",
        ),
    )


def _mapping_config(raw: Mapping[str, Any]) -> MappingConfig:
    camera_to_tool_raw = raw.get("camera_to_tool")
    camera_to_tool = None
    opencv_to_camera_rotation = None
    if camera_to_tool_raw is not None:
        if not isinstance(camera_to_tool_raw, Mapping):
            raise ValueError("mapping.camera_to_tool must be a mapping")
        camera_to_tool = _camera_to_tool_config(camera_to_tool_raw)
        opencv_to_camera_rotation = _opencv_to_camera_rotation(raw)
    matrix = None
    if "camera_xy_to_base_xy" in raw:
        parsed_matrix = _matrix(raw.get("camera_xy_to_base_xy"), "mapping.camera_xy_to_base_xy", 2, 2)
        matrix = (
            (parsed_matrix[0][0], parsed_matrix[0][1]),
            (parsed_matrix[1][0], parsed_matrix[1][1]),
        )
    elif camera_to_tool is None:
        raise ValueError("mapping.camera_xy_to_base_xy is required when mapping.camera_to_tool is not configured")
    reference_camera_xy = None
    if "reference_camera_xy_mm" in raw:
        reference_camera_xy = _pair(raw.get("reference_camera_xy_mm"), "mapping.reference_camera_xy_mm")
    elif camera_to_tool is None:
        raise ValueError("mapping.reference_camera_xy_mm is required when mapping.camera_to_tool is not configured")
    return MappingConfig(
        reference_camera_xy_mm=reference_camera_xy,
        camera_xy_to_base_xy=matrix,
        grasp_offset_base_xy_mm=_pair(raw.get("grasp_offset_base_xy_mm"), "mapping.grasp_offset_base_xy_mm"),
        max_xy_offset_mm=_positive_number(raw.get("max_xy_offset_mm"), "mapping.max_xy_offset_mm"),
        camera_to_tool=camera_to_tool,
        opencv_to_camera_rotation=opencv_to_camera_rotation,
    )


def _camera_to_tool_config(raw: Mapping[str, Any]) -> CameraToToolConfig:
    return CameraToToolConfig(
        translation_mm=_triple(
            _required(raw, "translation_mm", "mapping.camera_to_tool.translation_mm"),
            "mapping.camera_to_tool.translation_mm",
        ),
        rotation_deg=_triple(
            _required(raw, "rotation_deg", "mapping.camera_to_tool.rotation_deg"),
            "mapping.camera_to_tool.rotation_deg",
        ),
    )


def _opencv_to_camera_rotation(raw: Mapping[str, Any]) -> Matrix3:
    opencv_to_camera = _optional_mapping(raw, "opencv_to_camera")
    matrix = _matrix(
        _required(opencv_to_camera, "rotation_matrix", "mapping.opencv_to_camera.rotation_matrix"),
        "mapping.opencv_to_camera.rotation_matrix",
        3,
        3,
    )
    return (
        (matrix[0][0], matrix[0][1], matrix[0][2]),
        (matrix[1][0], matrix[1][1], matrix[1][2]),
        (matrix[2][0], matrix[2][1], matrix[2][2]),
    )


def _task_config(raw: Mapping[str, Any]) -> TaskConfig:
    task = TaskConfig(
        safe_z_mm=_finite_number(_required(raw, "safe_z_mm", "task.safe_z_mm"), "task.safe_z_mm"),
        grasp_z_mm=_finite_number(_required(raw, "grasp_z_mm", "task.grasp_z_mm"), "task.grasp_z_mm"),
        grasp_pause_s=_non_negative_number(raw.get("grasp_pause_s", 1.0), "task.grasp_pause_s"),
    )
    if not task.safe_z_mm > task.grasp_z_mm:
        raise ValueError("task heights must satisfy safe_z_mm > grasp_z_mm")
    return task


def _loop_config(raw: Mapping[str, Any]) -> LoopConfig:
    max_cycles_raw = raw.get("max_cycles", 1)
    max_cycles = None
    if max_cycles_raw is not None:
        max_cycles = _int(max_cycles_raw, "loop.max_cycles")
        if max_cycles <= 0:
            raise ValueError("loop.max_cycles must be positive or null")
    return LoopConfig(
        max_cycles=max_cycles,
        wait_for_user=bool(raw.get("wait_for_user", True)),
    )


def _safety_config(raw: Mapping[str, Any]) -> SafetyConfig:
    return SafetyConfig(require_marker_usable=bool(raw.get("require_marker_usable", True)))


def _pose(raw: Mapping[str, Any], name: str) -> Pose6D:
    return Pose6D(
        x_mm=_finite_number(_required(raw, "x_mm", f"{name}.x_mm"), f"{name}.x_mm"),
        y_mm=_finite_number(_required(raw, "y_mm", f"{name}.y_mm"), f"{name}.y_mm"),
        z_mm=_finite_number(_required(raw, "z_mm", f"{name}.z_mm"), f"{name}.z_mm"),
        rx_deg=_finite_number(_required(raw, "rx_deg", f"{name}.rx_deg"), f"{name}.rx_deg"),
        ry_deg=_finite_number(_required(raw, "ry_deg", f"{name}.ry_deg"), f"{name}.ry_deg"),
        rz_deg=_finite_number(_required(raw, "rz_deg", f"{name}.rz_deg"), f"{name}.rz_deg"),
    )


def _mapping(raw: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = raw.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"{key} must be a mapping")
    return value


def _optional_mapping(raw: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = raw.get(key, {})
    if not isinstance(value, Mapping):
        raise ValueError(f"{key} must be a mapping")
    return value


def _required(raw: Mapping[str, Any], key: str, name: str) -> Any:
    if key not in raw:
        raise ValueError(f"{name} is required")
    return raw[key]


def _pair(value: Any, name: str) -> tuple[float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 2:
        raise ValueError(f"{name} must contain 2 finite numbers")
    return (
        _finite_number(value[0], f"{name}[0]"),
        _finite_number(value[1], f"{name}[1]"),
    )


def _triple(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise ValueError(f"{name} must contain 3 finite numbers")
    return (
        _finite_number(value[0], f"{name}[0]"),
        _finite_number(value[1], f"{name}[1]"),
        _finite_number(value[2], f"{name}[2]"),
    )


def _matrix(value: Any, name: str, rows: int, columns: int) -> tuple[tuple[float, ...], ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != rows:
        raise ValueError(f"{name} must be a {rows}x{columns} matrix")
    matrix: list[tuple[float, ...]] = []
    for row_index, row in enumerate(value):
        if not isinstance(row, Sequence) or isinstance(row, (str, bytes)) or len(row) != columns:
            raise ValueError(f"{name} must be a {rows}x{columns} matrix")
        matrix.append(
            tuple(
                _finite_number(item, f"{name}[{row_index}][{column_index}]")
                for column_index, item in enumerate(row)
            )
        )
    return tuple(matrix)


def _int(value: Any, name: str) -> int:
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be an integer") from exc
    if not math.isfinite(float(result)):
        raise ValueError(f"{name} must be finite")
    return result


def _positive_number(value: Any, name: str) -> float:
    result = _finite_number(value, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def _non_negative_number(value: Any, name: str) -> float:
    result = _finite_number(value, name)
    if result < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return result


def _finite_number(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result
