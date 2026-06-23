from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Optional, Sequence

try:
    import cv2
    import numpy as np
except ModuleNotFoundError as exc:  # pragma: no cover - depends on local env
    cv2 = None
    np = None
    _OPENCV_IMPORT_ERROR = exc
else:
    _OPENCV_IMPORT_ERROR = None


Point2D = tuple[float, float]


@dataclass(frozen=True)
class ArucoPoseDetection:
    detected: bool
    usable: bool
    marker_id: Optional[int] = None
    corners_px: tuple[Point2D, ...] = ()
    center_px: Optional[Point2D] = None
    rvec: Optional[tuple[float, float, float]] = None
    tvec_mm: Optional[tuple[float, float, float]] = None
    distance_mm: Optional[float] = None
    reprojection_error_px: Optional[float] = None
    reason: str = "no_target"

    def to_dict(self) -> dict[str, Any]:
        return {
            "detected": self.detected,
            "usable": self.usable,
            "marker_id": self.marker_id,
            "corners_px": [[round(x, 3), round(y, 3)] for x, y in self.corners_px],
            "center_px": _point_to_list(self.center_px),
            "rvec": _vector_to_list(self.rvec),
            "tvec_mm": _vector_to_list(self.tvec_mm),
            "distance_mm": _round_optional(self.distance_mm),
            "reprojection_error_px": _round_optional(self.reprojection_error_px),
            "reason": self.reason,
        }


class ArucoVision:
    def __init__(
        self,
        camera_config: Mapping[str, Any],
        marker_config: Mapping[str, Any] | Any,
    ) -> None:
        _require_opencv()
        self.camera_matrix = np.asarray(camera_config["camera_matrix"], dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.asarray(camera_config["dist_coeffs"], dtype=np.float64).reshape(-1, 1)
        self.dictionary_name = str(_get(marker_config, "dictionary"))
        self.marker_id = int(_get(marker_config, "marker_id", _get(marker_config, "id", 0)))
        self.marker_size_mm = float(_get(marker_config, "size_mm"))
        self.max_reprojection_error_px = float(_get(marker_config, "max_reprojection_error_px", 4.0))
        if self.marker_size_mm <= 0.0:
            raise ValueError("marker.size_mm must be positive")
        self.dictionary = self._create_dictionary(self.dictionary_name)
        self.detector = self._create_detector(self.dictionary)

    def detect(self, frame_bgr: Any) -> ArucoPoseDetection:
        _require_opencv()
        if frame_bgr is None or getattr(frame_bgr, "ndim", 0) != 3:
            raise ValueError("frame_bgr must be a BGR image")
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detect_markers(gray)
        if ids is None or len(ids) == 0:
            return ArucoPoseDetection(detected=False, usable=False, reason="no_marker")

        flat_ids = [int(value) for value in np.asarray(ids).reshape(-1)]
        if self.marker_id not in flat_ids:
            return ArucoPoseDetection(
                detected=False,
                usable=False,
                reason=f"target_id_not_found:{self.marker_id}; detected={flat_ids}",
            )

        index = flat_ids.index(self.marker_id)
        marker_corners = np.asarray(corners[index], dtype=np.float64).reshape(4, 2)
        return self._estimate_pose(marker_corners)

    def draw_overlay(self, frame_bgr: Any, detection: ArucoPoseDetection) -> Any:
        _require_opencv()
        overlay = frame_bgr.copy()
        red = (0, 0, 255)
        green = (0, 220, 0)
        yellow = (0, 220, 220)

        if detection.detected and detection.corners_px:
            corners = np.asarray(detection.corners_px, dtype=np.float32).reshape(1, 4, 2)
            ids = np.asarray([[detection.marker_id]], dtype=np.int32)
            cv2.aruco.drawDetectedMarkers(overlay, [corners], ids)
            if detection.center_px is not None:
                center = tuple(int(round(value)) for value in detection.center_px)
                cv2.drawMarker(overlay, center, red, markerType=cv2.MARKER_CROSS, markerSize=18, thickness=2)
            if detection.usable and detection.rvec is not None and detection.tvec_mm is not None:
                cv2.drawFrameAxes(
                    overlay,
                    self.camera_matrix,
                    self.dist_coeffs,
                    np.asarray(detection.rvec, dtype=np.float64).reshape(3, 1),
                    np.asarray(detection.tvec_mm, dtype=np.float64).reshape(3, 1),
                    self.marker_size_mm * 0.5,
                )
            label = (
                f"id={detection.marker_id} "
                f"t={_format_vector(detection.tvec_mm)} "
                f"err={_round_optional(detection.reprojection_error_px)}"
            )
            cv2.putText(overlay, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, green, 2, cv2.LINE_AA)
            cv2.putText(overlay, detection.reason, (12, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.6, green if detection.usable else yellow, 2, cv2.LINE_AA)
        else:
            cv2.putText(overlay, f"no target: {detection.reason}", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, red, 2, cv2.LINE_AA)
        return overlay

    def _estimate_pose(self, corners_px: Any) -> ArucoPoseDetection:
        object_points = self._marker_object_points()
        image_points = np.asarray(corners_px, dtype=np.float64).reshape(4, 2)
        flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
        solved, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=flag,
        )
        if not solved:
            return self._raw_detection(image_points, usable=False, reason="solve_pnp_failed")

        projected, _jacobian = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            self.camera_matrix,
            self.dist_coeffs,
        )
        errors = np.linalg.norm(projected.reshape(4, 2) - image_points, axis=1)
        reprojection_error = float(np.sqrt(np.mean(errors * errors)))
        usable = reprojection_error <= self.max_reprojection_error_px
        reason = "ok" if usable else "reprojection_error_too_high"
        tvec_tuple = tuple(float(value) for value in np.asarray(tvec).reshape(3))
        rvec_tuple = tuple(float(value) for value in np.asarray(rvec).reshape(3))
        return ArucoPoseDetection(
            detected=True,
            usable=usable,
            marker_id=self.marker_id,
            corners_px=tuple((float(x), float(y)) for x, y in image_points),
            center_px=_center(image_points),
            rvec=rvec_tuple,
            tvec_mm=tvec_tuple,
            distance_mm=float(np.linalg.norm(np.asarray(tvec_tuple, dtype=np.float64))),
            reprojection_error_px=reprojection_error,
            reason=reason,
        )

    def _raw_detection(self, image_points: Any, *, usable: bool, reason: str) -> ArucoPoseDetection:
        points = np.asarray(image_points, dtype=np.float64).reshape(4, 2)
        return ArucoPoseDetection(
            detected=True,
            usable=usable,
            marker_id=self.marker_id,
            corners_px=tuple((float(x), float(y)) for x, y in points),
            center_px=_center(points),
            reason=reason,
        )

    def _marker_object_points(self) -> Any:
        half = self.marker_size_mm / 2.0
        return np.asarray(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )

    def _detect_markers(self, gray: Any) -> tuple[Any, Any]:
        if hasattr(self.detector, "detectMarkers"):
            corners, ids, _rejected = self.detector.detectMarkers(gray)
            return corners, ids
        corners, ids, _rejected = cv2.aruco.detectMarkers(gray, self.dictionary)
        return corners, ids

    @staticmethod
    def _create_dictionary(dictionary_name: str) -> Any:
        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        if dictionary_id is None:
            raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}")
        return cv2.aruco.getPredefinedDictionary(dictionary_id)

    @staticmethod
    def _create_detector(dictionary: Any) -> Any:
        parameters = cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, "ArucoDetector"):
            return cv2.aruco.ArucoDetector(dictionary, parameters)
        return None


def _center(points: Any) -> Point2D:
    center = np.mean(np.asarray(points, dtype=np.float64).reshape(4, 2), axis=0)
    return float(center[0]), float(center[1])


def _get(source: Mapping[str, Any] | Any, key: str, default: Any = None) -> Any:
    if isinstance(source, Mapping):
        if key in source:
            return source[key]
        if default is not None:
            return default
        raise KeyError(key)
    value = getattr(source, key, None)
    if value is not None:
        return value
    if default is not None:
        return default
    raise KeyError(key)


def _point_to_list(point: Optional[Point2D]) -> Optional[list[float]]:
    if point is None:
        return None
    return [round(float(point[0]), 3), round(float(point[1]), 3)]


def _vector_to_list(vector: Optional[Sequence[float]]) -> Optional[list[float]]:
    if vector is None:
        return None
    return [round(float(value), 3) for value in vector]


def _round_optional(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), 3)


def _format_vector(vector: Optional[Sequence[float]]) -> str:
    if vector is None:
        return "n/a"
    return "(" + ",".join(f"{float(value):.1f}" for value in vector) + ")"


def _require_opencv() -> None:
    if cv2 is None or np is None:
        raise RuntimeError(
            "Missing dependency OpenCV contrib. Install it with `python -m pip install opencv-contrib-python`."
        ) from _OPENCV_IMPORT_ERROR
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV ArUco module is required; install opencv-contrib-python.")
