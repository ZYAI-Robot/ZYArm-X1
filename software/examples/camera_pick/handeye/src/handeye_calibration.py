from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Optional, Sequence

import cv2
import numpy as np


Point2D = tuple[float, float]
Point3D = tuple[float, float, float]


@dataclass(frozen=True)
class BoardDetection:
    marker_ids: tuple[int, ...]
    corners_by_id: dict[int, np.ndarray]
    object_points_base_mm: np.ndarray
    image_points_px: np.ndarray


@dataclass(frozen=True)
class CalibrationResult:
    t_camera_base: np.ndarray
    t_base_camera: np.ndarray
    rvec_camera_base: np.ndarray
    tvec_camera_base: np.ndarray
    reprojection_error_px: float
    object_points_base_mm: np.ndarray
    image_points_px: np.ndarray
    marker_ids: tuple[int, ...]
    homography_base_to_image: Optional[np.ndarray] = None
    homography_image_to_base: Optional[np.ndarray] = None
    planar_reprojection_error_px: float = math.nan
    planar_control_error_px: float = math.nan
    planar_fit_mode: str = "all_corners"
    point_errors_px: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )
    marker_errors_px: dict[int, float] = field(default_factory=dict)
    planar_marker_errors_px: dict[int, float] = field(default_factory=dict)


class HandEyeCalibrator:
    """Eye-to-hand PnP calibration and base-plane projection."""

    REQUIRED_MARKER_IDS = (0, 1, 2, 3)
    MAX_REPROJECTION_ERROR_PX = 8.0
    PNP_WARNING_ERROR_PX = 2.0
    MAX_PLANAR_REPROJECTION_ERROR_PX = 2.0
    STABLE_SAMPLES_PER_MARKER = 6
    MAX_CAPTURE_FRAMES = 100
    MAX_CORNER_JITTER_PX = 1.5
    CAPTURE_RESET_MOTION_PX = 4.0
    RAY_PARALLEL_EPSILON = 1e-9

    def __init__(
        self,
        camera_config: Mapping[str, Any],
        board_config: Mapping[str, Any],
    ) -> None:
        self.camera_config = dict(camera_config)
        self.board_config = dict(board_config)
        self.camera_matrix, self.dist_coeffs = self._validate_camera_config()
        (
            self.dictionary_name,
            self.marker_points,
            self.origin_base_mm,
            self.board_to_base_xy,
        ) = self._validate_board_config()
        self._dictionary = self._create_dictionary(self.dictionary_name)
        self._detector_parameters = self._create_detector_parameters()
        self._detector = self._create_detector(
            self._dictionary,
            self._detector_parameters,
        )
        self.max_reprojection_error_px = self._positive_float_config(
            "max_reprojection_error_px",
            self.MAX_REPROJECTION_ERROR_PX,
        )
        self.max_planar_reprojection_error_px = self._positive_float_config(
            "max_planar_reprojection_error_px",
            self.MAX_PLANAR_REPROJECTION_ERROR_PX,
        )
        self.planar_fit_mode = str(
            self.board_config.get("planar_fit_mode", "marker_centers")
        )
        if self.planar_fit_mode not in ("marker_centers", "all_corners"):
            raise ValueError(
                "board.planar_fit_mode must be 'marker_centers' or 'all_corners'"
            )
        self.stable_samples_per_marker = self._positive_int_config(
            "stable_samples_per_marker",
            self.STABLE_SAMPLES_PER_MARKER,
        )
        self.max_capture_frames = self._positive_int_config(
            "max_capture_frames",
            self.MAX_CAPTURE_FRAMES,
        )
        self.max_corner_jitter_px = self._positive_float_config(
            "max_corner_jitter_px",
            self.MAX_CORNER_JITTER_PX,
        )
        self.capture_reset_motion_px = self._positive_float_config(
            "capture_reset_motion_px",
            self.CAPTURE_RESET_MOTION_PX,
        )
        self._last_result: Optional[CalibrationResult] = None

    @property
    def last_result(self) -> Optional[CalibrationResult]:
        return self._last_result

    def detect_board(self, frame_bgr: np.ndarray) -> BoardDetection:
        corners_by_id = self.detect_visible_markers(frame_bgr)
        missing = [
            marker_id
            for marker_id in self.REQUIRED_MARKER_IDS
            if marker_id not in corners_by_id
        ]
        if missing:
            detected = sorted(corners_by_id)
            if not detected:
                raise RuntimeError("No required ArUco markers were detected")
            raise RuntimeError(
                f"Missing required ArUco marker IDs: {missing}; detected IDs: {detected}"
            )
        return self._make_board_detection(corners_by_id)

    def detect_visible_markers(self, frame_bgr: np.ndarray) -> dict[int, np.ndarray]:
        if frame_bgr is None or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("frame_bgr must be a BGR image")
        self._validate_frame_resolution(frame_bgr)

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        corners_by_id = self._detect_on_gray(gray)

        missing = set(self.REQUIRED_MARKER_IDS) - set(corners_by_id)
        if missing:
            enhanced = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
            enhanced_corners = self._detect_on_gray(enhanced)
            for marker_id in missing:
                if marker_id in enhanced_corners:
                    corners_by_id[marker_id] = enhanced_corners[marker_id]
        return corners_by_id

    def calibrate_stable(
        self,
        read_frame: Callable[[], np.ndarray],
    ) -> tuple[BoardDetection, CalibrationResult]:
        observations: list[dict[int, np.ndarray]] = []
        last_error = "No required ArUco markers were detected"

        for _frame_index in range(self.max_capture_frames):
            frame = read_frame()
            visible = self.detect_visible_markers(frame)
            if visible:
                self.append_observation(observations, visible)
            try:
                detection = self.build_stable_detection(observations)
                result = self.calibrate_points(
                    detection.object_points_base_mm,
                    detection.image_points_px,
                    marker_ids=detection.marker_ids,
                )
                return detection, result
            except RuntimeError as exc:
                last_error = str(exc)

        raise RuntimeError(self._capture_timeout_message(observations, last_error))

    def build_stable_detection(
        self,
        observations: Sequence[Mapping[int, np.ndarray]],
    ) -> BoardDetection:
        corners_by_id: dict[int, np.ndarray] = {}
        unstable: list[str] = []
        counts = self.observation_counts(observations)

        for marker_id in self.REQUIRED_MARKER_IDS:
            samples = [
                np.asarray(observation[marker_id], dtype=np.float64).reshape(4, 2)
                for observation in observations
                if marker_id in observation
            ]
            if len(samples) < self.stable_samples_per_marker:
                continue
            center, jitter = self._find_stable_corner_window(samples)
            if center is None:
                unstable.append(f"{marker_id}:{jitter:.2f}px")
                continue
            corners_by_id[marker_id] = center

        missing = [
            marker_id
            for marker_id in self.REQUIRED_MARKER_IDS
            if marker_id not in corners_by_id
        ]
        if missing:
            detail = f"; unstable={unstable}" if unstable else ""
            raise RuntimeError(
                "Collecting stable ArUco corners: "
                f"missing={missing}; samples={counts}{detail}"
            )
        return self._make_board_detection(corners_by_id)

    def _find_stable_corner_window(
        self,
        samples: Sequence[np.ndarray],
    ) -> tuple[Optional[np.ndarray], float]:
        window_size = self.stable_samples_per_marker
        best_jitter = math.inf

        for start in range(len(samples) - window_size, -1, -1):
            window = np.stack(
                samples[start : start + window_size],
                axis=0,
            )
            center = np.median(window, axis=0)
            jitter = float(
                np.sqrt(np.mean(np.sum((window - center) ** 2, axis=2)))
            )
            best_jitter = min(best_jitter, jitter)
            if jitter <= self.max_corner_jitter_px:
                return center, jitter

        return None, best_jitter

    def observation_counts(
        self,
        observations: Sequence[Mapping[int, np.ndarray]],
    ) -> dict[int, int]:
        return {
            marker_id: sum(marker_id in observation for observation in observations)
            for marker_id in self.REQUIRED_MARKER_IDS
        }

    def append_observation(
        self,
        observations: list[dict[int, np.ndarray]],
        visible: Mapping[int, np.ndarray],
    ) -> None:
        if observations:
            reference = self._recent_marker_reference(observations)
            motions = [
                self._corner_motion(visible[marker_id], reference[marker_id])
                for marker_id in visible
                if marker_id in reference
            ]
            if motions and float(np.median(motions)) > self.capture_reset_motion_px:
                observations.clear()

        observations.append(
            {
                marker_id: np.asarray(points, dtype=np.float64).reshape(4, 2).copy()
                for marker_id, points in visible.items()
                if marker_id in self.REQUIRED_MARKER_IDS
            }
        )

    def calibrate(self, frame_bgr: np.ndarray) -> CalibrationResult:
        detection = self.detect_board(frame_bgr)
        result = self.calibrate_points(
            detection.object_points_base_mm,
            detection.image_points_px,
            marker_ids=detection.marker_ids,
        )
        self._last_result = result
        return result

    def calibrate_points(
        self,
        object_points_base_mm: Sequence[Sequence[float]],
        image_points_px: Sequence[Sequence[float]],
        *,
        marker_ids: Sequence[int] = REQUIRED_MARKER_IDS,
    ) -> CalibrationResult:
        object_points, image_points = self._normalize_calibration_points(
            object_points_base_mm,
            image_points_px,
        )
        rvec, tvec, t_camera_base, t_base_camera = self._solve_camera_pose(
            object_points,
            image_points,
        )
        errors, reprojection_error = self._calculate_pnp_errors(
            object_points,
            image_points,
            rvec,
            tvec,
        )
        (
            homography_base_to_image,
            homography_image_to_base,
            planar_reprojection_error,
            planar_control_error,
            planar_marker_errors,
        ) = self._calculate_plane_homography(
            object_points,
            image_points,
            marker_ids,
        )
        marker_errors = self._marker_errors(errors, marker_ids)

        result = CalibrationResult(
            t_camera_base=t_camera_base,
            t_base_camera=t_base_camera,
            rvec_camera_base=np.asarray(rvec, dtype=np.float64).reshape(3, 1),
            tvec_camera_base=np.asarray(tvec, dtype=np.float64).reshape(3, 1),
            reprojection_error_px=reprojection_error,
            object_points_base_mm=object_points.copy(),
            image_points_px=image_points.copy(),
            marker_ids=tuple(int(value) for value in marker_ids),
            homography_base_to_image=homography_base_to_image,
            homography_image_to_base=homography_image_to_base,
            planar_reprojection_error_px=planar_reprojection_error,
            planar_control_error_px=planar_control_error,
            planar_fit_mode=self.planar_fit_mode,
            point_errors_px=errors.copy(),
            marker_errors_px=marker_errors,
            planar_marker_errors_px=planar_marker_errors,
        )
        self.validate(result)
        self._last_result = result
        return result

    def validate(self, result: CalibrationResult) -> None:
        self._validate_transforms(result)
        self._validate_pnp_result(result)
        self._validate_planar_result(result)

    def _validate_transforms(self, result: CalibrationResult) -> None:
        for name, matrix in (
            ("T_camera_base", result.t_camera_base),
            ("T_base_camera", result.t_base_camera),
        ):
            if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
                raise RuntimeError(f"{name} must be a finite 4x4 matrix")
            if not np.allclose(matrix[3], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-8):
                raise RuntimeError(f"{name} has an invalid homogeneous row")

        rotation = result.t_camera_base[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
            raise RuntimeError("PnP rotation matrix is not orthonormal")
        if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-5):
            raise RuntimeError("PnP rotation matrix determinant is not +1")
        if not np.allclose(
            result.t_camera_base @ result.t_base_camera,
            np.eye(4),
            atol=1e-6,
        ):
            raise RuntimeError("PnP transform inverse is inconsistent")

    def _validate_pnp_result(self, result: CalibrationResult) -> None:
        camera_points = self._transform_points(
            result.t_camera_base,
            result.object_points_base_mm,
        )
        if np.any(camera_points[:, 2] <= 0.0):
            raise RuntimeError("PnP placed one or more board points behind the camera")
        if not math.isfinite(result.reprojection_error_px):
            raise RuntimeError("PnP reprojection error is not finite")
        if result.reprojection_error_px > self.max_reprojection_error_px:
            raise RuntimeError(
                "PnP reprojection error is too high: "
                f"{result.reprojection_error_px:.3f}px > "
                f"{self.max_reprojection_error_px:.3f}px; "
                f"per-marker RMSE: {self._format_marker_errors(result.marker_errors_px)}"
            )

    def _validate_planar_result(self, result: CalibrationResult) -> None:
        if result.homography_image_to_base is None:
            return
        if not math.isfinite(result.planar_control_error_px):
            raise RuntimeError("Planar control-point error is not finite")
        if result.planar_control_error_px > self.max_planar_reprojection_error_px:
            raise RuntimeError(
                "Planar control-point fit is too high: "
                f"{result.planar_control_error_px:.3f}px > "
                f"{self.max_planar_reprojection_error_px:.3f}px; "
                "check marker center coordinates and placement"
            )
        if (
            result.planar_fit_mode == "all_corners"
            and result.planar_reprojection_error_px
            > self.max_planar_reprojection_error_px
        ):
            raise RuntimeError(
                "Planar marker fit is too high: "
                f"{result.planar_reprojection_error_px:.3f}px > "
                f"{self.max_planar_reprojection_error_px:.3f}px; "
                "check printed marker dimensions, flatness, and corner ordering"
            )

    def pixel_to_base(
        self,
        point_px: Sequence[float],
        *,
        result: Optional[CalibrationResult] = None,
    ) -> np.ndarray:
        calibration = result or self._last_result
        if calibration is None:
            raise RuntimeError("Calibrate the camera before converting image points")
        point = np.asarray(point_px, dtype=np.float64).reshape(1, 1, 2)
        if not np.all(np.isfinite(point)):
            raise ValueError("Image point must contain finite values")

        if calibration.homography_image_to_base is not None:
            point_base_xy = cv2.perspectiveTransform(
                point,
                calibration.homography_image_to_base,
            ).reshape(2)
            point_base = np.array(
                [point_base_xy[0], point_base_xy[1], 0.0],
                dtype=np.float64,
            )
            if not np.all(np.isfinite(point_base)):
                raise RuntimeError("Planar pixel-to-base conversion produced non-finite values")
            return point_base

        normalized = cv2.undistortPoints(point, self.camera_matrix, self.dist_coeffs)
        x_normalized, y_normalized = normalized.reshape(2)
        ray_camera = np.array([x_normalized, y_normalized, 1.0], dtype=np.float64)

        rotation_base_camera = calibration.t_base_camera[:3, :3]
        camera_origin_base = calibration.t_base_camera[:3, 3]
        ray_base = rotation_base_camera @ ray_camera
        if abs(float(ray_base[2])) <= self.RAY_PARALLEL_EPSILON:
            raise RuntimeError("Camera ray is parallel to the base_link z=0 plane")

        scale = -float(camera_origin_base[2]) / float(ray_base[2])
        if scale <= 0.0:
            raise RuntimeError("Camera ray intersects the base plane behind the camera")
        point_base = camera_origin_base + scale * ray_base
        point_base[2] = 0.0
        if not np.all(np.isfinite(point_base)):
            raise RuntimeError("Pixel-to-base conversion produced non-finite values")
        return point_base

    def direction_to_base(
        self,
        axis_points_px: Sequence[Sequence[float]],
        *,
        result: Optional[CalibrationResult] = None,
    ) -> float:
        points = np.asarray(axis_points_px, dtype=np.float64).reshape(2, 2)
        first = self.pixel_to_base(points[0], result=result)
        second = self.pixel_to_base(points[1], result=result)
        direction = second[:2] - first[:2]
        if float(np.linalg.norm(direction)) <= 1e-9:
            raise RuntimeError("Projected block direction is degenerate")
        angle = float(
            math.degrees(math.atan2(float(direction[1]), float(direction[0]))) % 180.0
        )
        return 0.0 if math.isclose(angle, 180.0, abs_tol=1e-9) else angle

    def draw_result(
        self,
        frame_bgr: np.ndarray,
        detection: BoardDetection,
        result: CalibrationResult,
        *,
        axis_length_mm: float = 30.0,
    ) -> np.ndarray:
        overlay = frame_bgr.copy()
        self._draw_marker_observations(overlay, detection)
        self._draw_reprojected_points(overlay, result)
        self._draw_base_axes(overlay, result, axis_length_mm)
        self._draw_calibration_metrics(overlay, result)
        return overlay

    def _draw_marker_observations(
        self,
        overlay: np.ndarray,
        detection: BoardDetection,
    ) -> None:
        marker_corners = [
            detection.corners_by_id[marker_id].reshape(1, 4, 2).astype(np.float32)
            for marker_id in detection.marker_ids
        ]
        marker_ids = np.asarray(detection.marker_ids, dtype=np.int32).reshape(-1, 1)
        cv2.aruco.drawDetectedMarkers(overlay, marker_corners, marker_ids)

        for marker_id in detection.marker_ids:
            for index, point in enumerate(detection.corners_by_id[marker_id]):
                position = tuple(int(round(value)) for value in point)
                cv2.circle(overlay, position, 3, (255, 0, 255), -1)
                cv2.putText(
                    overlay,
                    f"{marker_id}:{index}",
                    (position[0] + 4, position[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 0, 255),
                    1,
                    cv2.LINE_AA,
                )

    def _draw_reprojected_points(
        self,
        overlay: np.ndarray,
        result: CalibrationResult,
    ) -> None:
        projected, _jacobian = cv2.projectPoints(
            result.object_points_base_mm,
            result.rvec_camera_base,
            result.tvec_camera_base,
            self.camera_matrix,
            self.dist_coeffs,
        )
        for point in projected.reshape(-1, 2):
            position = tuple(int(round(value)) for value in point)
            cv2.circle(overlay, position, 2, (0, 0, 255), -1)

    def _draw_base_axes(
        self,
        overlay: np.ndarray,
        result: CalibrationResult,
        axis_length_mm: float,
    ) -> None:
        origin = self.origin_base_mm
        axis_points = np.asarray(
            [
                origin,
                origin + np.array([axis_length_mm, 0.0, 0.0]),
                origin + np.array([0.0, axis_length_mm, 0.0]),
                origin + np.array([0.0, 0.0, axis_length_mm]),
            ],
            dtype=np.float64,
        )
        if result.homography_base_to_image is not None:
            projected_xy = cv2.perspectiveTransform(
                axis_points[:3, :2].reshape(-1, 1, 2),
                result.homography_base_to_image,
            ).reshape(-1, 2)
            axis_pixels = np.rint(projected_xy).astype(np.int32)
        else:
            projected_axes, _jacobian = cv2.projectPoints(
                axis_points[:3],
                result.rvec_camera_base,
                result.tvec_camera_base,
                self.camera_matrix,
                self.dist_coeffs,
            )
            axis_pixels = np.rint(projected_axes.reshape(-1, 2)).astype(np.int32)
        axis_origin = tuple(int(value) for value in axis_pixels[0])
        cv2.line(overlay, axis_origin, tuple(axis_pixels[1]), (0, 0, 255), 2)
        cv2.line(overlay, axis_origin, tuple(axis_pixels[2]), (0, 255, 0), 2)
        if result.reprojection_error_px <= self.PNP_WARNING_ERROR_PX:
            projected_z, _jacobian = cv2.projectPoints(
                axis_points[[0, 3]],
                result.rvec_camera_base,
                result.tvec_camera_base,
                self.camera_matrix,
                self.dist_coeffs,
            )
            z_pixels = np.rint(projected_z.reshape(-1, 2)).astype(np.int32)
            cv2.line(
                overlay,
                tuple(int(value) for value in z_pixels[0]),
                tuple(int(value) for value in z_pixels[1]),
                (255, 0, 0),
                2,
            )

    def _draw_calibration_metrics(
        self,
        overlay: np.ndarray,
        result: CalibrationResult,
    ) -> None:
        pnp_color = (
            (0, 200, 0)
            if result.reprojection_error_px <= self.PNP_WARNING_ERROR_PX
            else (0, 165, 255)
        )
        plane_color = (
            (0, 200, 0)
            if result.planar_control_error_px
            <= self.max_planar_reprojection_error_px
            else (0, 165, 255)
        )
        corner_color = (
            (0, 200, 0)
            if result.planar_reprojection_error_px
            <= self.max_planar_reprojection_error_px
            else (0, 165, 255)
        )
        cv2.putText(
            overlay,
            (
                f"Plane center fit={result.planar_control_error_px:.3f}px  "
                f"corner model={result.planar_reprojection_error_px:.3f}px"
            ),
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            plane_color if corner_color == (0, 200, 0) else corner_color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            (
                f"PnP={result.reprojection_error_px:.3f}px  corner RMSE: "
                f"{self._format_marker_errors(result.planar_marker_errors_px)}"
            ),
            (12, 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            pnp_color,
            2,
            cv2.LINE_AA,
        )
        if (
            result.planar_fit_mode == "marker_centers"
            and result.planar_reprojection_error_px
            > self.max_planar_reprojection_error_px
        ):
            cv2.putText(
                overlay,
                "Loose-marker mode: XY uses marker centers; corner mismatch is diagnostic",
                (12, 84),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 165, 255),
                2,
                cv2.LINE_AA,
            )

    @staticmethod
    def invert_transform(transform: np.ndarray) -> np.ndarray:
        matrix = np.asarray(transform, dtype=np.float64)
        if matrix.shape != (4, 4):
            raise ValueError("Transform must be 4x4")
        rotation = matrix[:3, :3]
        translation = matrix[:3, 3]
        inverse = np.eye(4, dtype=np.float64)
        inverse[:3, :3] = rotation.T
        inverse[:3, 3] = -(rotation.T @ translation)
        return inverse

    def _validate_camera_config(self) -> tuple[np.ndarray, np.ndarray]:
        raw_matrix = self.camera_config.get("camera_matrix")
        raw_distortion = self.camera_config.get("dist_coeffs")
        if raw_matrix is None or raw_distortion is None:
            raise ValueError(
                "Fill camera.camera_matrix and camera.dist_coeffs in config/handeye.py"
            )

        matrix = np.asarray(raw_matrix, dtype=np.float64)
        distortion = np.asarray(raw_distortion, dtype=np.float64).reshape(-1, 1)
        if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            raise ValueError("camera.camera_matrix must be a finite 3x3 matrix")
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            raise ValueError("camera focal lengths must be positive")
        if len(distortion) not in (4, 5, 8, 12, 14) or not np.all(np.isfinite(distortion)):
            raise ValueError(
                "camera.dist_coeffs must contain 4, 5, 8, 12, or 14 finite values"
            )
        return matrix, distortion

    def _validate_board_config(
        self,
    ) -> tuple[str, dict[int, np.ndarray], np.ndarray, np.ndarray]:
        dictionary_name = self.board_config.get("dictionary")
        if not isinstance(dictionary_name, str) or not dictionary_name:
            raise ValueError("Fill board.dictionary in config/handeye.py")

        board_to_base_xy = self._validate_board_to_base_xy()
        origin = self._validate_board_origin(board_to_base_xy)
        marker_points = self._validate_marker_points(board_to_base_xy)
        return dictionary_name, marker_points, origin, board_to_base_xy

    def _validate_board_to_base_xy(self) -> np.ndarray:
        raw_board_to_base = self.board_config.get(
            "board_to_base_xy",
            [[1.0, 0.0], [0.0, 1.0]],
        )
        board_to_base_xy = np.asarray(raw_board_to_base, dtype=np.float64)
        if (
            board_to_base_xy.shape != (2, 2)
            or not np.all(np.isfinite(board_to_base_xy))
        ):
            raise ValueError("board.board_to_base_xy must be a finite 2x2 matrix")
        if not np.allclose(
            board_to_base_xy.T @ board_to_base_xy,
            np.eye(2),
            atol=1e-8,
        ):
            raise ValueError("board.board_to_base_xy must contain only rotation/reflection")
        if not math.isclose(
            abs(float(np.linalg.det(board_to_base_xy))),
            1.0,
            abs_tol=1e-8,
        ):
            raise ValueError("board.board_to_base_xy determinant magnitude must be 1")
        return board_to_base_xy

    def _validate_board_origin(self, board_to_base_xy: np.ndarray) -> np.ndarray:
        raw_origin = self.board_config.get("origin_base_mm")
        origin = np.asarray(raw_origin, dtype=np.float64)
        if origin.shape != (3,) or not np.all(np.isfinite(origin)):
            raise ValueError("board.origin_base_mm must contain three finite values")
        return self._board_points_to_base(
            origin.reshape(1, 3),
            board_to_base_xy,
        )[0]

    def _validate_marker_points(
        self,
        board_to_base_xy: np.ndarray,
    ) -> dict[int, np.ndarray]:
        raw_markers = self.board_config.get("marker_corners_base_mm")
        if not isinstance(raw_markers, Mapping):
            raise ValueError("board.marker_corners_base_mm must be a mapping")

        marker_points: dict[int, np.ndarray] = {}
        for marker_id in self.REQUIRED_MARKER_IDS:
            raw_points = raw_markers.get(marker_id)
            if raw_points is None:
                raise ValueError(
                    "Fill board.marker_corners_base_mm"
                    f"[{marker_id}] in config/handeye.py"
                )
            points = np.asarray(raw_points, dtype=np.float64)
            if points.shape != (4, 3) or not np.all(np.isfinite(points)):
                raise ValueError(
                    f"Marker {marker_id} corners must be a finite 4x3 array"
                )
            marker_points[marker_id] = self._board_points_to_base(
                points,
                board_to_base_xy,
            )
        return marker_points

    @staticmethod
    def _board_points_to_base(
        points: np.ndarray,
        board_to_base_xy: np.ndarray,
    ) -> np.ndarray:
        transformed = np.asarray(points, dtype=np.float64).copy()
        transformed[:, :2] = (
            np.asarray(board_to_base_xy, dtype=np.float64)
            @ transformed[:, :2].T
        ).T
        return transformed

    def _validate_frame_resolution(self, frame_bgr: np.ndarray) -> None:
        height, width = frame_bgr.shape[:2]
        configured_width = self.camera_config.get("width")
        configured_height = self.camera_config.get("height")
        if configured_width is not None and width != int(configured_width):
            raise ValueError(
                f"Frame width {width} does not match camera.width {configured_width}"
            )
        if configured_height is not None and height != int(configured_height):
            raise ValueError(
                f"Frame height {height} does not match camera.height {configured_height}"
            )

    @staticmethod
    def _create_dictionary(dictionary_name: str) -> Any:
        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        if dictionary_id is None:
            raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}")
        return cv2.aruco.getPredefinedDictionary(dictionary_id)

    @staticmethod
    def _create_detector_parameters() -> Any:
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.cornerRefinementWinSize = 5
        parameters.cornerRefinementMaxIterations = 50
        parameters.cornerRefinementMinAccuracy = 0.01
        parameters.adaptiveThreshWinSizeMax = 53
        parameters.adaptiveThreshWinSizeStep = 4
        parameters.minMarkerPerimeterRate = 0.015
        parameters.detectInvertedMarker = True
        if hasattr(parameters, "useAruco3Detection"):
            parameters.useAruco3Detection = True
        return parameters

    @staticmethod
    def _create_detector(dictionary: Any, parameters: Any) -> Any:
        if not hasattr(cv2.aruco, "ArucoDetector"):
            return None
        return cv2.aruco.ArucoDetector(dictionary, parameters)

    def _detect_on_gray(self, gray: np.ndarray) -> dict[int, np.ndarray]:
        if self._detector is not None:
            corners, ids, _rejected = self._detector.detectMarkers(gray)
        else:  # OpenCV compatibility path.
            corners, ids, _rejected = cv2.aruco.detectMarkers(
                gray,
                self._dictionary,
                parameters=self._detector_parameters,
            )
        if ids is None or len(ids) == 0:
            return {}

        candidates: dict[int, np.ndarray] = {}
        for marker_corners, marker_id_value in zip(corners, ids.reshape(-1)):
            marker_id = int(marker_id_value)
            if marker_id not in self.REQUIRED_MARKER_IDS:
                continue
            points = np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)
            previous = candidates.get(marker_id)
            if previous is None or abs(cv2.contourArea(points.astype(np.float32))) > abs(
                cv2.contourArea(previous.astype(np.float32))
            ):
                candidates[marker_id] = points
        return candidates

    def _make_board_detection(
        self,
        corners_by_id: Mapping[int, np.ndarray],
    ) -> BoardDetection:
        normalized_corners = {
            marker_id: np.asarray(corners_by_id[marker_id], dtype=np.float64).reshape(4, 2)
            for marker_id in self.REQUIRED_MARKER_IDS
        }
        return BoardDetection(
            marker_ids=self.REQUIRED_MARKER_IDS,
            corners_by_id=normalized_corners,
            object_points_base_mm=np.concatenate(
                [self.marker_points[marker_id] for marker_id in self.REQUIRED_MARKER_IDS],
                axis=0,
            ),
            image_points_px=np.concatenate(
                [normalized_corners[marker_id] for marker_id in self.REQUIRED_MARKER_IDS],
                axis=0,
            ),
        )

    def _capture_timeout_message(
        self,
        observations: Sequence[Mapping[int, np.ndarray]],
        last_error: str,
    ) -> str:
        counts = self.observation_counts(observations)
        never_detected = [
            marker_id
            for marker_id, count in counts.items()
            if count == 0
        ]
        insufficient = [
            marker_id
            for marker_id, count in counts.items()
            if 0 < count < self.stable_samples_per_marker
        ]
        diagnosis: list[str] = []
        if never_detected:
            diagnosis.append(
                f"never detected IDs={never_detected}; check visibility, glare, "
                "printed border, and dictionary"
            )
        if insufficient:
            diagnosis.append(
                f"insufficient IDs={insufficient}; need at least "
                f"{self.stable_samples_per_marker} samples per ID"
            )
        diagnosis_text = f"; diagnosis: {'; '.join(diagnosis)}" if diagnosis else ""
        return (
            "Stable ArUco capture timed out after "
            f"{self.max_capture_frames} frames; samples={counts}{diagnosis_text}; "
            f"last error: {last_error}"
        )

    def _recent_marker_reference(
        self,
        observations: Sequence[Mapping[int, np.ndarray]],
    ) -> dict[int, np.ndarray]:
        recent = observations[-self.stable_samples_per_marker :]
        reference: dict[int, np.ndarray] = {}
        for marker_id in self.REQUIRED_MARKER_IDS:
            samples = [
                observation[marker_id]
                for observation in recent
                if marker_id in observation
            ]
            if samples:
                reference[marker_id] = np.median(np.stack(samples), axis=0)
        return reference

    @staticmethod
    def _corner_motion(current: np.ndarray, reference: np.ndarray) -> float:
        difference = (
            np.asarray(current, dtype=np.float64)
            - np.asarray(reference, dtype=np.float64)
        )
        return float(np.sqrt(np.mean(np.sum(difference**2, axis=1))))

    @staticmethod
    def _normalize_calibration_points(
        object_points_base_mm: Sequence[Sequence[float]],
        image_points_px: Sequence[Sequence[float]],
    ) -> tuple[np.ndarray, np.ndarray]:
        object_points = np.asarray(
            object_points_base_mm,
            dtype=np.float64,
        ).reshape(-1, 3)
        image_points = np.asarray(
            image_points_px,
            dtype=np.float64,
        ).reshape(-1, 2)
        if len(object_points) < 4:
            raise ValueError("PnP requires at least four 3D/2D point pairs")
        if len(object_points) != len(image_points):
            raise ValueError("PnP object/image point counts do not match")
        if not np.all(np.isfinite(object_points)) or not np.all(
            np.isfinite(image_points)
        ):
            raise ValueError("PnP points must contain finite values")
        return object_points, image_points

    def _solve_camera_pose(
        self,
        object_points: np.ndarray,
        image_points: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        solved, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not solved:
            raise RuntimeError("solvePnP failed to calculate the camera pose")

        rotation, _jacobian = cv2.Rodrigues(rvec)
        t_camera_base = np.eye(4, dtype=np.float64)
        t_camera_base[:3, :3] = rotation
        t_camera_base[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
        return (
            np.asarray(rvec, dtype=np.float64).reshape(3, 1),
            np.asarray(tvec, dtype=np.float64).reshape(3, 1),
            t_camera_base,
            self.invert_transform(t_camera_base),
        )

    def _calculate_pnp_errors(
        self,
        object_points: np.ndarray,
        image_points: np.ndarray,
        rvec: np.ndarray,
        tvec: np.ndarray,
    ) -> tuple[np.ndarray, float]:
        projected, _jacobian = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            self.camera_matrix,
            self.dist_coeffs,
        )
        errors = np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1)
        return errors, float(np.sqrt(np.mean(errors * errors)))

    def _calculate_plane_homography(
        self,
        object_points: np.ndarray,
        image_points: np.ndarray,
        marker_ids: Sequence[int],
    ) -> tuple[
        Optional[np.ndarray],
        Optional[np.ndarray],
        float,
        float,
        dict[int, float],
    ]:
        if not np.allclose(object_points[:, 2], 0.0, atol=1e-6):
            return None, None, math.nan, math.nan, {}
        base_xy = object_points[:, :2].reshape(-1, 1, 2)
        image_xy = image_points.reshape(-1, 1, 2)
        ids = tuple(int(value) for value in marker_ids)
        base_control, image_control = self._planar_control_points(
            object_points,
            image_points,
            ids,
            base_xy,
            image_xy,
        )

        homography, _mask = cv2.findHomography(
            base_control,
            image_control,
            method=0,
        )
        if homography is None or not np.all(np.isfinite(homography)):
            raise RuntimeError("Failed to calculate the base-plane homography")

        control_rmse, _control_errors = self._perspective_rmse(
            base_control,
            image_control,
            homography,
        )
        rmse, errors = self._perspective_rmse(
            base_xy,
            image_xy,
            homography,
        )
        return (
            homography,
            np.linalg.inv(homography),
            rmse,
            control_rmse,
            self._marker_errors(errors, ids),
        )

    def _planar_control_points(
        self,
        object_points: np.ndarray,
        image_points: np.ndarray,
        marker_ids: Sequence[int],
        base_xy: np.ndarray,
        image_xy: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        ids = tuple(marker_ids)
        points_per_marker = len(object_points) // len(ids) if ids else 0
        use_marker_centers = (
            self.planar_fit_mode == "marker_centers"
            and len(ids) >= 4
            and points_per_marker == 4
            and points_per_marker * len(ids) == len(object_points)
        )
        if use_marker_centers:
            base_control = np.asarray(
                [
                    self._quadrilateral_center(
                        object_points[index * 4 : (index + 1) * 4, :2]
                    )
                    for index in range(len(ids))
                ],
                dtype=np.float64,
            ).reshape(-1, 1, 2)
            image_control = np.asarray(
                [
                    self._quadrilateral_center(
                        image_points[index * 4 : (index + 1) * 4]
                    )
                    for index in range(len(ids))
                ],
                dtype=np.float64,
            ).reshape(-1, 1, 2)
            return base_control, image_control
        return base_xy, image_xy

    @staticmethod
    def _perspective_rmse(
        source_points: np.ndarray,
        expected_points: np.ndarray,
        homography: np.ndarray,
    ) -> tuple[float, np.ndarray]:
        projected = cv2.perspectiveTransform(
            source_points,
            homography,
        ).reshape(-1, 2)
        errors = np.linalg.norm(
            projected - expected_points.reshape(-1, 2),
            axis=1,
        )
        return float(np.sqrt(np.mean(errors * errors))), errors

    @staticmethod
    def _quadrilateral_center(points: np.ndarray) -> np.ndarray:
        corners = np.asarray(points, dtype=np.float64).reshape(4, 2)
        first_direction = corners[2] - corners[0]
        second_direction = corners[3] - corners[1]
        matrix = np.column_stack((first_direction, -second_direction))
        determinant = float(np.linalg.det(matrix))
        if abs(determinant) <= 1e-9:
            raise RuntimeError("Marker diagonals are degenerate")
        parameters = np.linalg.solve(matrix, corners[1] - corners[0])
        return corners[0] + parameters[0] * first_direction

    @staticmethod
    def _marker_errors(
        point_errors: np.ndarray,
        marker_ids: Sequence[int],
    ) -> dict[int, float]:
        ids = tuple(int(value) for value in marker_ids)
        if not ids or len(point_errors) % len(ids) != 0:
            return {}
        points_per_marker = len(point_errors) // len(ids)
        return {
            marker_id: float(
                np.sqrt(
                    np.mean(
                        point_errors[
                            index * points_per_marker : (index + 1) * points_per_marker
                        ]
                        ** 2
                    )
                )
            )
            for index, marker_id in enumerate(ids)
        }

    @staticmethod
    def _format_marker_errors(marker_errors: Mapping[int, float]) -> str:
        if not marker_errors:
            return "unavailable"
        return ", ".join(
            f"ID {marker_id}={error:.2f}px"
            for marker_id, error in sorted(marker_errors.items())
        )

    def _positive_float_config(self, name: str, default: float) -> float:
        value = float(self.board_config.get(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"board.{name} must be a positive finite number")
        return value

    def _positive_int_config(self, name: str, default: int) -> int:
        value = int(self.board_config.get(name, default))
        if value <= 0:
            raise ValueError(f"board.{name} must be a positive integer")
        return value

    @staticmethod
    def _transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
        homogeneous = np.column_stack(
            [np.asarray(points, dtype=np.float64), np.ones(len(points), dtype=np.float64)]
        )
        transformed = (np.asarray(transform, dtype=np.float64) @ homogeneous.T).T
        return transformed[:, :3]
