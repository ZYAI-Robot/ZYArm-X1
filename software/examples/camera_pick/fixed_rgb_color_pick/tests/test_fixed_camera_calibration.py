from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

try:
    import cv2
except ModuleNotFoundError:  # pragma: no cover - depends on local env
    cv2 = None


FIXED_RGB_COLOR_PICK_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = FIXED_RGB_COLOR_PICK_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

if cv2 is not None:
    from color_block_vision import ColorBlockVision
    from fixed_camera_calibration import CalibrationResult, FixedCameraCalibrator
    from fixed_color_pick_controller import DEFAULT_CONFIG_PATH, load_fixed_rgb_color_pick_config


CAMERA_MATRIX = np.array(
    [
        [800.0, 0.0, 320.0],
        [0.0, 800.0, 240.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

MARKERS = {
    0: [[-100.0, 100.0, 0.0], [-80.0, 100.0, 0.0], [-80.0, 80.0, 0.0], [-100.0, 80.0, 0.0]],
    1: [[80.0, 100.0, 0.0], [100.0, 100.0, 0.0], [100.0, 80.0, 0.0], [80.0, 80.0, 0.0]],
    2: [[80.0, -80.0, 0.0], [100.0, -80.0, 0.0], [100.0, -100.0, 0.0], [80.0, -100.0, 0.0]],
    3: [[-100.0, -80.0, 0.0], [-80.0, -80.0, 0.0], [-80.0, -100.0, 0.0], [-100.0, -100.0, 0.0]],
}


def _make_calibrator() -> FixedCameraCalibrator:
    return FixedCameraCalibrator(
        {
            "width": 640,
            "height": 480,
            "camera_matrix": CAMERA_MATRIX,
            "dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
        },
        {
            "dictionary": "DICT_4X4_50",
            "origin_base_mm": [0.0, 0.0, 0.0],
            "marker_corners_base_mm": MARKERS,
        },
    )


def _make_rotated_board_calibrator() -> FixedCameraCalibrator:
    return FixedCameraCalibrator(
        {
            "width": 640,
            "height": 480,
            "camera_matrix": CAMERA_MATRIX,
            "dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
        },
        {
            "dictionary": "DICT_4X4_50",
            "board_to_base_xy": [[0.0, 1.0], [-1.0, 0.0]],
            "origin_base_mm": [0.0, 0.0, 0.0],
            "marker_corners_base_mm": MARKERS,
        },
    )


class FakeDetector:
    def detectMarkers(self, _gray: np.ndarray):
        corners = [
            np.zeros((1, 4, 2), dtype=np.float32),
            np.zeros((1, 4, 2), dtype=np.float32),
        ]
        ids = np.array([[0], [1]], dtype=np.int32)
        return corners, ids, []


@unittest.skipIf(cv2 is None, "OpenCV is required for fixed RGB camera calibration tests")
class FixedCameraCalibrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.calibrator = _make_calibrator()
        self.object_points = np.concatenate(
            [np.asarray(MARKERS[marker_id], dtype=np.float64) for marker_id in range(4)],
            axis=0,
        )
        self.rvec_camera_base = np.array([[np.pi], [0.0], [0.0]], dtype=np.float64)
        self.tvec_camera_base = np.array([[0.0], [0.0], [500.0]], dtype=np.float64)
        projected, _jacobian = cv2.projectPoints(
            self.object_points,
            self.rvec_camera_base,
            self.tvec_camera_base,
            CAMERA_MATRIX,
            np.zeros(5, dtype=np.float64),
        )
        self.image_points = projected.reshape(-1, 2)

    def test_known_pose_recovers_transform_direction(self) -> None:
        result = self.calibrator.calibrate_points(
            self.object_points,
            self.image_points,
        )
        expected_rotation, _jacobian = cv2.Rodrigues(self.rvec_camera_base)
        expected = np.eye(4, dtype=np.float64)
        expected[:3, :3] = expected_rotation
        expected[:3, 3] = self.tvec_camera_base.reshape(3)

        np.testing.assert_allclose(result.t_camera_base, expected, atol=1e-6)
        np.testing.assert_allclose(
            result.t_base_camera,
            np.linalg.inv(expected),
            atol=1e-6,
        )
        self.assertLess(result.reprojection_error_px, 1e-6)

    def test_center_pixel_intersects_base_origin(self) -> None:
        result = self.calibrator.calibrate_points(
            self.object_points,
            self.image_points,
        )
        point_base = self.calibrator.pixel_to_base((320.0, 240.0), result=result)
        np.testing.assert_allclose(point_base, np.zeros(3), atol=1e-6)

    def test_image_axis_converts_to_base_yaw(self) -> None:
        result = self.calibrator.calibrate_points(
            self.object_points,
            self.image_points,
        )
        yaw = self.calibrator.direction_to_base(
            ((320.0, 240.0), (400.0, 240.0)),
            result=result,
        )
        self.assertAlmostEqual(yaw, 0.0, delta=1e-6)

    def test_board_coordinates_are_rotated_into_robot_flu_base(self) -> None:
        calibrator = _make_rotated_board_calibrator()

        np.testing.assert_allclose(
            calibrator.marker_points[0][0],
            np.array([100.0, 100.0, 0.0]),
            atol=1e-9,
        )
        np.testing.assert_allclose(
            calibrator.marker_points[1][0],
            np.array([100.0, -80.0, 0.0]),
            atol=1e-9,
        )

    def test_board_to_base_rotation_updates_position_and_yaw_together(self) -> None:
        calibrator = _make_rotated_board_calibrator()
        object_points_base = np.concatenate(
            [calibrator.marker_points[marker_id] for marker_id in range(4)],
            axis=0,
        )
        image_points, _jacobian = cv2.projectPoints(
            object_points_base,
            self.rvec_camera_base,
            self.tvec_camera_base,
            CAMERA_MATRIX,
            np.zeros(5, dtype=np.float64),
        )
        result = calibrator.calibrate_points(
            object_points_base,
            image_points.reshape(-1, 2),
        )

        target_base = np.array([[258.660, 102.702, 0.0]], dtype=np.float64)
        target_pixel, _jacobian = cv2.projectPoints(
            target_base,
            self.rvec_camera_base,
            self.tvec_camera_base,
            CAMERA_MATRIX,
            np.zeros(5, dtype=np.float64),
        )
        mapped = calibrator.pixel_to_base(
            target_pixel.reshape(2),
            result=result,
        )

        board_x_direction_base = np.array(
            [[258.660, 102.702, 0.0], [258.660, 2.702, 0.0]],
            dtype=np.float64,
        )
        direction_pixels, _jacobian = cv2.projectPoints(
            board_x_direction_base,
            self.rvec_camera_base,
            self.tvec_camera_base,
            CAMERA_MATRIX,
            np.zeros(5, dtype=np.float64),
        )
        yaw = calibrator.direction_to_base(
            direction_pixels.reshape(2, 2),
            result=result,
        )

        np.testing.assert_allclose(mapped, target_base[0], atol=1e-6)
        self.assertAlmostEqual(yaw, 90.0, delta=1e-6)

    def test_missing_marker_ids_stop_calibration(self) -> None:
        self.calibrator._detector = FakeDetector()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        with self.assertRaisesRegex(RuntimeError, "Missing required"):
            self.calibrator.detect_board(frame)

    def test_stable_detection_combines_markers_across_frames(self) -> None:
        self.calibrator.stable_samples_per_marker = 3
        observations = []
        for frame_index in range(6):
            marker_ids = (0, 2) if frame_index % 2 == 0 else (1, 3)
            observation = {}
            for marker_id in marker_ids:
                points = self.image_points[marker_id * 4 : marker_id * 4 + 4]
                observation[marker_id] = points + (frame_index - 2.5) * 0.02
            observations.append(observation)

        detection = self.calibrator.build_stable_detection(observations)

        self.assertEqual(detection.marker_ids, (0, 1, 2, 3))
        self.assertEqual(detection.image_points_px.shape, (16, 2))

    def test_observations_are_retained_for_full_capture_window(self) -> None:
        self.calibrator.stable_samples_per_marker = 3
        observations = []
        for marker_id in range(4):
            points = self.image_points[marker_id * 4 : marker_id * 4 + 4]
            for _sample_index in range(3):
                self.calibrator.append_observation(
                    observations,
                    {marker_id: points},
                )
            for _gap_index in range(25):
                other_id = (marker_id + 1) % 4
                other_points = self.image_points[
                    other_id * 4 : other_id * 4 + 4
                ]
                self.calibrator.append_observation(
                    observations,
                    {other_id: other_points},
                )

        counts = self.calibrator.observation_counts(observations)
        detection = self.calibrator.build_stable_detection(observations)

        self.assertGreater(len(observations), 100)
        self.assertTrue(all(count >= 3 for count in counts.values()))
        self.assertEqual(detection.marker_ids, (0, 1, 2, 3))

    def test_stable_window_is_not_rejected_by_later_cumulative_jitter(self) -> None:
        self.calibrator.stable_samples_per_marker = 6
        self.calibrator.max_corner_jitter_px = 1.5
        observations = []

        for sample_index in range(30):
            observation = {}
            for marker_id in range(4):
                points = self.image_points[marker_id * 4 : marker_id * 4 + 4]
                if sample_index < 6:
                    offset = 0.1 * (sample_index - 2.5)
                else:
                    offset = 2.0 * ((sample_index % 2) * 2 - 1)
                observation[marker_id] = points + offset
            observations.append(observation)

        all_samples = np.stack(
            [observation[1] for observation in observations],
            axis=0,
        )
        all_center = np.median(all_samples, axis=0)
        cumulative_jitter = float(
            np.sqrt(np.mean(np.sum((all_samples - all_center) ** 2, axis=2)))
        )
        detection = self.calibrator.build_stable_detection(observations)

        self.assertGreater(cumulative_jitter, self.calibrator.max_corner_jitter_px)
        self.assertEqual(detection.marker_ids, (0, 1, 2, 3))

    def test_marker_without_any_stable_window_remains_unstable(self) -> None:
        self.calibrator.stable_samples_per_marker = 6
        self.calibrator.max_corner_jitter_px = 1.5
        observations = []

        for sample_index in range(12):
            observation = {}
            for marker_id in range(4):
                points = self.image_points[marker_id * 4 : marker_id * 4 + 4]
                offset = 0.0
                if marker_id == 1:
                    offset = 3.0 * ((sample_index % 2) * 2 - 1)
                observation[marker_id] = points + offset
            observations.append(observation)

        with self.assertRaisesRegex(RuntimeError, r"unstable=\['1:"):
            self.calibrator.build_stable_detection(observations)

    def test_capture_timeout_reports_never_detected_marker(self) -> None:
        self.calibrator.max_capture_frames = 4
        visible = {
            marker_id: self.image_points[marker_id * 4 : marker_id * 4 + 4]
            for marker_id in (0, 2, 3)
        }
        self.calibrator.detect_visible_markers = lambda _frame: visible

        with self.assertRaisesRegex(RuntimeError, r"never detected IDs=\[1\]"):
            self.calibrator.calibrate_stable(
                lambda: np.zeros((480, 640, 3), dtype=np.uint8)
            )

    def test_plane_mapping_remains_accurate_with_imperfect_intrinsics(self) -> None:
        wrong_intrinsics_calibrator = FixedCameraCalibrator(
            {
                "width": 640,
                "height": 480,
                "camera_matrix": [
                    [780.0, 0.0, 315.0],
                    [0.0, 820.0, 245.0],
                    [0.0, 0.0, 1.0],
                ],
                "dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
            },
            {
                "dictionary": "DICT_4X4_50",
                "origin_base_mm": [0.0, 0.0, 0.0],
                "marker_corners_base_mm": MARKERS,
            },
        )
        result = wrong_intrinsics_calibrator.calibrate_points(
            self.object_points,
            self.image_points,
        )
        center_base = wrong_intrinsics_calibrator.pixel_to_base(
            (320.0, 240.0),
            result=result,
        )

        self.assertGreater(result.reprojection_error_px, 2.0)
        self.assertLess(result.planar_reprojection_error_px, 1e-6)
        np.testing.assert_allclose(center_base, np.zeros(3), atol=1e-6)

    def test_marker_center_fit_tolerates_printed_marker_size_error(self) -> None:
        actual_points = self.object_points.copy()
        for marker_index in range(4):
            marker_slice = slice(marker_index * 4, marker_index * 4 + 4)
            center = np.mean(actual_points[marker_slice], axis=0)
            actual_points[marker_slice] = (
                center + 0.75 * (actual_points[marker_slice] - center)
            )
        actual_image_points, _jacobian = cv2.projectPoints(
            actual_points,
            self.rvec_camera_base,
            self.tvec_camera_base,
            CAMERA_MATRIX,
            np.zeros(5, dtype=np.float64),
        )

        result = self.calibrator.calibrate_points(
            self.object_points,
            actual_image_points.reshape(-1, 2),
        )
        origin_base = self.calibrator.pixel_to_base(
            (320.0, 240.0),
            result=result,
        )

        self.assertEqual(result.planar_fit_mode, "marker_centers")
        self.assertLess(result.planar_control_error_px, 1e-6)
        self.assertGreater(result.planar_reprojection_error_px, 2.0)
        np.testing.assert_allclose(origin_base, np.zeros(3), atol=1e-6)

    def test_placeholder_config_is_rejected_with_fill_hint(self) -> None:
        with self.assertRaisesRegex(ValueError, "Fill camera.camera_matrix"):
            FixedCameraCalibrator(
                {"camera_matrix": None, "dist_coeffs": None},
                {
                    "dictionary": None,
                    "origin_base_mm": [0.0, 0.0, 0.0],
                    "marker_corners_base_mm": {0: None, 1: None, 2: None, 3: None},
                },
            )

    def test_parallel_ray_is_rejected(self) -> None:
        result = CalibrationResult(
            t_camera_base=np.eye(4),
            t_base_camera=np.array(
                [
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [-1.0, 0.0, 0.0, 500.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            ),
            rvec_camera_base=np.zeros((3, 1)),
            tvec_camera_base=np.zeros((3, 1)),
            reprojection_error_px=0.0,
            object_points_base_mm=self.object_points,
            image_points_px=self.image_points,
            marker_ids=(0, 1, 2, 3),
        )
        with self.assertRaisesRegex(RuntimeError, "parallel"):
            self.calibrator.pixel_to_base((320.0, 240.0), result=result)


def run_live_debug() -> int:
    camera, board, _task = load_fixed_rgb_color_pick_config(DEFAULT_CONFIG_PATH)
    vision = ColorBlockVision(camera)
    calibrator = FixedCameraCalibrator(camera, board)
    try:
        vision.open()
        window_name = "Fixed RGB camera calibration"
        cv2.namedWindow(window_name)
        observations: list[dict[int, np.ndarray]] = []
        while True:
            frame = vision.read_frame()
            visible = calibrator.detect_visible_markers(frame)
            if visible:
                calibrator.append_observation(observations, visible)
            try:
                detection = calibrator.build_stable_detection(observations)
                result = calibrator.calibrate_points(
                    detection.object_points_base_mm,
                    detection.image_points_px,
                    marker_ids=detection.marker_ids,
                )
                break
            except RuntimeError as exc:
                preview = frame.copy()
                if visible:
                    marker_ids = sorted(visible)
                    cv2.aruco.drawDetectedMarkers(
                        preview,
                        [
                            visible[marker_id].reshape(1, 4, 2).astype(np.float32)
                            for marker_id in marker_ids
                        ],
                        np.asarray(marker_ids, dtype=np.int32).reshape(-1, 1),
                    )
                counts = calibrator.observation_counts(observations)
                cv2.putText(
                    preview,
                    str(exc),
                    (12, 32),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 165, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    preview,
                    f"stable samples: {counts}",
                    (12, 64),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 165, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    preview,
                    "Hold camera/board still; press q or Esc to exit",
                    (12, 96),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 165, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow(window_name, preview)
                key = cv2.waitKey(20) & 0xFF
                if key in (27, ord("q")):
                    return 1

        overlay = calibrator.draw_result(frame, detection, result)
        display = overlay.copy()
        clicked_point: list[tuple[int, int] | None] = [None]

        def on_mouse(event, x, y, _flags, _parameter) -> None:
            if event != cv2.EVENT_LBUTTONDOWN:
                return
            point_base = calibrator.pixel_to_base((x, y), result=result)
            clicked_point[0] = (x, y)
            print(
                f"pixel=({x}, {y}) -> base_link="
                f"({point_base[0]:.3f}, {point_base[1]:.3f}, {point_base[2]:.3f}) mm"
            )

        print("T_base_camera:")
        print(result.t_base_camera)
        print(f"Reprojection RMSE: {result.reprojection_error_px:.4f}px")
        print(f"Plane center RMSE: {result.planar_control_error_px:.4f}px")
        print(f"Plane corner model RMSE: {result.planar_reprojection_error_px:.4f}px")
        print(f"Plane per-marker corner RMSE: {result.planar_marker_errors_px}")
        if (
            result.planar_fit_mode == "marker_centers"
            and result.planar_reprojection_error_px
            > calibrator.max_planar_reprojection_error_px
        ):
            print(
                "Warning: loose-marker corner geometry does not exactly match the "
                "configuration. XY conversion uses marker centers."
            )
        if result.reprojection_error_px > calibrator.PNP_WARNING_ERROR_PX:
            print(
                "Warning: PnP error suggests approximate camera intrinsics/distortion. "
                "Plane clicks use the directly fitted marker homography."
            )
        print("Click an image point to print its base_link coordinate; press q or Esc to exit.")
        cv2.setMouseCallback(window_name, on_mouse)
        while True:
            display[:] = overlay
            if clicked_point[0] is not None:
                cv2.circle(display, clicked_point[0], 5, (0, 255, 255), -1)
            cv2.imshow(window_name, display)
            key = cv2.waitKey(20) & 0xFF
            if key in (27, ord("q")):
                return 0
    finally:
        vision.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    if "--camera" in sys.argv:
        raise SystemExit(run_live_debug())
    unittest.main()
