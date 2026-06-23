from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

try:
    import cv2
    import numpy as np
except ModuleNotFoundError:  # pragma: no cover - depends on local env
    cv2 = None
    np = None

from aruco_vision import ArucoPoseDetection, ArucoVision


@unittest.skipIf(cv2 is None or not hasattr(cv2, "aruco"), "opencv-contrib-python is not installed")
class ArucoVisionTests(unittest.TestCase):
    def _vision(self, marker_id: int = 0, max_error: float = 4.0) -> ArucoVision:
        return ArucoVision(
            {
                "camera_matrix": [[800.0, 0.0, 160.0], [0.0, 800.0, 120.0], [0.0, 0.0, 1.0]],
                "dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
            },
            {
                "dictionary": "DICT_4X4_50",
                "id": marker_id,
                "size_mm": 30.0,
                "max_reprojection_error_px": max_error,
            },
        )

    def _marker_frame(self, marker_id: int = 0):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 100)
        frame = np.full((240, 320, 3), 255, dtype=np.uint8)
        frame[70:170, 110:210] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        return frame

    def test_detects_target_marker_and_outputs_pose(self) -> None:
        detection = self._vision().detect(self._marker_frame())

        self.assertTrue(detection.detected)
        self.assertTrue(detection.usable)
        self.assertEqual(detection.marker_id, 0)
        self.assertEqual(len(detection.corners_px), 4)
        self.assertIsNotNone(detection.tvec_mm)
        self.assertIsNotNone(detection.reprojection_error_px)

    def test_non_target_id_reports_failure(self) -> None:
        detection = self._vision(marker_id=1).detect(self._marker_frame(marker_id=0))

        self.assertFalse(detection.detected)
        self.assertFalse(detection.usable)
        self.assertIn("target_id_not_found", detection.reason)

    def test_high_reprojection_error_is_not_usable(self) -> None:
        detection = self._vision(max_error=0.000001).detect(self._marker_frame())

        self.assertTrue(detection.detected)
        self.assertFalse(detection.usable)
        self.assertEqual(detection.reason, "reprojection_error_too_high")

    def test_solve_pnp_failure_reports_reason(self) -> None:
        with patch("aruco_vision.cv2.solvePnP", return_value=(False, None, None)):
            detection = self._vision().detect(self._marker_frame())

        self.assertTrue(detection.detected)
        self.assertFalse(detection.usable)
        self.assertEqual(detection.reason, "solve_pnp_failed")
        self.assertIsNone(detection.tvec_mm)

    def test_to_dict_is_structured(self) -> None:
        payload = ArucoPoseDetection(
            detected=True,
            usable=True,
            marker_id=0,
            corners_px=((1.0, 2.0), (3.0, 4.0)),
            center_px=(2.0, 3.0),
            rvec=(0.0, 0.0, 0.0),
            tvec_mm=(1.0, 2.0, 3.0),
            distance_mm=4.0,
            reprojection_error_px=0.5,
            reason="ok",
        ).to_dict()

        self.assertEqual(payload["marker_id"], 0)
        self.assertEqual(payload["center_px"], [2.0, 3.0])
        self.assertEqual(payload["reason"], "ok")


if __name__ == "__main__":
    unittest.main()
