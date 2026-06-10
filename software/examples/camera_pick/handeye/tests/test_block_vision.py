from __future__ import annotations

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


HAND_EYE_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = HAND_EYE_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from block_vision import BlockVision
from zyarm_pick_controller import DEFAULT_CONFIG_PATH, load_handeye_config


def _synthetic_frame(
    *,
    center: tuple[int, int] = (160, 120),
    size: tuple[int, int] = (80, 40),
    angle_deg: float = 25.0,
) -> np.ndarray:
    frame = np.full((240, 320, 3), 255, dtype=np.uint8)
    box = cv2.boxPoints((center, size, angle_deg)).astype(np.int32)
    cv2.fillConvexPoly(frame, box, (0, 0, 220))
    return frame


class FakeCapture:
    def __init__(self, frame: np.ndarray, *, fourcc: str = "MJPG") -> None:
        self.frame = frame
        self.fourcc = fourcc
        self.opened = True
        self.released = False
        self.read_count = 0
        self.set_calls: list[tuple[int, float]] = []

    def isOpened(self) -> bool:
        return self.opened

    def set(self, property_id: int, value: float) -> bool:
        self.set_calls.append((property_id, value))
        return True

    def read(self) -> tuple[bool, np.ndarray]:
        self.read_count += 1
        return True, self.frame.copy()

    def get(self, property_id: int) -> float:
        if property_id == cv2.CAP_PROP_FOURCC:
            return float(cv2.VideoWriter_fourcc(*self.fourcc))
        return 0.0

    def release(self) -> None:
        self.released = True
        self.opened = False


class BlockVisionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.frame = _synthetic_frame()
        self.vision = BlockVision({"device": 0, "width": 320, "height": 240})
        self.vision.build_color_model(self.frame, (105, 70, 110, 100))

    def test_roi_color_model_handles_red_hue_wrap(self) -> None:
        model = self.vision.color_model
        self.assertIsNotNone(model)
        assert model is not None
        self.assertEqual(len(model.hue_ranges), 2)

    def test_detect_becomes_stable_after_repeated_frames(self) -> None:
        result = None
        for _index in range(self.vision.STABLE_FRAME_COUNT):
            result, mask = self.vision.detect(self.frame)
        assert result is not None
        self.assertTrue(result.detected)
        self.assertTrue(result.stable)
        self.assertTrue(result.graspable)
        self.assertAlmostEqual(result.center_px[0], 160.0, delta=2.0)
        self.assertAlmostEqual(result.center_px[1], 120.0, delta=2.0)
        self.assertEqual(mask.shape, self.frame.shape[:2])
        self.assertIsNotNone(result.axis_points_px)

    def test_detect_stable_discards_previous_cycle_history(self) -> None:
        for _index in range(self.vision.STABLE_FRAME_COUNT):
            result, _mask = self.vision.detect(self.frame)
        self.assertTrue(result.graspable)

        frames_read = 0

        def read_frame():
            nonlocal frames_read
            frames_read += 1
            return self.frame

        self.vision.read_frame = read_frame
        result = self.vision.detect_stable(show=False)

        self.assertTrue(result.graspable)
        self.assertEqual(frames_read, self.vision.STABLE_FRAME_COUNT)
    def test_largest_valid_candidate_is_selected(self) -> None:
        frame = self.frame.copy()
        cv2.rectangle(frame, (20, 20), (45, 45), (0, 0, 220), -1)
        result, _mask = self.vision.detect(frame)
        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.center_px[0], 160.0, delta=3.0)
        self.assertAlmostEqual(result.center_px[1], 120.0, delta=3.0)

    def test_no_target_returns_reason_without_coordinates(self) -> None:
        blank = np.full_like(self.frame, 255)
        result, mask = self.vision.detect(blank)
        self.assertFalse(result.detected)
        self.assertFalse(result.graspable)
        self.assertIsNone(result.center_px)
        self.assertEqual(result.reject_reason, "no_valid_color_block")
        self.assertEqual(int(np.count_nonzero(mask)), 0)

    def test_square_ignores_min_area_rect_quarter_turn_ambiguity(self) -> None:
        vision = BlockVision({"device": 0, "width": 320, "height": 240})
        first = _synthetic_frame(size=(60, 60), angle_deg=3.0)
        vision.build_color_model(first, (120, 80, 80, 80))

        result = None
        for angle in (3.0, 94.0, 4.0, 95.0, 3.5):
            frame = _synthetic_frame(size=(60, 60), angle_deg=angle)
            result, _mask = vision.detect(frame)

        assert result is not None
        self.assertTrue(result.graspable)
        self.assertEqual(result.angle_deg, 0.0)

    def test_invalid_roi_is_rejected(self) -> None:
        vision = BlockVision({"device": 0})
        with self.assertRaisesRegex(ValueError, "cancelled or is empty"):
            vision.build_color_model(self.frame, (0, 0, 0, 0))

    def test_camera_lifecycle_uses_configured_resolution(self) -> None:
        capture = FakeCapture(self.frame)
        capture_args = []

        def capture_factory(*args):
            capture_args.append(args)
            return capture

        vision = BlockVision(
            {
                "device": 0,
                "width": 320,
                "height": 240,
                "fourcc": "MJPG",
                "fps": 30.0,
                "auto_exposure": False,
                "exposure": -4.0,
            },
            capture_factory=capture_factory,
        )
        vision.open()
        read = vision.read_frame()
        self.assertEqual(read.shape, self.frame.shape)
        expected_args = (
            (
                0,
                cv2.CAP_DSHOW,
                [
                    cv2.CAP_PROP_FOURCC,
                    cv2.VideoWriter_fourcc(*"MJPG"),
                    cv2.CAP_PROP_FRAME_WIDTH,
                    320,
                    cv2.CAP_PROP_FRAME_HEIGHT,
                    240,
                    cv2.CAP_PROP_FPS,
                    30,
                ],
            )
            if sys.platform == "win32"
            else (0,)
        )
        self.assertEqual(capture_args, [expected_args])
        self.assertEqual(capture.read_count, vision.DEFAULT_WARMUP_FRAMES)
        self.assertIn((cv2.CAP_PROP_AUTO_EXPOSURE, 0.25), capture.set_calls)
        self.assertIn((cv2.CAP_PROP_EXPOSURE, -4.0), capture.set_calls)
        vision.close()
        self.assertTrue(capture.released)

    def test_invalid_camera_stream_config_is_rejected_before_open(self) -> None:
        factory_calls = []
        vision = BlockVision(
            {"device": 0, "fourcc": "TOO_LONG", "fps": 30.0},
            capture_factory=lambda *args: factory_calls.append(args),
        )

        with self.assertRaisesRegex(ValueError, "camera.fourcc"):
            vision.open()

        self.assertEqual(factory_calls, [])

    def test_camera_rejects_pixel_format_fallback(self) -> None:
        capture = FakeCapture(self.frame, fourcc="YUY2")
        vision = BlockVision(
            {
                "device": 0,
                "width": 320,
                "height": 240,
                "fourcc": "MJPG",
                "fps": 30.0,
            },
            capture_factory=lambda *_args: capture,
        )

        with self.assertRaisesRegex(RuntimeError, "pixel format mismatch"):
            vision.open()

        self.assertTrue(capture.released)


def run_live_debug() -> int:
    camera, _board, _task = load_handeye_config(DEFAULT_CONFIG_PATH)
    vision = BlockVision(camera)
    try:
        vision.open()
        first_frame = vision.read_frame()
        vision.select_color_roi(first_frame)
        while True:
            frame = vision.read_frame()
            result, mask = vision.detect(frame)
            cv2.imshow("BlockVision", vision.draw_result(frame, result))
            cv2.imshow("BlockVision mask", mask)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                return 0
    finally:
        vision.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    if "--camera" in sys.argv:
        raise SystemExit(run_live_debug())
    unittest.main()
