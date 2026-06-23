from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from camera_stream import CameraStream


class FakeCv2:
    CAP_DSHOW = 700
    CAP_PROP_FOURCC = 1
    CAP_PROP_FRAME_WIDTH = 2
    CAP_PROP_FRAME_HEIGHT = 3
    CAP_PROP_FPS = 4
    CAP_PROP_AUTO_EXPOSURE = 5
    CAP_PROP_EXPOSURE = 6
    CAP_PROP_BUFFERSIZE = 7

    @staticmethod
    def VideoWriter_fourcc(*value: str) -> int:
        return sum(ord(item) << (8 * index) for index, item in enumerate(value))


class FakeCapture:
    def __init__(self) -> None:
        self.released = False
        self.read_count = 0
        self.set_calls: list[tuple[int, float]] = []

    def isOpened(self) -> bool:
        return True

    def read(self):
        self.read_count += 1
        return True, {"frame": self.read_count}

    def set(self, prop: int, value: float) -> bool:
        self.set_calls.append((prop, value))
        return True

    def release(self) -> None:
        self.released = True


class CameraStreamTests(unittest.TestCase):
    def test_windows_open_uses_directshow_stream_params(self) -> None:
        capture = FakeCapture()
        capture_args = []

        def capture_factory(*args):
            capture_args.append(args)
            return capture

        stream = CameraStream(
            {
                "index": 1,
                "width": 1920,
                "height": 1080,
                "fourcc": "MJPG",
                "fps": 30,
                "warmup_frames": 1,
                "buffersize": 1,
            },
            cv2_module=FakeCv2,
            capture_factory=capture_factory,
        )

        frame = stream.read_frame()

        self.assertEqual(frame, {"frame": 1})
        expected_args = (
            (
                1,
                FakeCv2.CAP_DSHOW,
                [
                    FakeCv2.CAP_PROP_FOURCC,
                    FakeCv2.VideoWriter_fourcc(*"MJPG"),
                    FakeCv2.CAP_PROP_FRAME_WIDTH,
                    1920,
                    FakeCv2.CAP_PROP_FRAME_HEIGHT,
                    1080,
                    FakeCv2.CAP_PROP_FPS,
                    30,
                ],
            )
            if sys.platform == "win32"
            else (1,)
        )
        self.assertEqual(capture_args, [expected_args])
        self.assertEqual(capture.read_count, 1)
        self.assertIn((FakeCv2.CAP_PROP_BUFFERSIZE, 1), capture.set_calls)
        stream.close()
        self.assertTrue(capture.released)

    def test_invalid_fourcc_is_rejected_before_open(self) -> None:
        factory_calls = []
        stream = CameraStream(
            {"index": 1, "fourcc": "TOO_LONG"},
            cv2_module=FakeCv2,
            capture_factory=lambda *args: factory_calls.append(args),
        )

        with self.assertRaisesRegex(ValueError, "camera.fourcc"):
            stream.open()

        self.assertEqual(factory_calls, [])


if __name__ == "__main__":
    unittest.main()
