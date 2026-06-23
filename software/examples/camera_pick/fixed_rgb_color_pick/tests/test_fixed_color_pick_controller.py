from __future__ import annotations

import sys
import threading
import unittest
from contextlib import redirect_stdout
from dataclasses import dataclass
from io import StringIO
from pathlib import Path
from unittest.mock import patch

import numpy as np

try:
    import cv2 as _cv2  # noqa: F401
except ModuleNotFoundError:  # pragma: no cover - depends on local env
    _HAS_OPENCV = False
else:
    _HAS_OPENCV = True


FIXED_RGB_COLOR_PICK_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = FIXED_RGB_COLOR_PICK_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

if _HAS_OPENCV:
    from color_block_vision import BlockDetection, BlockDetectionTimeout
    from fixed_color_pick_controller import (
        DEFAULT_CONFIG_PATH,
        FixedColorPickController,
        PickTarget,
        load_fixed_rgb_color_pick_config,
        main,
    )


@dataclass
class FakeResult:
    accepted: bool
    message: str = "ok"


@dataclass
class FakeCalibrationResult:
    planar_reprojection_error_px: float = 0.0


class FakeArm:
    def __init__(self, *, fail_at_call: int | None = None) -> None:
        self.is_connected = False
        self.closed = False
        self.calls: list[tuple] = []
        self.fail_at_call = fail_at_call

    def connect(self):
        self.is_connected = True
        self.calls.append(("connect",))
        return self

    def close(self) -> None:
        self.closed = True
        self.is_connected = False
        self.calls.append(("close",))

    def set_gripper(self, position: float, *, sync: bool = False) -> FakeResult:
        self.calls.append(("set_gripper", position, sync))
        return self._result()

    def move_ik(self, *values: float) -> FakeResult:
        self.calls.append(("move_ik", *values))
        return self._result()

    def reset(self) -> FakeResult:
        self.calls.append(("reset",))
        return self._result()

    def _result(self) -> FakeResult:
        command_count = sum(
            1
            for call in self.calls
            if call[0] in ("set_gripper", "move_ik", "reset")
        )
        accepted = self.fail_at_call is None or command_count != self.fail_at_call
        return FakeResult(accepted=accepted, message="forced failure" if not accepted else "ok")


class FakeVision:
    def __init__(self) -> None:
        self.opened = False
        self.closed = False
        self.frame = np.zeros((10, 10, 3), dtype=np.uint8)
        self.roi_count = 0
        self.detect_count = 0
        self.detection = BlockDetection(
            detected=True,
            stable=True,
            graspable=True,
            center_px=(100.0, 120.0),
            axis_points_px=((90.0, 120.0), (110.0, 120.0)),
            angle_deg=0.0,
            area_px=500.0,
        )

    def open(self) -> None:
        self.opened = True

    def read_frame(self):
        return self.frame.copy()

    def select_color_roi(self, _frame) -> None:
        self.roi_count += 1

    def detect_stable(self) -> BlockDetection:
        self.detect_count += 1
        return self.detection

    def close(self) -> None:
        self.closed = True


class FakeCalibrator:
    def __init__(self) -> None:
        self.result = FakeCalibrationResult()
        self.calibrate_count = 0
        self.max_planar_reprojection_error_px = 2.0

    def calibrate_stable(self, read_frame):
        read_frame()
        self.calibrate_count += 1
        return None, self.result

    def pixel_to_base(self, _point, *, result):
        assert result is self.result
        return np.array([200.0, 30.0, 0.0], dtype=np.float64)

    def direction_to_base(self, _points, *, result):
        assert result is self.result
        return 15.0


@unittest.skipUnless(_HAS_OPENCV, "OpenCV is required for fixed RGB pick controller tests")
class PickControllerTests(unittest.TestCase):
    def _controller(
        self,
        *,
        arm: FakeArm | None = None,
        safe_z: float | None = 60.0,
        sleep_fn=None,
    ) -> FixedColorPickController:
        arm = arm or FakeArm()
        return FixedColorPickController(
            {},
            {},
            {
                "safe_z_mm": safe_z,
                "approach_z_mm": 0.0,
                "grasp_z_mm": -80.0,
                "approach_pause_s": 1.0,
                "place_x_mm": 120.0,
                "place_y_mm": -150.0,
            },
            arm_port="FAKE",
            vision=FakeVision(),
            calibrator=FakeCalibrator(),
            arm=arm,
            sleep_fn=sleep_fn or (lambda seconds: arm.calls.append(("sleep", seconds))),
        )

    def test_run_once_orders_sdk_commands_and_closes_resources(self) -> None:
        arm = FakeArm()
        controller = self._controller(arm=arm)
        target = controller.run_once()

        self.assertEqual(target, PickTarget(200.0, 30.0, 0.0, 15.0))
        command_names = [call[0] for call in arm.calls]
        self.assertEqual(
            command_names,
            [
                "connect",
                "set_gripper",
                "move_ik",
                "sleep",
                "move_ik",
                "move_ik",
                "set_gripper",
                "sleep",
                "move_ik",
                "move_ik",
                "move_ik",
                "set_gripper",
                "move_ik",
                "reset",
                "close",
            ],
        )
        self.assertEqual(arm.calls[6], ("set_gripper", 0.0, False))
        self.assertEqual(arm.calls[7], ("sleep", 2.0))
        self.assertEqual(arm.calls[11], ("set_gripper", 1.0, True))
        self.assertEqual(arm.calls[12][0], "move_ik")
        self.assertEqual(arm.calls[13], ("reset",))
        self.assertTrue(controller.vision.closed)
        self.assertTrue(arm.closed)

    def test_run_reuses_color_and_calibration_for_next_cycle(self) -> None:
        arm = FakeArm()
        controller = self._controller(arm=arm)
        decisions = iter((True, False))
        controller.wait_for_next_cycle = lambda: next(decisions)
        controller.execute_pick_with_live_preview = (
            lambda target: (controller.execute_pick(target), False)[1]
        )

        targets = controller.run()

        self.assertEqual(len(targets), 2)
        self.assertEqual(controller.vision.roi_count, 1)
        self.assertEqual(controller.calibrator.calibrate_count, 1)
        self.assertEqual(controller.vision.detect_count, 2)
        self.assertEqual(sum(call[0] == "connect" for call in arm.calls), 1)
        self.assertEqual(sum(call[0] == "reset" for call in arm.calls), 2)
        self.assertTrue(controller.vision.closed)
        self.assertTrue(arm.closed)

    def test_run_returns_to_wait_after_recognition_timeout(self) -> None:
        arm = FakeArm()
        controller = self._controller(arm=arm)
        detections = iter(
            (
                BlockDetectionTimeout("not stable"),
                controller.vision.detection,
            )
        )

        def detect_stable():
            result = next(detections)
            if isinstance(result, Exception):
                raise result
            return result

        controller.vision.detect_stable = detect_stable
        decisions = iter((True, False))
        controller.wait_for_next_cycle = lambda: next(decisions)
        controller.execute_pick_with_live_preview = (
            lambda target: (controller.execute_pick(target), False)[1]
        )

        targets = controller.run()

        self.assertEqual(targets, [PickTarget(200.0, 30.0, 0.0, 15.0)])
        self.assertEqual(controller.calibrator.calibrate_count, 1)
        self.assertEqual(sum(call[0] == "reset" for call in arm.calls), 1)

    def test_wait_for_next_cycle_maps_keys_and_delays_after_space(self) -> None:
        arm = FakeArm()
        controller = self._controller(arm=arm)

        with (
            patch("fixed_color_pick_controller.cv2.destroyAllWindows"),
            patch("fixed_color_pick_controller.cv2.destroyWindow"),
            patch("fixed_color_pick_controller.cv2.imshow"),
            patch(
                "fixed_color_pick_controller.cv2.waitKey",
                side_effect=(ord(" "), ord("q")),
            ),
        ):
            self.assertTrue(controller.wait_for_next_cycle())
            self.assertFalse(controller.wait_for_next_cycle())

        self.assertIn(("sleep", controller.NEXT_CYCLE_DELAY_S), arm.calls)

    def test_motion_runs_in_worker_while_main_thread_updates_preview(self) -> None:
        controller = self._controller()
        motion_started = threading.Event()
        release_motion = threading.Event()
        motion_thread_ids: list[int] = []
        main_thread_id = threading.get_ident()

        def execute_pick(_target) -> None:
            motion_thread_ids.append(threading.get_ident())
            motion_started.set()
            self.assertTrue(release_motion.wait(timeout=1.0))

        def wait_key(_delay: int) -> int:
            self.assertTrue(motion_started.is_set())
            release_motion.set()
            return -1

        controller.execute_pick = execute_pick
        with (
            patch("fixed_color_pick_controller.cv2.destroyAllWindows"),
            patch("fixed_color_pick_controller.cv2.destroyWindow"),
            patch("fixed_color_pick_controller.cv2.imshow") as imshow,
            patch("fixed_color_pick_controller.cv2.waitKey", side_effect=wait_key),
        ):
            exit_requested = controller.execute_pick_with_live_preview(
                PickTarget(200.0, 30.0, 0.0, 15.0)
            )

        self.assertFalse(exit_requested)
        self.assertEqual(len(motion_thread_ids), 1)
        self.assertNotEqual(motion_thread_ids[0], main_thread_id)
        self.assertTrue(imshow.called)

    def test_motion_preview_defers_quit_until_worker_finishes(self) -> None:
        controller = self._controller()
        release_motion = threading.Event()

        def execute_pick(_target) -> None:
            self.assertTrue(release_motion.wait(timeout=1.0))

        def wait_key(_delay: int) -> int:
            release_motion.set()
            return ord("q")

        controller.execute_pick = execute_pick
        with (
            patch("fixed_color_pick_controller.cv2.destroyAllWindows"),
            patch("fixed_color_pick_controller.cv2.destroyWindow"),
            patch("fixed_color_pick_controller.cv2.imshow"),
            patch("fixed_color_pick_controller.cv2.waitKey", side_effect=wait_key),
        ):
            exit_requested = controller.execute_pick_with_live_preview(
                PickTarget(200.0, 30.0, 0.0, 15.0)
            )

        self.assertTrue(exit_requested)

    def test_motion_worker_error_is_raised_on_main_thread(self) -> None:
        controller = self._controller()

        def execute_pick(_target) -> None:
            raise RuntimeError("forced motion failure")

        controller.execute_pick = execute_pick
        with self.assertRaisesRegex(RuntimeError, "forced motion failure"):
            controller.execute_pick_with_live_preview(
                PickTarget(200.0, 30.0, 0.0, 15.0)
            )

    def test_sdk_failure_stops_following_commands(self) -> None:
        arm = FakeArm(fail_at_call=2)
        controller = self._controller(arm=arm)
        with self.assertRaisesRegex(RuntimeError, "move_to_safe_z"):
            controller.execute_pick(PickTarget(200.0, 30.0, 0.0, 15.0))

        commands = [
            call
            for call in arm.calls
            if call[0] in ("set_gripper", "move_ik", "reset")
        ]
        self.assertEqual(len(commands), 2)

    def test_execute_pick_prints_exact_ik_commands(self) -> None:
        controller = self._controller()
        output = StringIO()

        with redirect_stdout(output):
            controller.execute_pick(PickTarget(200.0, 30.0, 0.0, 15.0))

        self.assertIn(
            "IK command [move_to_safe_z]: "
            "move_ik(200.000, 30.000, 60.000, 0.000, 0.000, 15.000)",
            output.getvalue(),
        )
        self.assertIn(
            "IK command [descend_to_approach]: "
            "move_ik(200.000, 30.000, 0.000, 0.000, 0.000, 15.000)",
            output.getvalue(),
        )
        self.assertIn(
            "IK command [descend_to_grasp]: "
            "move_ik(200.000, 30.000, -80.000, 0.000, 0.000, 15.000)",
            output.getvalue(),
        )
        self.assertIn(
            "IK command [lift_block_to_approach]: "
            "move_ik(200.000, 30.000, 0.000, 0.000, 0.000, 15.000)",
            output.getvalue(),
        )
        self.assertIn(
            "IK command [move_to_place]: "
            "move_ik(120.000, -150.000, 0.000, 0.000, 0.000, 15.000)",
            output.getvalue(),
        )
        self.assertIn(
            "IK command [descend_to_place]: "
            "move_ik(120.000, -150.000, -80.000, 0.000, 0.000, 15.000)",
            output.getvalue(),
        )
        self.assertIn(
            "IK command [lift_from_place]: "
            "move_ik(120.000, -150.000, 0.000, 0.000, 0.000, 15.000)",
            output.getvalue(),
        )
        self.assertIn("Pause after move_to_safe_z: 1.000s", output.getvalue())
        self.assertIn("Arm command [reset_to_home]: reset()", output.getvalue())

    def test_fixed_rgb_color_pick_config_contains_tunable_pick_heights(self) -> None:
        _camera, _board, task = load_fixed_rgb_color_pick_config(DEFAULT_CONFIG_PATH)

        values = {
            name: float(task[name])
            for name in (
                "safe_z_mm",
                "approach_z_mm",
                "grasp_z_mm",
                "approach_pause_s",
                "place_x_mm",
                "place_y_mm",
            )
        }
        self.assertTrue(all(np.isfinite(value) for value in values.values()))
        self.assertGreater(values["safe_z_mm"], values["approach_z_mm"])
        self.assertGreater(values["approach_z_mm"], values["grasp_z_mm"])
        self.assertGreaterEqual(values["approach_pause_s"], 0.0)

    def test_missing_safe_z_stops_before_motion(self) -> None:
        arm = FakeArm()
        controller = self._controller(arm=arm, safe_z=None)
        detection = controller.vision.detection
        with self.assertRaisesRegex(ValueError, "Fill task.safe_z_mm"):
            controller.build_pick_target(detection, controller.calibrator.result)
        self.assertEqual(arm.calls, [])

    def test_invalid_task_height_order_stops_before_connecting(self) -> None:
        arm = FakeArm()
        controller = self._controller(arm=arm)
        controller.task_config["grasp_z_mm"] = 10.0

        with self.assertRaisesRegex(ValueError, "safe_z_mm > approach_z_mm"):
            controller.execute_pick(PickTarget(200.0, 30.0, 0.0, 15.0))

        self.assertEqual(arm.calls, [])

    def test_missing_place_coordinate_stops_before_connecting(self) -> None:
        arm = FakeArm()
        controller = self._controller(arm=arm)
        controller.task_config["place_x_mm"] = None

        with self.assertRaisesRegex(ValueError, "Fill task.place_x_mm"):
            controller.execute_pick(PickTarget(200.0, 30.0, 0.0, 15.0))

        self.assertEqual(arm.calls, [])

    def test_ungraspable_detection_stops_before_coordinate_conversion(self) -> None:
        controller = self._controller()
        detection = BlockDetection(
            detected=True,
            stable=False,
            graspable=False,
            reject_reason="target_not_stable",
        )
        with self.assertRaisesRegex(RuntimeError, "target_not_stable"):
            controller.build_pick_target(detection, controller.calibrator.result)


if __name__ == "__main__":
    if "--port" in sys.argv:
        raise SystemExit(main())
    unittest.main()
