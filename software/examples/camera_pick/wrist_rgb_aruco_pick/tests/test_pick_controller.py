from __future__ import annotations

import sys
import unittest
from dataclasses import dataclass
from io import StringIO
from pathlib import Path
from contextlib import redirect_stdout


ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from aruco_vision import ArucoPoseDetection
from config_loader import parse_config
from pick_controller import WristArucoPickController


def valid_config(max_cycles: int | None = 1, *, wait_for_user: bool = False):
    return parse_config(
        {
            "camera": {
                "index": 0,
                "width": 640,
                "height": 480,
                "camera_matrix": [[800.0, 0.0, 320.0], [0.0, 800.0, 240.0], [0.0, 0.0, 1.0]],
                "dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
            },
            "marker": {
                "dictionary": "DICT_4X4_50",
                "id": 0,
                "size_mm": 30.0,
            },
            "observe_pose": {
                "x_mm": 180.0,
                "y_mm": 0.0,
                "z_mm": 80.0,
                "rx_deg": 0.0,
                "ry_deg": -30.0,
                "rz_deg": 0.0,
            },
            "place_pose": {
                "x_mm": 160.0,
                "y_mm": -90.0,
                "z_mm": 30.0,
                "rx_deg": 0.0,
                "ry_deg": 0.0,
                "rz_deg": 0.0,
            },
            "mapping": {
                "reference_camera_xy_mm": [0.0, 0.0],
                "camera_xy_to_base_xy": [[0.0, -1.0], [-1.0, 0.0]],
                "grasp_offset_base_xy_mm": [0.0, 0.0],
                "max_xy_offset_mm": 80.0,
            },
            "task": {
                "safe_z_mm": 60.0,
                "grasp_z_mm": -60.0,
                "grasp_pause_s": 0.0,
            },
            "loop": {"max_cycles": max_cycles, "wait_for_user": wait_for_user},
        }
    )


@dataclass
class FakeResult:
    accepted: bool
    message: str = "ok"


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
        command_count = sum(1 for call in self.calls if call[0] in ("set_gripper", "move_ik", "reset"))
        accepted = self.fail_at_call is None or command_count != self.fail_at_call
        return FakeResult(accepted=accepted, message="forced failure" if not accepted else "ok")


class FakeVision:
    def __init__(self, detections: list[ArucoPoseDetection] | None = None) -> None:
        self.detections = detections or [
            ArucoPoseDetection(
                detected=True,
                usable=True,
                marker_id=0,
                tvec_mm=(10.0, 20.0, 100.0),
                reason="ok",
            )
        ]
        self.detect_count = 0

    def detect(self, _frame):
        detection = self.detections[min(self.detect_count, len(self.detections) - 1)]
        self.detect_count += 1
        return detection


class PickControllerTests(unittest.TestCase):
    def _controller(
        self,
        *,
        arm: FakeArm | None = None,
        max_cycles: int | None = 1,
        wait_for_user: bool = False,
        vision: FakeVision | None = None,
        preview_fn=None,
    ) -> WristArucoPickController:
        return WristArucoPickController(
            valid_config(max_cycles, wait_for_user=wait_for_user),
            arm_port="FAKE",
            arm=arm,
            frame_reader=lambda: object(),
            vision=vision or FakeVision(),
            show_preview=preview_fn is not None,
            preview_fn=preview_fn,
            input_fn=lambda _prompt: "q",
            sleep_fn=lambda _seconds: None,
        )

    def test_dry_run_does_not_connect_or_move_arm(self) -> None:
        arm = FakeArm()
        controller = self._controller(arm=arm)
        output = StringIO()

        with redirect_stdout(output):
            results = controller.run(dry_run=True)

        self.assertEqual(len(results), 1)
        self.assertEqual(arm.calls, [])
        self.assertIn("Plan observe_pose", output.getvalue())
        self.assertIn("Plan place_pose", output.getvalue())

    def test_preview_callback_receives_detection_before_dry_run_plan(self) -> None:
        previews = []

        def preview(frame, detection, _plan, _status):
            previews.append((frame, detection))
            return ord(" ")

        controller = self._controller(preview_fn=preview)

        results = controller.run(dry_run=True)

        self.assertEqual(len(results), 1)
        self.assertEqual(len(previews), 1)
        self.assertIs(previews[0][1], results[0].detection)

    def test_ready_target_waits_for_space_before_planning(self) -> None:
        keys = iter((-1, ord(" ")))
        vision = FakeVision(
            [
                ArucoPoseDetection(detected=True, usable=True, marker_id=0, tvec_mm=(10.0, 0.0, 100.0), reason="ok"),
                ArucoPoseDetection(detected=True, usable=True, marker_id=0, tvec_mm=(20.0, 0.0, 100.0), reason="ok"),
            ]
        )
        controller = self._controller(
            vision=vision,
            preview_fn=lambda _frame, _detection, _plan, _status: next(keys),
        )

        results = controller.run(dry_run=True)

        self.assertEqual(len(results), 1)
        self.assertEqual(vision.detect_count, 2)
        self.assertEqual(results[0].detection.tvec_mm, (20.0, 0.0, 100.0))

    def test_execute_one_cycle_orders_grasp_place_and_observe(self) -> None:
        arm = FakeArm()
        controller = self._controller(arm=arm)

        results = controller.run()

        self.assertEqual(len(results), 1)
        command_names = [call[0] for call in arm.calls]
        self.assertEqual(
            command_names,
            [
                "connect",
                "move_ik",
                "set_gripper",
                "move_ik",
                "move_ik",
                "set_gripper",
                "move_ik",
                "move_ik",
                "move_ik",
                "set_gripper",
                "move_ik",
                "reset",
                "close",
            ],
        )
        self.assertEqual(arm.calls[1][1:4], (180.0, 0.0, 80.0))
        self.assertEqual(arm.calls[1][4:7], (0.0, -30.0, 0.0))
        for call_index in (3, 4, 6):
            self.assertEqual(arm.calls[call_index][4:7], (0.0, 0.0, 0.0))
        self.assertEqual(arm.calls[-3][1:4], (180.0, 0.0, 80.0))
        self.assertTrue(arm.closed)

    def test_multiple_cycles_re_detect_instead_of_reusing_old_target(self) -> None:
        arm = FakeArm()
        vision = FakeVision(
            [
                ArucoPoseDetection(detected=True, usable=True, marker_id=0, tvec_mm=(10.0, 0.0, 100.0), reason="ok"),
                ArucoPoseDetection(detected=True, usable=True, marker_id=0, tvec_mm=(20.0, 0.0, 100.0), reason="ok"),
            ]
        )
        controller = self._controller(arm=arm, max_cycles=2, vision=vision)

        results = controller.run()

        self.assertEqual(len(results), 2)
        self.assertEqual(vision.detect_count, 2)
        self.assertNotEqual(results[0].plan.target_y_mm, results[1].plan.target_y_mm)

    def test_real_run_keeps_waiting_for_space_until_preview_exit(self) -> None:
        arm = FakeArm()
        keys = [ord(" "), ord("q")]

        def preview(_frame, _detection, _plan, _status):
            if keys:
                return keys.pop(0)
            return ord("q")

        controller = self._controller(
            arm=arm,
            max_cycles=None,
            wait_for_user=False,
            preview_fn=preview,
        )

        results = controller.run()

        self.assertEqual(len(results), 1)
        self.assertEqual(arm.calls[-2][0], "reset")
        self.assertTrue(arm.closed)

    def test_preview_exit_before_pick_resets_connected_arm(self) -> None:
        arm = FakeArm()
        controller = self._controller(
            arm=arm,
            max_cycles=None,
            preview_fn=lambda _frame, _detection, _plan, _status: ord("q"),
        )

        results = controller.run()

        self.assertEqual(results, [])
        self.assertEqual([call[0] for call in arm.calls], ["connect", "move_ik", "reset", "close"])

    def test_user_can_stop_after_first_cycle(self) -> None:
        arm = FakeArm()
        controller = self._controller(arm=arm, max_cycles=None, wait_for_user=True)

        results = controller.run()

        self.assertEqual(len(results), 1)

    def test_unusable_detection_retries_until_target_is_ready(self) -> None:
        vision = FakeVision(
            [
                ArucoPoseDetection(detected=False, usable=False, marker_id=None, reason="no_marker"),
                ArucoPoseDetection(detected=True, usable=False, marker_id=0, reason="reprojection_error_too_high"),
                ArucoPoseDetection(detected=True, usable=True, marker_id=0, tvec_mm=(10.0, 20.0, 100.0), reason="ok"),
            ]
        )
        controller = self._controller(
            vision=vision,
            preview_fn=lambda _frame, _detection, _plan, _status: ord(" "),
        )

        results = controller.run(dry_run=True)

        self.assertEqual(len(results), 1)
        self.assertEqual(vision.detect_count, 3)
        self.assertTrue(results[0].detection.usable)

    def test_unsafe_offset_retries_until_target_is_in_range(self) -> None:
        vision = FakeVision(
            [
                ArucoPoseDetection(detected=True, usable=True, marker_id=0, tvec_mm=(100.0, 100.0, 100.0), reason="ok"),
                ArucoPoseDetection(detected=True, usable=True, marker_id=0, tvec_mm=(10.0, 20.0, 100.0), reason="ok"),
            ]
        )
        controller = self._controller(
            vision=vision,
            preview_fn=lambda _frame, _detection, _plan, _status: ord(" "),
        )

        results = controller.run(dry_run=True)

        self.assertEqual(len(results), 1)
        self.assertEqual(vision.detect_count, 2)
        self.assertEqual(results[0].detection.tvec_mm, (10.0, 20.0, 100.0))

    def test_sdk_failure_stops_following_commands(self) -> None:
        arm = FakeArm(fail_at_call=3)
        controller = self._controller(arm=arm)

        with self.assertRaisesRegex(RuntimeError, "move_to_target_safe"):
            controller.run()

        commands = [call for call in arm.calls if call[0] in ("set_gripper", "move_ik")]
        self.assertEqual(len(commands), 3)


if __name__ == "__main__":
    unittest.main()
