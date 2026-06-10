from __future__ import annotations

import argparse
import importlib.util
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Optional

import cv2

from block_vision import (
    BlockDetection,
    BlockDetectionCancelled,
    BlockDetectionTimeout,
    BlockVision,
)
from handeye_calibration import CalibrationResult, HandEyeCalibrator


HAND_EYE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = HAND_EYE_ROOT.parents[3]
DEFAULT_CONFIG_PATH = HAND_EYE_ROOT / "config" / "handeye.py"
SDK_SRC = REPOSITORY_ROOT / "software" / "zyarm_sdk" / "python" / "src"


@dataclass(frozen=True)
class PickTarget:
    x_mm: float
    y_mm: float
    z_mm: float
    yaw_deg: float


class ZYArmPickController:
    """Orchestrate one calibration and repeated pick-and-place cycles."""

    GRASP_RX_DEG = 0.0
    GRASP_RY_DEG = 0.0
    GRIPPER_OPEN = 1.0
    GRIPPER_CLOSED = 0.0
    NEXT_CYCLE_DELAY_S = 0.5
    NEXT_CYCLE_WINDOW = "ZYArm ready for next block"
    MOTION_WINDOW = "ZYArm motion preview"

    def __init__(
        self,
        camera_config: Mapping[str, Any],
        board_config: Mapping[str, Any],
        task_config: Mapping[str, Any],
        *,
        arm_port: str,
        vision: Optional[BlockVision] = None,
        calibrator: Optional[HandEyeCalibrator] = None,
        arm: Any = None,
        arm_factory: Optional[Callable[[str], Any]] = None,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> None:
        if not arm_port:
            raise ValueError("arm_port must not be empty")
        camera = dict(camera_config)
        board = dict(board_config)
        self.task_config = dict(task_config)
        self.arm_port = arm_port
        self.vision = vision or BlockVision(camera)
        self.calibrator = calibrator or HandEyeCalibrator(
            camera,
            board,
        )
        self.arm = arm
        self._arm_factory = arm_factory or self._create_sdk_arm
        self._sleep = sleep_fn

    def connect(self) -> None:
        if self.arm is not None and self.arm.is_connected:
            return
        if self.arm is None:
            self.arm = self._arm_factory(self.arm_port)
        if not self.arm.is_connected:
            connected = self.arm.connect()
            if connected is not None:
                self.arm = connected
        if not self.arm.is_connected:
            raise RuntimeError(f"Failed to connect ZYArm on port {self.arm_port}")

    def calibrate_camera_stable(self) -> CalibrationResult:
        _detection, result = self.calibrator.calibrate_stable(self.vision.read_frame)
        if (
            result.planar_reprojection_error_px
            > self.calibrator.max_planar_reprojection_error_px
        ):
            print(
                "Calibration warning: marker centers are usable, but the configured "
                "marker corner geometry differs from the image "
                f"({result.planar_reprojection_error_px:.3f}px RMSE). "
                "Measure marker center positions for best absolute accuracy."
            )
        return result

    def locate_block(self) -> BlockDetection:
        detection = self.vision.detect_stable()
        if not detection.graspable:
            raise RuntimeError(detection.reject_reason or "Target block is not graspable")
        return detection

    def build_pick_target(
        self,
        detection: BlockDetection,
        calibration: CalibrationResult,
    ) -> PickTarget:
        if not detection.graspable:
            raise RuntimeError(detection.reject_reason or "Target block is not graspable")
        if detection.center_px is None or detection.axis_points_px is None:
            raise RuntimeError("Block detection is missing center or direction points")

        safe_z = self._task_float("safe_z_mm")
        if safe_z <= 0.0:
            raise ValueError("task.safe_z_mm must be positive")

        center_base = self.calibrator.pixel_to_base(
            detection.center_px,
            result=calibration,
        )
        yaw_deg = self.calibrator.direction_to_base(
            detection.axis_points_px,
            result=calibration,
        )
        if safe_z <= float(center_base[2]):
            raise ValueError("task.safe_z_mm must be above the pick plane")

        return PickTarget(
            x_mm=float(center_base[0]),
            y_mm=float(center_base[1]),
            z_mm=float(center_base[2]),
            yaw_deg=float(yaw_deg),
        )

    def execute_pick(self, target: PickTarget) -> None:
        (
            safe_z,
            approach_z,
            grasp_z,
            approach_pause_s,
            place_x,
            place_y,
        ) = self._validated_task_parameters()

        self.connect()
        self._execute_grasp(
            target,
            safe_z=safe_z,
            approach_z=approach_z,
            grasp_z=grasp_z,
            approach_pause_s=approach_pause_s,
        )
        self._execute_place(
            target,
            approach_z=approach_z,
            grasp_z=grasp_z,
            place_x=place_x,
            place_y=place_y,
        )

    def _execute_grasp(
        self,
        target: PickTarget,
        *,
        safe_z: float,
        approach_z: float,
        grasp_z: float,
        approach_pause_s: float,
    ) -> None:
        self._require_command(
            "open_gripper",
            self.arm.set_gripper(self.GRIPPER_OPEN, sync=False),
        )
        self._send_ik(
            "move_to_safe_z",
            target.x_mm,
            target.y_mm,
            safe_z,
            target.yaw_deg,
        )
        if approach_pause_s > 0.0:
            print(f"Pause after move_to_safe_z: {approach_pause_s:.3f}s")
            self._sleep(approach_pause_s)
        self._send_ik(
            "descend_to_approach",
            target.x_mm,
            target.y_mm,
            approach_z,
            target.yaw_deg,
        )
        self._send_ik(
            "descend_to_grasp",
            target.x_mm,
            target.y_mm,
            grasp_z,
            target.yaw_deg,
        )
        self._require_command(
            "close_gripper",
            self.arm.set_gripper(self.GRIPPER_CLOSED, sync=False),
        )
        self._sleep(2.0)
        self._send_ik(
            "lift_block_to_approach",
            target.x_mm,
            target.y_mm,
            approach_z,
            target.yaw_deg,
        )

    def _execute_place(
        self,
        target: PickTarget,
        *,
        approach_z: float,
        grasp_z: float,
        place_x: float,
        place_y: float,
    ) -> None:
        self._send_ik(
            "move_to_place",
            place_x,
            place_y,
            approach_z,
            target.yaw_deg,
        )
        self._send_ik(
            "descend_to_place",
            place_x,
            place_y,
            grasp_z,
            target.yaw_deg,
        )
        self._require_command(
            "release_gripper",
            self.arm.set_gripper(self.GRIPPER_OPEN, sync=True),
        )
        self._send_ik(
            "lift_from_place",
            place_x,
            place_y,
            approach_z,
            target.yaw_deg,
        )
        print("Arm command [reset_to_home]: reset()")
        self._require_command("reset_to_home", self.arm.reset())

    def _send_ik(
        self,
        stage: str,
        x: float,
        y: float,
        z: float,
        yaw_deg: float,
    ) -> None:
        print(
            f"IK command [{stage}]: "
            f"move_ik({x:.3f}, {y:.3f}, {z:.3f}, "
            f"{self.GRASP_RX_DEG:.3f}, {self.GRASP_RY_DEG:.3f}, "
            f"{yaw_deg:.3f})"
        )
        self._require_command(
            stage,
            self.arm.move_ik(
                x,
                y,
                z,
                self.GRASP_RX_DEG,
                self.GRASP_RY_DEG,
                yaw_deg,
            ),
        )

    def initialize_session(self) -> CalibrationResult:
        self.vision.open()
        color_frame = self.vision.read_frame()
        self.vision.select_color_roi(color_frame)
        calibration = self.calibrate_camera_stable()
        print("Color model and handeye calibration are ready for this session")
        return calibration

    def recognize_target(self, calibration: CalibrationResult) -> PickTarget:
        detection = self.locate_block()
        target = self.build_pick_target(detection, calibration)

        print(
            "Pick target in base_link: "
            f"x={target.x_mm:.3f}mm, y={target.y_mm:.3f}mm, "
            f"z={target.z_mm:.3f}mm, yaw={target.yaw_deg:.3f}deg"
        )
        return target

    def execute_pick_with_live_preview(self, target: PickTarget) -> bool:
        motion_errors: list[BaseException] = []

        def run_motion() -> None:
            try:
                self.execute_pick(target)
            except BaseException as exc:
                motion_errors.append(exc)

        cv2.destroyAllWindows()
        motion_thread = threading.Thread(
            target=run_motion,
            name="zyarm-pick-place",
            daemon=False,
        )
        motion_thread.start()

        exit_requested = False
        exit_notice_printed = False
        preview_error: Optional[BaseException] = None
        window_shown = False
        try:
            while motion_thread.is_alive():
                frame = self.vision.read_frame()
                status = (
                    "Robot motion in progress - exit requested"
                    if exit_requested
                    else "Robot motion in progress - q/Esc: exit after motion"
                )
                cv2.imshow(
                    self.MOTION_WINDOW,
                    self._status_overlay(frame, status),
                )
                window_shown = True
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    exit_requested = True
                    if not exit_notice_printed:
                        print(
                            "Exit requested; waiting for the current robot motion "
                            "to finish"
                        )
                        exit_notice_printed = True
        except BaseException as exc:
            preview_error = exc
        finally:
            motion_thread.join()
            if window_shown:
                self._destroy_window(self.MOTION_WINDOW)

        if motion_errors:
            raise motion_errors[0]
        if preview_error is not None:
            raise preview_error
        return exit_requested

    def wait_for_next_cycle(self) -> bool:
        cv2.destroyAllWindows()
        print(
            "Cycle complete: move the block, then press SPACE to recognize and "
            "pick again; press q or Esc to exit"
        )

        while True:
            frame = self.vision.read_frame()
            cv2.imshow(
                self.NEXT_CYCLE_WINDOW,
                self._status_overlay(
                    frame,
                    "Move block, then SPACE: pick again    q/Esc: exit",
                ),
            )
            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                self._destroy_window(self.NEXT_CYCLE_WINDOW)
                print(
                    f"Starting next recognition in {self.NEXT_CYCLE_DELAY_S:.1f}s; "
                    "keep hands clear"
                )
                self._sleep(self.NEXT_CYCLE_DELAY_S)
                return True
            if key in (27, ord("q")):
                return False

    def run(self) -> list[PickTarget]:
        targets: list[PickTarget] = []
        try:
            calibration = self.initialize_session()
            while True:
                try:
                    target = self.recognize_target(calibration)
                except BlockDetectionCancelled:
                    return targets
                except BlockDetectionTimeout as exc:
                    print(f"Recognition did not complete: {exc}")
                else:
                    exit_requested = self.execute_pick_with_live_preview(target)
                    targets.append(target)
                    if exit_requested:
                        return targets
                if not self.wait_for_next_cycle():
                    return targets
        finally:
            self.close()

    def run_once(self) -> PickTarget:
        try:
            calibration = self.initialize_session()
            target = self.recognize_target(calibration)
            self.execute_pick(target)
            return target
        finally:
            self.close()

    def close(self) -> None:
        self.vision.close()
        cv2.destroyAllWindows()
        if self.arm is not None:
            self.arm.close()

    @staticmethod
    def _require_command(stage: str, result: Any) -> None:
        if not result.accepted:
            raise RuntimeError(f"ZYArm stage '{stage}' failed: {result.message}")

    @staticmethod
    def _create_sdk_arm(port: str) -> Any:
        if str(SDK_SRC) not in sys.path:
            sys.path.insert(0, str(SDK_SRC))
        from zyarm_sdk import ZyArm, ZyArmConfig

        return ZyArm(ZyArmConfig(port=port))

    def _task_float(self, name: str) -> float:
        value = self.task_config.get(name)
        if value is None:
            raise ValueError(f"Fill task.{name} in config/handeye.py")
        try:
            return float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"task.{name} must be a number") from exc

    def _validated_task_parameters(
        self,
    ) -> tuple[float, float, float, float, float, float]:
        values = (
            self._task_float("safe_z_mm"),
            self._task_float("approach_z_mm"),
            self._task_float("grasp_z_mm"),
            self._task_float("approach_pause_s"),
            self._task_float("place_x_mm"),
            self._task_float("place_y_mm"),
        )
        safe_z, approach_z, grasp_z, approach_pause_s, _place_x, _place_y = values
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Task heights and pause must be finite numbers")
        if approach_pause_s < 0.0:
            raise ValueError("task.approach_pause_s must not be negative")
        if not safe_z > approach_z > grasp_z:
            raise ValueError(
                "Task heights must satisfy safe_z_mm > approach_z_mm > grasp_z_mm"
            )
        return values

    @staticmethod
    def _status_overlay(frame: Any, text: str) -> Any:
        overlay = frame.copy()
        cv2.putText(
            overlay,
            text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 165, 255),
            2,
            cv2.LINE_AA,
        )
        return overlay

    @staticmethod
    def _destroy_window(window_name: str) -> None:
        try:
            cv2.destroyWindow(window_name)
        except cv2.error:
            pass


def load_handeye_config(config_path: Path) -> tuple[Mapping[str, Any], ...]:
    path = config_path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Handeye config does not exist: {path}")

    spec = importlib.util.spec_from_file_location("zyarm_handeye_config", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load handeye config: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    values = []
    for name in ("camera", "board", "task"):
        value = getattr(module, name, None)
        if not isinstance(value, Mapping):
            raise ValueError(f"Config object '{name}' must be a mapping")
        values.append(value)
    return tuple(values)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate a fixed RGB camera and pick one selected color block."
    )
    parser.add_argument(
        "--port",
        required=True,
        help="ZYArm serial port, for example COM3 or /dev/ttyUSB0.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help="Path to config/handeye.py.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    camera, board, task = load_handeye_config(args.config)
    controller = ZYArmPickController(
        camera,
        board,
        task,
        arm_port=args.port,
    )
    controller.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
