from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Optional

try:
    import cv2
except ModuleNotFoundError as exc:  # pragma: no cover - depends on local env
    cv2 = None
    _OPENCV_IMPORT_ERROR = exc
else:
    _OPENCV_IMPORT_ERROR = None

from aruco_vision import ArucoPoseDetection, ArucoVision
from camera_stream import CameraStream
from config_loader import WristArucoConfig
from pose_mapping import PickPlan, TargetOffsetError, map_marker_to_target_xy


REPOSITORY_ROOT = Path(__file__).resolve().parents[5]
SDK_SRC = REPOSITORY_ROOT / "software" / "zyarm_sdk" / "python" / "src"


@dataclass(frozen=True)
class CycleResult:
    detection: ArucoPoseDetection
    plan: PickPlan


class PickCancelled(RuntimeError):
    """Raised when the user exits from the preview window."""


class WristArucoPickController:
    GRIPPER_OPEN = 1.0
    GRIPPER_CLOSED = 0.0
    DETECTION_RETRY_PAUSE_S = 0.05

    def __init__(
        self,
        config: WristArucoConfig,
        *,
        arm_port: Optional[str] = None,
        arm: Any = None,
        arm_factory: Optional[Callable[[str], Any]] = None,
        frame_reader: Optional[Callable[[], Any]] = None,
        vision: Optional[ArucoVision] = None,
        show_preview: bool = False,
        preview_wait_ms: int = 500,
        preview_fn: Optional[Callable[[Any, ArucoPoseDetection, Optional[PickPlan], str], Optional[int]]] = None,
        input_fn: Callable[[str], str] = input,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> None:
        self.config = config
        self.arm_port = arm_port
        self.arm = arm
        self._arm_factory = arm_factory or self._create_sdk_arm
        self._frame_reader = frame_reader
        self._show_preview = show_preview
        self._preview_wait_ms = preview_wait_ms
        self._preview_fn = preview_fn
        self._preview_window_open = False
        self._input = input_fn
        self._sleep = sleep_fn
        self.vision = vision or ArucoVision(config.camera, config.marker)
        self._camera_stream: Optional[CameraStream] = None

    def dry_run_once(self) -> CycleResult:
        detection, plan = self.observe_until_plan()
        self.print_plan(plan)
        return CycleResult(detection=detection, plan=plan)

    def run(self, *, dry_run: bool = False) -> list[CycleResult]:
        results: list[CycleResult] = []
        try:
            cycle_index = 0
            at_observe_pose = False
            while True:
                if self.config.loop.max_cycles is not None and cycle_index >= self.config.loop.max_cycles:
                    return results
                if dry_run:
                    try:
                        results.append(self.dry_run_once())
                    except PickCancelled:
                        return results
                    return results

                self.connect()
                if not at_observe_pose:
                    self.move_to_observe()
                    at_observe_pose = True
                try:
                    detection, plan = self.observe_until_plan()
                except PickCancelled:
                    return results
                self.print_plan(plan)
                exit_requested = self.execute_plan_with_live_preview(plan)
                at_observe_pose = True
                results.append(CycleResult(detection=detection, plan=plan))
                cycle_index += 1

                if exit_requested:
                    return results
                if self.config.loop.max_cycles is not None and cycle_index >= self.config.loop.max_cycles:
                    return results
                if self.config.loop.wait_for_user and not self._continue_after_cycle():
                    return results
        finally:
            self.close(close_arm=not dry_run)

    def connect(self) -> None:
        if self.arm is not None and self.arm.is_connected:
            return
        if self.arm is None:
            if not self.arm_port:
                raise ValueError("arm_port is required when dry-run is not enabled")
            self.arm = self._arm_factory(self.arm_port)
        if not self.arm.is_connected:
            connected = self.arm.connect()
            if connected is not None:
                self.arm = connected
        if not self.arm.is_connected:
            raise RuntimeError(f"Failed to connect ZYArm on port {self.arm_port}")

    def move_to_observe(self) -> None:
        pose = self.config.observe_pose
        self._send_ik("move_to_observe", *pose.as_move_ik_args())

    def observe_until_plan(self) -> tuple[ArucoPoseDetection, PickPlan]:
        last_wait_reason: Optional[str] = None
        while True:
            frame = self.read_frame()
            detection = self.vision.detect(frame)
            ready, reason = self._detection_ready(detection)
            plan = None
            if ready:
                try:
                    plan = self.build_plan(detection)
                except TargetOffsetError as exc:
                    reason = str(exc)
            key = self._show_detection_preview(frame, detection, plan, reason)
            if key in (27, ord("q"), ord("Q")):
                raise PickCancelled("Pick cancelled from preview window before motion")
            if plan is not None:
                if key == ord(" "):
                    return detection, plan
                reason = "usable target detected; press SPACE to execute pick"
            if reason != last_wait_reason:
                print(f"Waiting for usable ArUco target: {reason}")
                last_wait_reason = reason
            self._sleep(self.DETECTION_RETRY_PAUSE_S)

    def build_plan(self, detection: ArucoPoseDetection) -> PickPlan:
        ready, reason = self._detection_ready(detection)
        if not ready:
            raise RuntimeError(reason)
        target_x, target_y, delta_camera, delta_base = map_marker_to_target_xy(
            detection.tvec_mm,
            observe_pose=self.config.observe_pose,
            mapping=self.config.mapping,
        )
        task = self.config.task
        return PickPlan(
            target_x_mm=target_x,
            target_y_mm=target_y,
            safe_z_mm=task.safe_z_mm,
            grasp_z_mm=task.grasp_z_mm,
            target_rx_deg=0.0,
            target_ry_deg=0.0,
            target_rz_deg=0.0,
            delta_camera_xy_mm=delta_camera,
            delta_base_xy_mm=delta_base,
        )

    def _detection_ready(self, detection: ArucoPoseDetection) -> tuple[bool, str]:
        if not detection.detected:
            return False, f"ArUco target was not detected: {detection.reason}"
        if self.config.safety.require_marker_usable and not detection.usable:
            return False, f"ArUco pose is not usable: {detection.reason}"
        if detection.tvec_mm is None:
            return False, "ArUco detection is missing tvec_mm"
        return True, ""

    def execute_plan(self, plan: PickPlan) -> None:
        task = self.config.task
        place = self.config.place_pose

        self._require_command(
            "open_gripper",
            self.arm.set_gripper(self.GRIPPER_OPEN, sync=False),
        )
        self._send_ik(
            "move_to_target_safe",
            plan.target_x_mm,
            plan.target_y_mm,
            plan.safe_z_mm,
            plan.target_rx_deg,
            plan.target_ry_deg,
            plan.target_rz_deg,
        )
        self._send_ik(
            "descend_to_grasp",
            plan.target_x_mm,
            plan.target_y_mm,
            plan.grasp_z_mm,
            plan.target_rx_deg,
            plan.target_ry_deg,
            plan.target_rz_deg,
        )
        self._require_command(
            "close_gripper",
            self.arm.set_gripper(self.GRIPPER_CLOSED, sync=False),
        )
        if task.grasp_pause_s > 0.0:
            self._sleep(task.grasp_pause_s)
        self._send_ik(
            "lift_from_grasp",
            plan.target_x_mm,
            plan.target_y_mm,
            plan.safe_z_mm,
            plan.target_rx_deg,
            plan.target_ry_deg,
            plan.target_rz_deg,
        )
        self._send_ik(
            "move_to_place_safe",
            place.x_mm,
            place.y_mm,
            plan.safe_z_mm,
            place.rx_deg,
            place.ry_deg,
            place.rz_deg,
        )
        self._send_ik("move_to_place", *place.as_move_ik_args())
        self._require_command(
            "release_gripper",
            self.arm.set_gripper(self.GRIPPER_OPEN, sync=True),
        )
        self.move_to_observe()

    def execute_plan_with_live_preview(self, plan: PickPlan) -> bool:
        if not self._show_preview:
            self.execute_plan(plan)
            return False

        motion_errors: list[BaseException] = []

        def run_motion() -> None:
            try:
                self.execute_plan(plan)
            except BaseException as exc:
                motion_errors.append(exc)

        motion_thread = threading.Thread(
            target=run_motion,
            name="zyarm-wrist-aruco-pick-place",
            daemon=False,
        )
        motion_thread.start()

        exit_requested = False
        exit_notice_printed = False
        preview_error: Optional[BaseException] = None
        try:
            while motion_thread.is_alive():
                frame = self.read_frame()
                detection = self.vision.detect(frame)
                status = (
                    "Motion in progress - exit requested"
                    if exit_requested
                    else "Motion in progress - q/Esc: exit after motion"
                )
                key = self._show_detection_preview(frame, detection, plan, status, wait_ms=1)
                if key in (27, ord("q"), ord("Q")):
                    exit_requested = True
                    if not exit_notice_printed:
                        print("Exit requested; waiting for the current robot motion to finish")
                        exit_notice_printed = True
        except BaseException as exc:
            preview_error = exc
        finally:
            motion_thread.join()

        if motion_errors:
            raise motion_errors[0]
        if preview_error is not None:
            raise preview_error
        return exit_requested

    def print_plan(self, plan: PickPlan) -> None:
        observe = self.config.observe_pose
        place = self.config.place_pose
        print(
            "Aruco delta camera xy: "
            f"({plan.delta_camera_xy_mm[0]:.3f}, {plan.delta_camera_xy_mm[1]:.3f}) mm"
        )
        print(
            "Aruco delta base xy: "
            f"({plan.delta_base_xy_mm[0]:.3f}, {plan.delta_base_xy_mm[1]:.3f}) mm"
        )
        print(
            "Plan observe_pose: "
            f"move_ik({observe.x_mm:.3f}, {observe.y_mm:.3f}, {observe.z_mm:.3f}, "
            f"{observe.rx_deg:.3f}, {observe.ry_deg:.3f}, {observe.rz_deg:.3f})"
        )
        print(
            "Plan target safe: "
            f"move_ik({plan.target_x_mm:.3f}, {plan.target_y_mm:.3f}, {plan.safe_z_mm:.3f}, "
            f"{plan.target_rx_deg:.3f}, {plan.target_ry_deg:.3f}, {plan.target_rz_deg:.3f})"
        )
        print(
            "Plan place_pose: "
            f"move_ik({place.x_mm:.3f}, {place.y_mm:.3f}, {place.z_mm:.3f}, "
            f"{place.rx_deg:.3f}, {place.ry_deg:.3f}, {place.rz_deg:.3f})"
        )

    def read_frame(self) -> Any:
        if self._frame_reader is not None:
            return self._frame_reader()
        self._require_opencv()
        if self._camera_stream is None:
            self._camera_stream = CameraStream(self.config.camera, cv2_module=cv2)
        return self._camera_stream.read_frame()

    def close(self, *, close_arm: bool = True) -> None:
        if self._camera_stream is not None:
            self._camera_stream.close()
            self._camera_stream = None
        if self._preview_window_open and cv2 is not None:
            try:
                cv2.destroyWindow("Wrist RGB ArUco Pick")
            except cv2.error:
                pass
            self._preview_window_open = False
        if close_arm and self.arm is not None:
            try:
                if self.arm.is_connected:
                    self.reset_to_home()
            finally:
                self.arm.close()

    def reset_to_home(self) -> None:
        print("Arm command [reset_to_home]: reset()")
        self._require_command("reset_to_home", self.arm.reset())

    def _continue_after_cycle(self) -> bool:
        answer = self._input("Cycle complete. Press Enter for next cycle, or q + Enter to exit: ")
        return answer.strip().lower() not in {"q", "quit", "exit"}

    def _show_detection_preview(
        self,
        frame: Any,
        detection: ArucoPoseDetection,
        plan: Optional[PickPlan] = None,
        status: str = "",
        wait_ms: Optional[int] = None,
    ) -> int:
        if self._preview_fn is not None:
            key = self._preview_fn(frame, detection, plan, status)
            return -1 if key is None else int(key)
        if not self._show_preview:
            return ord(" ")
        self._require_opencv()
        overlay = self.vision.draw_overlay(frame, detection)
        self._draw_pick_status(overlay, plan, status)
        cv2.imshow("Wrist RGB ArUco Pick", overlay)
        self._preview_window_open = True
        delay_ms = self._preview_wait_ms if wait_ms is None else wait_ms
        key = cv2.waitKey(max(0, int(delay_ms))) & 0xFF
        return key

    @staticmethod
    def _draw_pick_status(overlay: Any, plan: Optional[PickPlan], status: str) -> None:
        if cv2 is None:
            return
        green = (0, 220, 0)
        yellow = (0, 220, 220)
        color = green if plan is not None else yellow
        text = status
        if plan is not None and not text:
            text = "Target ready - press SPACE to pick; q/Esc to exit"
        if text:
            cv2.putText(
                overlay,
                text,
                (12, 82),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2,
                cv2.LINE_AA,
            )
        if plan is not None:
            cv2.putText(
                overlay,
                f"target=({plan.target_x_mm:.1f}, {plan.target_y_mm:.1f}, {plan.safe_z_mm:.1f})",
                (12, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                green,
                2,
                cv2.LINE_AA,
            )

    def _send_ik(
        self,
        stage: str,
        x: float,
        y: float,
        z: float,
        rx: float,
        ry: float,
        rz: float,
    ) -> None:
        print(
            f"IK command [{stage}]: "
            f"move_ik({x:.3f}, {y:.3f}, {z:.3f}, {rx:.3f}, {ry:.3f}, {rz:.3f})"
        )
        self._require_command(stage, self.arm.move_ik(x, y, z, rx, ry, rz))

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

    @staticmethod
    def _require_opencv() -> None:
        if cv2 is None:
            raise RuntimeError(
                "Missing dependency OpenCV. Install it with `python -m pip install opencv-contrib-python`."
            ) from _OPENCV_IMPORT_ERROR
