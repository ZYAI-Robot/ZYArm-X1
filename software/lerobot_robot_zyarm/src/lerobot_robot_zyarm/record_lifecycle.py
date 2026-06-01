from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from math import ceil, inf
from typing import Any, Callable, Optional


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class FreshnessDiagnostic:
    episode_index: int
    leader_action_age_ms: float
    follower_state_age_ms: float
    action_max_age_ms: float
    state_max_age_ms: float
    leader_ok: bool
    follower_ok: bool

    @property
    def ok(self) -> bool:
        return self.leader_ok and self.follower_ok


class ZyArmRecordStop(RuntimeError):
    def __init__(
        self,
        *,
        stage: str,
        episode_index: int,
        reason: str,
        episode_invalid: bool = False,
    ) -> None:
        super().__init__(f"{stage} failed for episode {episode_index}: {reason}")
        self.stage = stage
        self.episode_index = episode_index
        self.reason = reason
        self.episode_invalid = episode_invalid


class ZyArmRecordLifecycle:
    def __init__(
        self,
        *,
        warmup_s: float = 3.0,
        warmup_frames: Optional[int] = None,
        reset_time_s: float = 5.0,
        control_hz: float = 50.0,
        bridge_interpolation_target_speed: float = 1.0,
        bridge_interpolation_min_frames: int = 5,
        bridge_interpolation_max_frames: int = 50,
        bridge_interpolation_threshold: float = 0.15,
        monotonic: Callable[[], float] = time.perf_counter,
        sleep: Callable[[float], None] = time.sleep,
        log: logging.Logger = logger,
    ) -> None:
        if warmup_s < 0:
            raise ValueError("warmup_s must be non-negative")
        if warmup_frames is not None and warmup_frames < 0:
            raise ValueError("warmup_frames must be non-negative")
        if reset_time_s < 0:
            raise ValueError("reset_time_s must be non-negative")
        if control_hz <= 0:
            raise ValueError("control_hz must be positive")
        if bridge_interpolation_target_speed < 0:
            raise ValueError("bridge_interpolation_target_speed must be non-negative")
        if bridge_interpolation_min_frames <= 0:
            raise ValueError("bridge_interpolation_min_frames must be positive")
        if bridge_interpolation_max_frames <= 0:
            raise ValueError("bridge_interpolation_max_frames must be positive")
        if bridge_interpolation_min_frames > bridge_interpolation_max_frames:
            raise ValueError("bridge_interpolation_min_frames must be <= max_frames")
        self.warmup_s = float(warmup_s)
        self.warmup_frames = warmup_frames
        self.reset_time_s = float(reset_time_s)
        self.control_period_s = 1.0 / float(control_hz)
        self.bridge_interpolation_target_speed = float(bridge_interpolation_target_speed)
        self.bridge_interpolation_min_frames = int(bridge_interpolation_min_frames)
        self.bridge_interpolation_max_frames = int(bridge_interpolation_max_frames)
        self.bridge_interpolation_threshold = float(bridge_interpolation_threshold)
        self._monotonic = monotonic
        self._sleep = sleep
        self._log = log

    def run_pre_roll_warmup(
        self,
        *,
        teleop: Any,
        robot: Any,
        episode_index: int,
        finalize: Optional[Callable[[], None]] = None,
    ) -> None:
        stage = "A_PRE_ROLL_WARMUP"
        self._log.info("[A_PRE_ROLL_WARMUP_ENTRY] episode_index=%s", episode_index)
        try:
            if finalize is not None:
                self._log.info("[A_PRE_ROLL_WARMUP_FINALIZE] episode_index=%s", episode_index)
                finalize()
            self._bridge_interpolate_if_needed(
                teleop=teleop,
                robot=robot,
                episode_index=episode_index,
            )
            prepare_episode = getattr(teleop, "prepare_episode", None)
            if callable(prepare_episode):
                prepare_episode()
            if self.warmup_frames is None:
                self._run_follow_for(
                    duration_s=self.warmup_s,
                    teleop=teleop,
                    robot=robot,
                )
            else:
                self._run_follow_frames(
                    frame_count=self.warmup_frames,
                    teleop=teleop,
                    robot=robot,
                )
        except Exception as exc:
            self._stop(stage=stage, episode_index=episode_index, reason=str(exc))

    def run_enter_diagnostic(
        self,
        *,
        teleop: Any,
        robot: Any,
        episode_index: int,
        action_max_age_ms: float,
        state_max_age_ms: float,
    ) -> FreshnessDiagnostic:
        stage = "B_ENTER_DIAGNOSTIC"
        try:
            leader = teleop.get_diagnostic_action()
            follower = robot.refresh_state_for_diagnostics()
            action_limit = inf if action_max_age_ms is None else float(action_max_age_ms)
            state_limit = inf if state_max_age_ms is None else float(state_max_age_ms)
            diagnostic = FreshnessDiagnostic(
                episode_index=episode_index,
                leader_action_age_ms=float(leader.age_ms),
                follower_state_age_ms=float(follower.age_ms),
                action_max_age_ms=action_limit,
                state_max_age_ms=state_limit,
                leader_ok=float(leader.age_ms) <= action_limit,
                follower_ok=float(follower.age_ms) <= state_limit,
            )
            self._log.info(
                "[B_ENTER_FRESHNESS] episode_index=%s leader_action_age_ms=%.1f "
                "follower_state_age_ms=%.1f action_max_age_ms=%.1f "
                "state_max_age_ms=%.1f leader_ok=%s follower_ok=%s",
                diagnostic.episode_index,
                diagnostic.leader_action_age_ms,
                diagnostic.follower_state_age_ms,
                diagnostic.action_max_age_ms,
                diagnostic.state_max_age_ms,
                diagnostic.leader_ok,
                diagnostic.follower_ok,
            )
            if not diagnostic.ok:
                self._stop(
                    stage=stage,
                    episode_index=episode_index,
                    reason=(
                        "freshness diagnostic failed: "
                        f"leader_ok={diagnostic.leader_ok} follower_ok={diagnostic.follower_ok}"
                    ),
                )
            return diagnostic
        except ZyArmRecordStop:
            raise
        except Exception as exc:
            self._stop(stage=stage, episode_index=episode_index, reason=str(exc))

    def record_frame(
        self,
        *,
        teleop: Any,
        robot: Any,
        episode_index: int,
        episode: Any = None,
    ) -> tuple[Any, Any]:
        stage = "B_RECORDING"
        try:
            action = teleop.get_action()
            robot.send_action(action)
            observation = robot.get_observation()
            return action, observation
        except Exception as exc:
            self._mark_episode_invalid(episode, str(exc))
            self._stop(
                stage=stage,
                episode_index=episode_index,
                reason=str(exc),
                episode_invalid=True,
            )

    def run_reset_follow(self, *, teleop: Any, robot: Any, episode_index: int) -> None:
        stage = "C_RESET_FOLLOW"
        try:
            self._run_follow_for(
                duration_s=self.reset_time_s,
                teleop=teleop,
                robot=robot,
            )
        except Exception as exc:
            self._stop(stage=stage, episode_index=episode_index, reason=str(exc))

    @staticmethod
    def _extract_positions(data: dict[str, Any]) -> list[float]:
        keys = [f"joint{index}.pos" for index in range(7)]
        return [float(data[key]) for key in keys]

    def _bridge_interpolate_if_needed(self, *, teleop: Any, robot: Any, episode_index: int) -> None:
        self._log.info(
            "[A_BRIDGE_ENTRY] episode_index=%s target_speed=%.3f threshold=%.3f",
            episode_index,
            self.bridge_interpolation_target_speed,
            self.bridge_interpolation_threshold,
        )
        if self.bridge_interpolation_target_speed <= 0:
            self._log.info("[A_BRIDGE_ENTRY] target_speed <= 0, skip bridge")
            return
        follower_observation = robot.get_observation()
        leader_action = teleop.get_action()
        follower_positions = self._extract_positions(follower_observation)
        leader_positions = self._extract_positions(leader_action)
        deltas = [
            leader_positions[index] - follower_positions[index]
            for index in range(min(6, len(leader_positions), len(follower_positions)))
        ]
        distance = sum(delta * delta for delta in deltas) ** 0.5
        self._log.info(
            "[A_BRIDGE_PRE_CHECK] episode_index=%s follower=%s leader=%s distance=%.3f threshold=%.3f",
            episode_index,
            [f"{p:.3f}" for p in follower_positions[:6]],
            [f"{p:.3f}" for p in leader_positions[:6]],
            distance,
            self.bridge_interpolation_threshold,
        )
        if distance <= self.bridge_interpolation_threshold:
            self._log.debug(
                "[A_BRIDGE_INTERPOLATION] episode_index=%s distance=%.3f threshold=%.3f skip=True",
                episode_index,
                distance,
                self.bridge_interpolation_threshold,
            )
            return
        raw_frames = distance / (self.bridge_interpolation_target_speed * self.control_period_s)
        frame_count = ceil(raw_frames)
        frame_count = max(self.bridge_interpolation_min_frames, frame_count)
        frame_count = min(self.bridge_interpolation_max_frames, frame_count)
        duration_s = frame_count * self.control_period_s
        actual_speed = distance / duration_s if duration_s > 0 else 0.0
        self._log.info(
            "[A_BRIDGE_INTERPOLATION] episode_index=%s distance=%.3f threshold=%.3f "
            "frames=%s duration_ms=%.1f speed=%.3f",
            episode_index,
            distance,
            self.bridge_interpolation_threshold,
            frame_count,
            duration_s * 1000.0,
            actual_speed,
        )
        self._run_interpolation(
            robot=robot,
            start_positions=follower_positions,
            end_positions=leader_positions,
            frame_count=frame_count,
        )

    def _run_interpolation(
        self,
        *,
        robot: Any,
        start_positions: list[float],
        end_positions: list[float],
        frame_count: int,
    ) -> None:
        from .conversion import positions_to_action

        for index in range(1, frame_count + 1):
            t = index / frame_count
            interpolated_positions = [
                start + t * (end - start)
                for start, end in zip(start_positions, end_positions)
            ]
            robot.send_action(positions_to_action(interpolated_positions))
            if index < frame_count:
                self._sleep(self.control_period_s)

    def _run_follow_for(self, *, duration_s: float, teleop: Any, robot: Any) -> None:
        deadline = self._monotonic() + duration_s
        while self._monotonic() < deadline:
            action = teleop.get_action()
            robot.send_action(action)
            remaining = deadline - self._monotonic()
            if remaining > 0:
                self._sleep(min(self.control_period_s, remaining))

    def _run_follow_frames(self, *, frame_count: int, teleop: Any, robot: Any) -> None:
        for index in range(frame_count):
            action = teleop.get_action()
            robot.send_action(action)
            if index < frame_count - 1:
                self._sleep(self.control_period_s)

    def _stop(
        self,
        *,
        stage: str,
        episode_index: int,
        reason: str,
        episode_invalid: bool = False,
    ) -> None:
        self._log.error(
            "ZYArm record stop: stage=%s episode_index=%s reason=%s episode_invalid=%s",
            stage,
            episode_index,
            reason,
            episode_invalid,
        )
        raise ZyArmRecordStop(
            stage=stage,
            episode_index=episode_index,
            reason=reason,
            episode_invalid=episode_invalid,
        )

    @staticmethod
    def _mark_episode_invalid(episode: Any, reason: str) -> None:
        if episode is None:
            return
        mark_invalid = getattr(episode, "mark_invalid", None)
        if callable(mark_invalid):
            try:
                mark_invalid(reason=reason)
            except TypeError:
                mark_invalid()
            return
        clear_episode_buffer = getattr(episode, "clear_episode_buffer", None)
        if callable(clear_episode_buffer):
            clear_episode_buffer()
            return
        try:
            setattr(episode, "invalid", True)
            setattr(episode, "invalid_reason", reason)
        except Exception:
            return
