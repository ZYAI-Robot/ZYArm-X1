from __future__ import annotations

from dataclasses import dataclass

import pytest

from lerobot_robot_zyarm.record_lifecycle import ZyArmRecordLifecycle, ZyArmRecordStop


@dataclass(frozen=True)
class _DiagnosticAction:
    age_ms: float = 10.0
    action: dict | None = None
    source: str = "master_data"
    sequence: int = 1


@dataclass(frozen=True)
class _DiagnosticState:
    age_ms: float = 10.0
    positions: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5)
    source: str = "cmd6_query"
    sequence: int = 1


class _FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration_s: float) -> None:
        self.now += duration_s


class _Teleop:
    def __init__(self) -> None:
        self.prepare_episode_calls = 0
        self.action_calls = 0
        self.diagnostic_action_calls = 0

    def prepare_episode(self) -> None:
        self.prepare_episode_calls += 1

    def get_action(self) -> dict:
        self.action_calls += 1
        return {f"joint{index}.pos": float(self.action_calls) for index in range(7)}

    def get_diagnostic_action(self) -> _DiagnosticAction:
        self.diagnostic_action_calls += 1
        return _DiagnosticAction(action={f"joint{index}.pos": 10.0 for index in range(7)})


class _Robot:
    def __init__(self) -> None:
        self.sent_actions = []
        self.observation_calls = 0
        self.diagnostic_state_calls = 0

    def send_action(self, action: dict) -> None:
        self.sent_actions.append(action)

    def get_observation(self) -> dict:
        self.observation_calls += 1
        return {f"joint{index}.pos": 0.0 for index in range(7)}

    def refresh_state_for_diagnostics(self) -> _DiagnosticState:
        self.diagnostic_state_calls += 1
        return _DiagnosticState()


class _Episode:
    def __init__(self) -> None:
        self.invalid = False
        self.invalid_reason = None

    def mark_invalid(self, *, reason: str) -> None:
        self.invalid = True
        self.invalid_reason = reason


def test_lifecycle_pre_roll_consumes_finalize_and_runs_five_frame_warmup() -> None:
    clock = _FakeClock()
    teleop = _Teleop()
    robot = _Robot()
    finalize_calls = []
    lifecycle = ZyArmRecordLifecycle(
        warmup_frames=5,
        reset_time_s=5.0,
        control_hz=2.0,
        bridge_interpolation_frames=0,
        bridge_interpolation_threshold=0.0,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    lifecycle.run_pre_roll_warmup(
        teleop=teleop,
        robot=robot,
        episode_index=0,
        finalize=lambda: finalize_calls.append("finalize"),
    )

    assert finalize_calls == ["finalize"]
    assert teleop.prepare_episode_calls == 1
    assert teleop.action_calls == 5
    assert len(robot.sent_actions) == 5
    assert robot.observation_calls == 0
    assert clock.now == pytest.approx(2.0)


def test_lifecycle_enter_diagnostic_reads_leader_and_follower_without_dataset_frame() -> None:
    teleop = _Teleop()
    robot = _Robot()
    lifecycle = ZyArmRecordLifecycle()

    diagnostic = lifecycle.run_enter_diagnostic(
        teleop=teleop,
        robot=robot,
        episode_index=4,
        action_max_age_ms=100.0,
        state_max_age_ms=100.0,
    )

    assert diagnostic.ok
    assert teleop.diagnostic_action_calls == 1
    assert teleop.action_calls == 0
    assert robot.diagnostic_state_calls == 1
    assert robot.observation_calls == 0
    assert robot.sent_actions == []


def test_lifecycle_enter_diagnostic_stops_before_first_frame_when_freshness_fails() -> None:
    class _StaleRobot(_Robot):
        def refresh_state_for_diagnostics(self) -> _DiagnosticState:
            return _DiagnosticState(age_ms=150.0)

    lifecycle = ZyArmRecordLifecycle()

    with pytest.raises(ZyArmRecordStop) as exc_info:
        lifecycle.run_enter_diagnostic(
            teleop=_Teleop(),
            robot=_StaleRobot(),
            episode_index=2,
            action_max_age_ms=100.0,
            state_max_age_ms=100.0,
        )

    assert exc_info.value.stage == "B_ENTER_DIAGNOSTIC"
    assert not exc_info.value.episode_invalid


def test_lifecycle_reset_follow_uses_reset_time_without_observation_or_dataset() -> None:
    clock = _FakeClock()
    teleop = _Teleop()
    robot = _Robot()
    lifecycle = ZyArmRecordLifecycle(
        warmup_s=3.0,
        reset_time_s=5.0,
        control_hz=2.0,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    lifecycle.run_reset_follow(teleop=teleop, robot=robot, episode_index=0)

    assert teleop.action_calls == 10
    assert len(robot.sent_actions) == 10
    assert robot.observation_calls == 0
    assert clock.now == pytest.approx(5.0)


def test_lifecycle_record_frame_marks_episode_invalid_on_mid_recording_failure() -> None:
    class _FailingRobot(_Robot):
        def get_observation(self) -> dict:
            raise RuntimeError("camera failed")

    lifecycle = ZyArmRecordLifecycle()
    episode = _Episode()

    with pytest.raises(ZyArmRecordStop) as exc_info:
        lifecycle.record_frame(
            teleop=_Teleop(),
            robot=_FailingRobot(),
            episode_index=8,
            episode=episode,
        )

    assert episode.invalid
    assert episode.invalid_reason == "camera failed"
    assert exc_info.value.stage == "B_RECORDING"
    assert exc_info.value.episode_invalid


def test_lifecycle_pre_roll_applies_bridge_interpolation_when_jump_exceeds_threshold() -> None:
    clock = _FakeClock()
    teleop = _Teleop()
    robot = _Robot()
    lifecycle = ZyArmRecordLifecycle(
        warmup_frames=5,
        reset_time_s=5.0,
        control_hz=2.0,
        bridge_interpolation_frames=4,
        bridge_interpolation_threshold=0.1,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    lifecycle.run_pre_roll_warmup(teleop=teleop, robot=robot, episode_index=0)

    assert teleop.action_calls == 6
    assert len(robot.sent_actions) == 9
    assert robot.observation_calls == 1


def test_lifecycle_pre_roll_skips_bridge_interpolation_within_threshold() -> None:
    class _SmallJumpTeleop(_Teleop):
        def get_action(self) -> dict:
            self.action_calls += 1
            value = 0.05
            return {f"joint{index}.pos": value for index in range(7)}

    clock = _FakeClock()
    teleop = _SmallJumpTeleop()
    robot = _Robot()
    lifecycle = ZyArmRecordLifecycle(
        warmup_frames=5,
        reset_time_s=5.0,
        control_hz=2.0,
        bridge_interpolation_frames=4,
        bridge_interpolation_threshold=0.2,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    lifecycle.run_pre_roll_warmup(teleop=teleop, robot=robot, episode_index=0)

    assert teleop.action_calls == 6
    assert len(robot.sent_actions) == 5
    assert robot.observation_calls == 1


def test_lifecycle_pre_roll_skips_bridge_interpolation_when_disabled() -> None:
    clock = _FakeClock()
    teleop = _Teleop()
    robot = _Robot()
    lifecycle = ZyArmRecordLifecycle(
        warmup_frames=5,
        reset_time_s=5.0,
        control_hz=2.0,
        bridge_interpolation_frames=0,
        bridge_interpolation_threshold=0.0,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    lifecycle.run_pre_roll_warmup(teleop=teleop, robot=robot, episode_index=0)

    assert teleop.action_calls == 5
    assert len(robot.sent_actions) == 5
    assert robot.observation_calls == 0
