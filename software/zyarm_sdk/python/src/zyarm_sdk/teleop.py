from __future__ import annotations

import threading
from typing import Optional

from .arm import ZyArm
from .config import TeleopConfig
from .errors import StaleStateError
from .protocol import MasterDataFrame
from .retarget import RetargetConfig, Retargeter
from .types import StateSource, TeleopAction, TeleopStepResult


class ZyArmLeader:
    def __init__(self, arm: ZyArm, config: Optional[TeleopConfig] = None) -> None:
        self.arm = arm
        self.config = config or TeleopConfig()

    def start(self) -> None:
        self.arm.enter_master_mode(frequency_hz=self.config.leader_hz)

    def stop(self) -> None:
        self.arm.stop_master_mode()

    def get_action(
        self,
        *,
        max_age_ms: Optional[float] = None,
        wait: bool = False,
        timeout_ms: Optional[float] = None,
    ) -> Optional[TeleopAction]:
        frame = None
        if wait:
            before = self.arm.transport.master_data_sequence
            frame = self.arm.transport.wait_for_master_data_after(
                before,
                (self.config.wait_timeout_ms if timeout_ms is None else timeout_ms) / 1000.0,
            )
        else:
            frame = self.arm.transport.latest_master_data()
        if frame is None:
            return None
        return self.action_from_frame(frame, max_age_ms=max_age_ms)

    def action_from_frame(
        self,
        frame: MasterDataFrame,
        *,
        max_age_ms: Optional[float] = None,
    ) -> TeleopAction:
        action = TeleopAction(
            positions=tuple(self.arm.mapping.hardware_to_public(frame.values)),
            source=StateSource.MASTER_DATA,
            timestamp=frame.received_at,
            sequence=frame.sequence,
            raw_line=frame.raw_line,
        )
        limit = self.config.action_max_age_ms if max_age_ms is None else max_age_ms
        if action.age_ms > limit:
            raise StaleStateError(f"Leader action is stale: {action.age_ms:.1f} ms > {limit:.1f} ms")
        return action


class ZyArmFollower:
    def __init__(
        self,
        arm: ZyArm,
        *,
        retarget: Optional[RetargetConfig] = None,
        config: Optional[TeleopConfig] = None,
    ) -> None:
        self.arm = arm
        self.config = config or TeleopConfig()
        self.retargeter = Retargeter(retarget)

    def send_action(self, action: TeleopAction, *, wait_state: bool = False) -> TeleopStepResult:
        positions = self.retargeter.apply(action)
        command = self.arm.fast_io(
            positions,
            self.retargeter.apply_mask,
            wait_state=wait_state,
            timeout_ms=self.config.wait_timeout_ms,
        )
        observation = self.arm.get_latest_state(self.config.state_max_age_ms)
        return TeleopStepResult(action=action, command=command, observation=observation)

    def get_observation(self):
        return self.arm.get_latest_state(self.config.state_max_age_ms)


class ZyArmTeleopPair:
    def __init__(
        self,
        leader: ZyArm,
        follower: ZyArm,
        *,
        config: Optional[TeleopConfig] = None,
        retarget: Optional[RetargetConfig] = None,
    ) -> None:
        self.config = config or TeleopConfig()
        self.leader = ZyArmLeader(leader, self.config)
        self.follower = ZyArmFollower(follower, retarget=retarget, config=self.config)
        self._follow_stop = threading.Event()
        self._follow_thread: Optional[threading.Thread] = None
        self._follower_slave_started = False

    def connect(self) -> "ZyArmTeleopPair":
        self.leader.arm.connect()
        self.follower.arm.connect()
        return self

    def start_step_mode(self) -> None:
        try:
            self._start_follower_slave_mode()
            self.leader.start()
        except Exception:
            self._stop_follower_slave_mode()
            raise

    def step(self, *, wait_action: bool = False, wait_state: bool = False) -> Optional[TeleopStepResult]:
        action = self.leader.get_action(wait=wait_action)
        if action is None:
            return None
        return self.follower.send_action(action, wait_state=wait_state)

    def start_auto_follow(self) -> None:
        if self._follow_thread and self._follow_thread.is_alive():
            return
        self._follow_stop.clear()
        self.start_step_mode()
        last_sequence = self.leader.arm.transport.master_data_sequence
        self._follow_thread = threading.Thread(
            target=self._auto_follow_loop,
            args=(last_sequence,),
            daemon=True,
        )
        self._follow_thread.start()

    def stop(self) -> None:
        self._follow_stop.set()
        if self._follow_thread and self._follow_thread.is_alive():
            self._follow_thread.join(timeout=1.0)
        self._follow_thread = None
        self.leader.stop()
        self._stop_follower_slave_mode()

    def close(self) -> None:
        self.stop()
        self.leader.arm.close()
        self.follower.arm.close()

    def _start_follower_slave_mode(self) -> None:
        if self._follower_slave_started:
            return
        result = self.follower.arm.set_master_slave_lpf(self.config.slave_filter_lpf_alpha)
        if not result.accepted:
            raise RuntimeError("follower slave filter LPF configuration failed")
        result = self.follower.arm.enter_slave_mode()
        if not result.accepted:
            raise RuntimeError("follower slave mode start failed")
        self._follower_slave_started = True

    def _stop_follower_slave_mode(self) -> None:
        if not self._follower_slave_started:
            return
        try:
            self.follower.arm.stop_master_mode()
        finally:
            self._follower_slave_started = False

    def _auto_follow_loop(self, last_sequence: int) -> None:
        while not self._follow_stop.is_set():
            frame = self.leader.arm.transport.wait_for_master_data_after(
                last_sequence,
                self.config.wait_timeout_ms / 1000.0,
            )
            if frame is None:
                continue
            last_sequence = frame.sequence
            try:
                action = self.leader.action_from_frame(frame)
                self.follower.send_action(action, wait_state=False)
            except StaleStateError:
                continue
            except Exception:
                continue
