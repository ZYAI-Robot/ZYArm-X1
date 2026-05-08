from __future__ import annotations

import time
from typing import Optional, Sequence

from .config import ZyArmConfig
from .mapping import JointMapping
from .protocol import CommandId, MasterSlaveRole
from .safety import SafetyController, validate_apply_mask
from .transport import SerialTransport
from .types import ArmFrameStats, ArmState, CommandResult, FastIoResult, StateSource


class ZyArm:
    def __init__(self, config: ZyArmConfig, *, transport: Optional[SerialTransport] = None) -> None:
        self.config = config
        self.mapping = JointMapping(config.mapping)
        self.safety = SafetyController(config.safety)
        self.transport = transport or SerialTransport(config)

    @property
    def is_connected(self) -> bool:
        return self.transport.is_connected

    def connect(self) -> "ZyArm":
        self.transport.connect()
        return self

    def close(self) -> None:
        self.transport.close()

    def __enter__(self) -> "ZyArm":
        return self.connect()

    def __exit__(self, *_exc) -> None:
        self.close()

    def reset(self, *, timeout_ms: Optional[float] = None) -> CommandResult:
        return self._send_action_command(CommandId.RESET, timeout_ms=timeout_ms)

    def stop(self, *, timeout_ms: Optional[float] = None) -> CommandResult:
        return self._send_ack_command(CommandId.STOP, timeout_ms=timeout_ms)

    def send_command(
        self,
        command_id: int,
        params: Optional[Sequence[float]] = None,
        *,
        wait_ack: bool = True,
        timeout_ms: Optional[float] = None,
    ) -> CommandResult:
        dispatched_at = time.perf_counter()
        ok = self.transport.send_command(
            int(command_id),
            params,
            wait_ack=wait_ack,
            timeout_s=None if timeout_ms is None else timeout_ms / 1000.0,
        )
        return CommandResult(
            accepted=ok,
            message="command dispatched" if ok else "ACK timeout or error",
            command_id=int(command_id),
            dispatched_at=dispatched_at,
        )

    def move_ik(
        self,
        x: float,
        y: float,
        z: float,
        rx: float = 0.0,
        ry: float = 0.0,
        rz: float = 0.0,
        *,
        timeout_ms: Optional[float] = None,
    ) -> CommandResult:
        return self._send_action_command(
            CommandId.IK_INVERSE,
            [x, y, z, rx, ry, rz],
            timeout_ms=timeout_ms,
        )

    def set_gripper(
        self,
        position: float,
        *,
        sync: bool = False,
        timeout_ms: Optional[float] = None,
    ) -> CommandResult:
        normalized = min(1.0, max(0.0, float(position)))
        hardware = normalized * self.config.mapping.gripper_command_max
        params = [hardware, 1.0 if sync else 0.0]
        if sync:
            return self._send_action_command(
                CommandId.SET_CLAW,
                params,
                timeout_ms=timeout_ms,
            )
        return self.send_command(CommandId.SET_CLAW, params, wait_ack=False)

    def play_record(self, action_id: int, *, timeout_ms: Optional[float] = None) -> CommandResult:
        return self._send_ack_command(
            CommandId.RECORD_PLAYER,
            [float(action_id)],
            timeout_ms=timeout_ms,
            default_timeout_s=self.config.play_record_timeout_s,
        )

    def set_speed(self, speed: float) -> CommandResult:
        return self.send_command(CommandId.SET_JOINT_SPEED, [float(speed)], wait_ack=False)

    def set_remote_mode(self, enabled: bool) -> CommandResult:
        return self.send_command(CommandId.REMOTE_MODE, [1.0 if enabled else 0.0], wait_ack=True)

    def remote_reset(self, *, timeout_ms: Optional[float] = None) -> CommandResult:
        return self._send_action_command(CommandId.REMOTE_RESET, timeout_ms=timeout_ms)

    def remote_control(
        self,
        x: float,
        y: float,
        z: float,
        rx: float = 0.0,
        ry: float = 0.0,
        rz: float = 0.0,
        gripper: float = 0.0,
    ) -> CommandResult:
        return self.send_command(CommandId.REMOTE, [x, y, z, rx, ry, rz, gripper], wait_ack=False)

    def get_latest_state(self, max_age_ms: Optional[float] = None) -> Optional[ArmState]:
        frame = self.transport.latest_status()
        if frame is None:
            return None
        state = self._state_from_frame(frame, StateSource.CACHE)
        self.safety.assert_fresh(state, max_age_ms)
        return state

    def query_state(self, *, timeout_ms: float = 1000.0) -> Optional[ArmState]:
        before = self.transport.status_sequence
        self.transport.send_command(CommandId.STATUS, wait_ack=False)
        frame = self.transport.wait_for_status_after(before, timeout_ms / 1000.0)
        if frame is None:
            return None
        return self._state_from_frame(frame, StateSource.CMD6_QUERY)

    def fast_io(
        self,
        positions: Sequence[float],
        apply_mask: Optional[Sequence[bool]] = None,
        *,
        wait_state: bool = False,
        timeout_ms: float = 50.0,
    ) -> FastIoResult:
        mask = validate_apply_mask(apply_mask)
        safe_positions = self.safety.sanitize_positions(positions)
        hardware = self.mapping.public_to_hardware(safe_positions, mask)
        before = self.transport.status_sequence
        dispatched_at = time.perf_counter()
        self.transport.send_command(CommandId.JOINT_IO_FAST, hardware, wait_ack=False)
        measured = None
        if wait_state:
            frame = self.transport.wait_for_status_after(before, timeout_ms / 1000.0)
            if frame is not None:
                measured = self._state_from_frame(frame, StateSource.CMD36_MEASURED_SNAPSHOT)
        return FastIoResult(
            accepted=True,
            message="CMD36 dispatched",
            command_id=int(CommandId.JOINT_IO_FAST),
            dispatched_at=dispatched_at,
            measured_snapshot=measured,
            state_waited=wait_state,
        )

    def enter_master_slave_mode(
        self,
        role: MasterSlaveRole,
        *,
        frequency_hz: float = 50.0,
    ) -> CommandResult:
        role_value = MasterSlaveRole(role)
        return self._send_ack_command(
            CommandId.MASTER_SLAVE,
            [float(role_value), float(frequency_hz)],
        )

    def enter_master_mode(self, *, frequency_hz: float = 50.0) -> CommandResult:
        return self.enter_master_slave_mode(MasterSlaveRole.MASTER, frequency_hz=frequency_hz)

    def enter_slave_mode(self, *, frequency_hz: float = 50.0) -> CommandResult:
        return self.enter_master_slave_mode(MasterSlaveRole.SLAVE, frequency_hz=frequency_hz)

    def stop_master_mode(self) -> CommandResult:
        return self._send_ack_command(CommandId.MASTER_SLAVE_STOP)

    def set_master_slave_lpf(self, alpha: float) -> CommandResult:
        return self._send_ack_command(CommandId.MASTER_SLAVE_SET_LPF, [float(alpha)])

    def get_frame_stats(self) -> ArmFrameStats:
        return self.transport.get_frame_stats()

    def reset_frame_stats(self) -> None:
        self.transport.reset_frame_stats()

    def _send_ack_command(
        self,
        command_id: int,
        params: Optional[Sequence[float]] = None,
        *,
        timeout_ms: Optional[float] = None,
        default_timeout_s: Optional[float] = None,
    ) -> CommandResult:
        dispatched_at = time.perf_counter()
        timeout_s = default_timeout_s if timeout_ms is None else timeout_ms / 1000.0
        ok = self.transport.send_command(
            command_id,
            params,
            wait_ack=True,
            timeout_s=timeout_s,
        )
        return CommandResult(
            accepted=ok,
            message="ACK success" if ok else "ACK timeout or error",
            command_id=int(command_id),
            dispatched_at=dispatched_at,
        )

    def _send_action_command(
        self,
        command_id: int,
        params: Optional[Sequence[float]] = None,
        *,
        timeout_ms: Optional[float] = None,
    ) -> CommandResult:
        # 动作类 ACK 代表固件执行完成，不只是命令写入成功，因此使用更长的默认等待时间。
        return self._send_ack_command(
            command_id,
            params,
            timeout_ms=timeout_ms,
            default_timeout_s=self.config.action_timeout_s,
        )

    def _state_from_frame(self, frame, source: StateSource) -> ArmState:
        return ArmState(
            positions=tuple(self.mapping.hardware_to_public(frame.values)),
            source=source,
            timestamp=frame.received_at,
            sequence=frame.sequence,
            raw_line=frame.raw_line,
        )
