from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

from zyarm_hardware.config import JOINT_COUNT, ArmConfig
from zyarm_hardware.protocol import CommandId, MasterDataFrame, SerialProtocol, StatusFrame


NO_CHANGE_SENTINEL = -999.9
MASTER_ROLE = 1
SLAVE_ROLE = 2

MasterDataListener = Callable[[List[float]], None]
JointStatePublisher = Callable[[List[float]], None]


@dataclass(frozen=True)
class CommandResult:
    accepted: bool
    message: str


@dataclass(frozen=True)
class JointIoFastResult:
    accepted: bool
    message: str
    measured_positions: List[float]
    measured_valid: bool


class ArmDevice:
    def __init__(
        self,
        config: ArmConfig,
        logger,
        *,
        publish_joint_state: JointStatePublisher,
    ) -> None:
        self.config = config
        self._logger = logger
        self._publish_joint_state = publish_joint_state
        self._protocol = SerialProtocol(config.protocol_config(), logger)
        self._protocol.add_status_listener(self._on_status_frame)
        self._protocol.add_master_data_listener(self._on_master_data_frame)
        self._master_data_listeners: List[MasterDataListener] = []
        self._latest_joint_state: Optional[List[float]] = None
        self._master_data_is_authoritative = False

    @property
    def name(self) -> str:
        return self.config.name

    @property
    def is_connected(self) -> bool:
        return self._protocol.is_connected

    def add_master_data_listener(self, listener: MasterDataListener) -> None:
        self._master_data_listeners.append(listener)

    def connect(self) -> None:
        self._protocol.connect()
        if self.config.startup_reset:
            ok = self._protocol.send_command(CommandId.RESET, wait_ack=True)
            if not ok:
                raise RuntimeError(f"Arm '{self.name}' failed to reset during startup")

    def close(self) -> None:
        self._protocol.close()

    def dispatch_joint_target(
        self,
        positions: Sequence[float],
        apply_mask: Sequence[bool],
    ) -> CommandResult:
        serialized_positions = self._serialize_positions(positions, apply_mask)
        try:
            ok = self._protocol.send_command(
                CommandId.JOINT_SYNC,
                serialized_positions + [0.0, 0.0, 1.0],
                wait_ack=False,
            )
        except Exception as exc:
            return CommandResult(False, f"Arm '{self.name}' failed to dispatch joint target: {exc}")
        if not ok:
            return CommandResult(False, f"Arm '{self.name}' failed to write joint target")
        return CommandResult(True, f"Arm '{self.name}' joint target dispatched")

    def dispatch_joint_io_fast(
        self,
        positions: Sequence[float],
        apply_mask: Sequence[bool],
    ) -> JointIoFastResult:
        serialized_positions = self._serialize_positions(positions, apply_mask)
        try:
            ok = self._protocol.send_command(
                CommandId.JOINT_IO_FAST,
                serialized_positions,
                wait_ack=False,
            )
        except Exception as exc:
            return JointIoFastResult(
                False,
                f"Arm '{self.name}' failed to dispatch fast I/O: {exc}",
                [0.0] * JOINT_COUNT,
                False,
            )
        if not ok:
            return JointIoFastResult(
                False,
                f"Arm '{self.name}' failed to write fast I/O command",
                [0.0] * JOINT_COUNT,
                False,
            )
        measured_positions = [0.0] * JOINT_COUNT
        measured_valid = False
        if self._latest_joint_state is not None:
            measured_positions = list(self._latest_joint_state)
            measured_valid = True
        return JointIoFastResult(
            True,
            f"Arm '{self.name}' fast I/O dispatched",
            measured_positions,
            measured_valid,
        )

    def set_status_report(self, enabled: bool, frequency_hz: float) -> CommandResult:
        if enabled and frequency_hz <= 0.0:
            return CommandResult(False, "frequency_hz must be positive when enabling status report")
        params = [1.0, float(frequency_hz)] if enabled else [0.0]
        status_received_at = self._protocol.latest_status_received_at()
        try:
            ok = self._protocol.send_command(CommandId.STATUS_REPORT, params, wait_ack=True)
        except Exception as exc:
            return CommandResult(False, f"Arm '{self.name}' failed to configure status report: {exc}")
        if not ok and enabled and self._protocol.has_status_since(status_received_at):
            self._logger.warning(
                f"Arm '{self.name}' did not receive a status-report ACK, but fresh [STATUS] data arrived; "
                "treating status report as enabled"
            )
            ok = True
        if not ok:
            return CommandResult(False, f"Arm '{self.name}' failed to configure status report")
        return CommandResult(True, f"Arm '{self.name}' status report updated")

    def enter_master_slave(self, role: int, frequency_hz: float) -> CommandResult:
        try:
            ok = self._protocol.send_command(
                CommandId.MASTER_SLAVE,
                [float(role), float(frequency_hz)],
                wait_ack=True,
            )
        except Exception as exc:
            return CommandResult(False, f"Arm '{self.name}' failed to enter master-slave mode: {exc}")
        if not ok:
            return CommandResult(False, f"Arm '{self.name}' failed to enter master-slave mode")
        return CommandResult(True, f"Arm '{self.name}' entered master-slave mode")

    def stop_master_slave(self) -> CommandResult:
        try:
            ok = self._protocol.send_command(CommandId.MASTER_SLAVE_STOP, wait_ack=True)
        except Exception as exc:
            return CommandResult(False, f"Arm '{self.name}' failed to stop master-slave mode: {exc}")
        if not ok:
            return CommandResult(False, f"Arm '{self.name}' failed to stop master-slave mode")
        return CommandResult(True, f"Arm '{self.name}' exited master-slave mode")

    def set_master_data_authoritative(self, enabled: bool) -> None:
        self._master_data_is_authoritative = enabled

    def publish_master_data_joint_state(self, positions: Sequence[float]) -> None:
        normalized = _validate_positions(positions)
        self._latest_joint_state = normalized
        self._publish_joint_state(normalized)

    def _on_status_frame(self, frame: StatusFrame) -> None:
        positions = self.config.kinematics.hw_to_ros(frame.values)
        self._latest_joint_state = positions
        if not self._master_data_is_authoritative:
            self._publish_joint_state(positions)

    def _on_master_data_frame(self, frame: MasterDataFrame) -> None:
        positions = self.config.kinematics.hw_to_ros(frame.values)
        for listener in list(self._master_data_listeners):
            listener(positions)

    def _serialize_positions(
        self,
        positions: Sequence[float],
        apply_mask: Sequence[bool],
    ) -> List[float]:
        values = _validate_positions(positions)
        mask = _validate_apply_mask(apply_mask)
        hardware_positions = self.config.kinematics.ros_to_hw(values)
        return [
            hardware_positions[index] if mask[index] else NO_CHANGE_SENTINEL
            for index in range(JOINT_COUNT)
        ]


def _validate_positions(positions: Sequence[float]) -> List[float]:
    values = [float(value) for value in positions]
    if len(values) != JOINT_COUNT:
        raise ValueError(f"Expected {JOINT_COUNT} joint positions, got {len(values)}")
    return values


def _validate_apply_mask(apply_mask: Sequence[bool]) -> List[bool]:
    values = [bool(value) for value in apply_mask]
    if len(values) != JOINT_COUNT:
        raise ValueError(f"Expected {JOINT_COUNT} apply flags, got {len(values)}")
    return values
