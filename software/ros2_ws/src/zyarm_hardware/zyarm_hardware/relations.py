from __future__ import annotations

from typing import Callable, List

from zyarm_hardware.config import RelationConfig
from zyarm_hardware.device import MASTER_ROLE, ArmDevice, CommandResult


CommandPublisher = Callable[[List[float]], None]


class ArmRelationCoordinator:
    def __init__(
        self,
        config: RelationConfig,
        leader: ArmDevice,
        follower: ArmDevice,
        logger,
        *,
        publish_follower_command: CommandPublisher,
    ) -> None:
        self.config = config
        self._leader = leader
        self._follower = follower
        self._logger = logger
        self._publish_follower_command = publish_follower_command
        self._enabled = False
        self._leader.add_master_data_listener(self._on_leader_master_data)

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    def set_enabled(self, enabled: bool) -> CommandResult:
        if enabled:
            return self._enable()
        return self._disable()

    def close(self) -> None:
        if self._enabled:
            self._disable()

    def _enable(self) -> CommandResult:
        if self._enabled:
            return CommandResult(True, f"Relation '{self.config.name}' is already enabled")
        if not self._leader.is_connected or not self._follower.is_connected:
            return CommandResult(
                False,
                f"Relation '{self.config.name}' requires connected leader and follower devices",
            )

        leader_result = self._leader.enter_master_slave(MASTER_ROLE, self.config.follow_frequency_hz)
        if not leader_result.accepted:
            return leader_result
        self._leader.set_master_data_authoritative(True)
        self._enabled = True
        return CommandResult(True, f"Relation '{self.config.name}' enabled")

    def _disable(self) -> CommandResult:
        if not self._enabled:
            return CommandResult(True, f"Relation '{self.config.name}' is already disabled")
        self._enabled = False
        self._leader.set_master_data_authoritative(False)
        leader_result = self._leader.stop_master_slave()
        if not leader_result.accepted:
            return leader_result
        return CommandResult(True, f"Relation '{self.config.name}' disabled")

    def _on_leader_master_data(self, positions: List[float]) -> None:
        if not self._enabled:
            return
        self._leader.publish_master_data_joint_state(positions)
        result = self._follower.dispatch_joint_io_fast(positions, [True] * len(positions))
        if not result.accepted:
            self._logger.warning(
                f"Relation '{self.config.name}' failed to relay leader command to follower: {result.message}"
            )
            return
        self._publish_follower_command(positions)
