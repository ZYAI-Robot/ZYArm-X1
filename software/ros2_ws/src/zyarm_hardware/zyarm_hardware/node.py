from __future__ import annotations

from typing import Callable, Dict

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from zyarm_hardware.config import load_system_config
from zyarm_hardware.device import ArmDevice
from zyarm_hardware.publishers import build_joint_state_message
from zyarm_hardware.relations import ArmRelationCoordinator

try:
    from zyarm_interfaces.srv import JointIoFast, SetJointTarget, SetRelationEnabled, SetStatusReport
except (ImportError, ModuleNotFoundError):  # pragma: no cover - before interfaces are regenerated.
    class _JointIoFastRequest:
        def __init__(self) -> None:
            self.positions = [0.0] * 7
            self.apply_mask = [False] * 7

    class _SetJointTargetRequest:
        def __init__(self) -> None:
            self.positions = [0.0] * 7
            self.apply_mask = [False] * 7

    class _SetStatusReportRequest:
        def __init__(self) -> None:
            self.enabled = False
            self.frequency_hz = 0.0

    class _SetRelationEnabledRequest:
        def __init__(self) -> None:
            self.enabled = False

    class SetJointTarget:  # type: ignore[no-redef]
        Request = _SetJointTargetRequest

    class JointIoFast:  # type: ignore[no-redef]
        Request = _JointIoFastRequest

    class SetStatusReport:  # type: ignore[no-redef]
        Request = _SetStatusReportRequest

    class SetRelationEnabled:  # type: ignore[no-redef]
        Request = _SetRelationEnabledRequest


class ArmSystemNode(Node):
    def __init__(self, *, config_file: str | None = None) -> None:
        parameter_overrides = []
        if config_file is not None:
            parameter_overrides.append(Parameter("config_file", value=str(config_file)))
        super().__init__("zyarm_arm_system", parameter_overrides=parameter_overrides)
        self.declare_parameter("config_file", "")
        config_path = str(self.get_parameter("config_file").value).strip()
        if not config_path:
            raise RuntimeError("Parameter 'config_file' is required")

        self._config = load_system_config(config_path)
        self._joint_state_publishers = {
            arm_name: self.create_publisher(JointState, f"{arm_name}/joint_state", 10)
            for arm_name in self._config.arms
        }
        self._joint_command_publishers: Dict[str, object] = {}
        self._devices: Dict[str, ArmDevice] = {}
        self._relations: Dict[str, ArmRelationCoordinator] = {}
        self._managed_services = []
        self._initialize_runtime()

    def destroy_node(self):
        for relation in self._relations.values():
            relation.close()
        for device in self._devices.values():
            device.close()
        super().destroy_node()

    def _initialize_runtime(self) -> None:
        for arm_name, arm_config in self._config.arms.items():
            device = ArmDevice(
                arm_config,
                self.get_logger(),
                publish_joint_state=self._make_joint_state_publisher(arm_name),
            )
            self._devices[arm_name] = device
            self._managed_services.append(
                self.create_service(
                    JointIoFast,
                    f"{arm_name}/joint_io_fast",
                    self._make_joint_io_fast_handler(device),
                )
            )
            self._managed_services.append(
                self.create_service(
                    SetJointTarget,
                    f"{arm_name}/set_joint_target",
                    self._make_joint_target_handler(device),
                )
            )
            self._managed_services.append(
                self.create_service(
                    SetStatusReport,
                    f"{arm_name}/set_status_report",
                    self._make_status_report_handler(device),
                )
            )

        for arm_name, device in self._devices.items():
            self.get_logger().info(f"Connecting arm '{arm_name}' on {device.config.port}")
            device.connect()

        for relation_name, relation_config in self._config.relationships.items():
            follower = relation_config.follower
            if follower not in self._joint_command_publishers:
                self._joint_command_publishers[follower] = self.create_publisher(
                    JointState,
                    f"{follower}/joint_command",
                    10,
                )
            relation = ArmRelationCoordinator(
                relation_config,
                self._devices[relation_config.leader],
                self._devices[relation_config.follower],
                self.get_logger(),
                publish_follower_command=self._make_joint_command_publisher(follower),
            )
            self._relations[relation_name] = relation
            self._managed_services.append(
                self.create_service(
                    SetRelationEnabled,
                    f"{relation_name}/set_enabled",
                    self._make_relation_handler(relation),
                )
            )

    def _make_joint_state_publisher(self, arm_name: str) -> Callable[[list[float]], None]:
        publisher = self._joint_state_publishers[arm_name]

        def publish(positions: list[float]) -> None:
            publisher.publish(
                build_joint_state_message(
                    positions,
                    stamp=self.get_clock().now().to_msg(),
                )
            )

        return publish

    def _make_joint_command_publisher(self, follower_name: str) -> Callable[[list[float]], None]:
        publisher = self._joint_command_publishers[follower_name]

        def publish(positions: list[float]) -> None:
            publisher.publish(
                build_joint_state_message(
                    positions,
                    stamp=self.get_clock().now().to_msg(),
                )
            )

        return publish

    def _make_joint_target_handler(self, device: ArmDevice):
        def handle(request, response):
            try:
                result = device.dispatch_joint_target(request.positions, request.apply_mask)
            except Exception as exc:
                response.accepted = False
                response.message = str(exc)
                return response
            response.accepted = result.accepted
            response.message = result.message
            return response

        return handle

    def _make_joint_io_fast_handler(self, device: ArmDevice):
        def handle(request, response):
            try:
                result = device.dispatch_joint_io_fast(request.positions, request.apply_mask)
            except Exception as exc:
                response.accepted = False
                response.message = str(exc)
                response.measured_positions = [0.0] * 7
                response.measured_valid = False
                return response
            response.accepted = result.accepted
            response.message = result.message
            response.measured_positions = list(result.measured_positions)
            response.measured_valid = bool(result.measured_valid)
            return response

        return handle

    def _make_status_report_handler(self, device: ArmDevice):
        def handle(request, response):
            try:
                result = device.set_status_report(bool(request.enabled), float(request.frequency_hz))
            except Exception as exc:
                response.accepted = False
                response.message = str(exc)
                return response
            response.accepted = result.accepted
            response.message = result.message
            return response

        return handle

    def _make_relation_handler(self, relation: ArmRelationCoordinator):
        def handle(request, response):
            try:
                result = relation.set_enabled(bool(request.enabled))
            except Exception as exc:
                response.accepted = False
                response.message = str(exc)
                return response
            response.accepted = result.accepted
            response.message = result.message
            return response

        return handle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmSystemNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
