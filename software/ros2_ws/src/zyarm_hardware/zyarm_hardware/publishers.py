from __future__ import annotations

from typing import Sequence

try:
    from sensor_msgs.msg import JointState
except ModuleNotFoundError:  # pragma: no cover - exercised only without ROS installed.
    class JointState:  # type: ignore[no-redef]
        def __init__(self) -> None:
            self.header = type("Header", (), {"stamp": None})()
            self.name = []
            self.position = []
            self.velocity = []


ARM_JOINTS = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5"]
GRIPPER_JOINT = "joint6"
ALL_JOINTS = ARM_JOINTS + [GRIPPER_JOINT]


def build_joint_state_message(positions: Sequence[float], *, stamp=None) -> JointState:
    message = JointState()
    if hasattr(message, "header"):
        message.header.stamp = stamp
    message.name = list(ALL_JOINTS)
    message.position = [float(value) for value in positions]
    message.velocity = [0.0] * len(message.position)
    return message

