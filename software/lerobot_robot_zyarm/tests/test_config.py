from lerobot_robot_zyarm.config import ZyArmFollowerRobotConfig, ZyArmLeaderTeleoperatorConfig


def test_config_types_are_registered() -> None:
    follower = ZyArmFollowerRobotConfig(port="/dev/ttyUSB0")
    leader = ZyArmLeaderTeleoperatorConfig(port="/dev/ttyUSB1")

    assert follower.type == "zyarm_follower"
    assert follower.baudrate == 230_400
    assert follower.slave_filter_lpf_alpha == 0.15
    assert leader.type == "zyarm_leader"
    assert leader.baudrate == 230_400
