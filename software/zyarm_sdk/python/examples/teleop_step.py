from zyarm_sdk import ZyArm, ZyArmConfig
from zyarm_sdk.teleop import ZyArmTeleopPair


def main() -> None:
    leader = ZyArm(ZyArmConfig(port="/dev/ttyUSB0"))
    follower = ZyArm(ZyArmConfig(port="/dev/ttyUSB1"))
    pair = ZyArmTeleopPair(leader, follower).connect()
    try:
        pair.start_step_mode()
        result = pair.step(wait_action=True)
        if result is not None:
            print(result.action.positions)
    finally:
        pair.close()


if __name__ == "__main__":
    main()
