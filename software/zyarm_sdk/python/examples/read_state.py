from zyarm_sdk import ZyArm, ZyArmConfig


def main() -> None:
    with ZyArm(ZyArmConfig(port="/dev/ttyUSB0")) as arm:
        state = arm.query_state(timeout_ms=1000)
        if state is None:
            print("No fresh state received")
            return
        print(state.positions)


if __name__ == "__main__":
    main()
