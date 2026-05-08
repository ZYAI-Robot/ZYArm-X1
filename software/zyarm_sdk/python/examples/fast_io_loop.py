import time

from zyarm_sdk import ZyArm, ZyArmConfig


def main() -> None:
    target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]
    with ZyArm(ZyArmConfig(port="/dev/ttyUSB0")) as arm:
        for _ in range(100):
            arm.fast_io(target)
            time.sleep(0.02)


if __name__ == "__main__":
    main()
