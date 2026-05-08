from __future__ import annotations

import sys
from pathlib import Path

SDK_SRC = Path(__file__).resolve().parents[1] / "zyarm_sdk" / "python" / "src"
if str(SDK_SRC) not in sys.path:
    sys.path.insert(0, str(SDK_SRC))

from zyarm_sdk import ZyArm, ZyArmConfig  # noqa: E402
from zyarm_sdk.protocol import CommandId  # noqa: E402


def create_arm(port: str, baudrate: int = 230_400, timeout_s: float = 0.02) -> ZyArm:
    return ZyArm(ZyArmConfig(port=port, baudrate=baudrate, timeout_s=timeout_s)).connect()


def ok(result) -> bool:
    return bool(getattr(result, "accepted", result))
