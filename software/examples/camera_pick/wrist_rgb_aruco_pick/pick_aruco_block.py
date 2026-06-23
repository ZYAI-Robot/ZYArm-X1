from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional


ROOT = Path(__file__).resolve().parent
SRC_DIR = ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from config_loader import load_config
from pick_controller import WristArucoPickController


DEFAULT_CONFIG = ROOT / "config" / "wrist_rgb_aruco_pick.yaml"


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        controller = WristArucoPickController(
            config,
            arm_port=args.port,
            show_preview=not args.no_show,
            preview_wait_ms=args.preview_wait_ms,
        )
        controller.run(dry_run=args.dry_run)
        return 0
    except Exception as exc:  # noqa: BLE001 - CLI should report readable errors.
        print(f"error: {exc}", file=sys.stderr)
        return 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pick and place an ArUco-tagged block with a wrist RGB camera.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="Path to wrist RGB ArUco YAML config.")
    parser.add_argument("--port", help="ZYArm serial port, for example COM3 or /dev/ttyUSB0.")
    parser.add_argument("--dry-run", action="store_true", help="Plan one cycle without connecting or moving the arm.")
    parser.add_argument("--no-show", action="store_true", help="Do not show the ArUco detection preview window.")
    parser.add_argument(
        "--preview-wait-ms",
        type=int,
        default=500,
        help="Milliseconds to keep the preview window responsive after detection; 0 waits for a key.",
    )
    return parser


if __name__ == "__main__":
    raise SystemExit(main())
