from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional

try:
    import cv2
except ModuleNotFoundError as exc:  # pragma: no cover - depends on local env
    cv2 = None
    _OPENCV_IMPORT_ERROR = exc
else:
    _OPENCV_IMPORT_ERROR = None


ROOT = Path(__file__).resolve().parent
SRC_DIR = ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from aruco_vision import ArucoVision
from camera_stream import CameraStream
from config_loader import load_config


DEFAULT_CONFIG = ROOT / "config" / "wrist_rgb_aruco_pick.yaml"


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        _require_opencv()
        config = load_config(args.config)
        vision = ArucoVision(config.camera, config.marker)
        if args.image is not None:
            frame = cv2.imread(str(args.image))
            if frame is None:
                raise RuntimeError(f"Failed to read image: {args.image}")
            detection = vision.detect(frame)
            overlay = vision.draw_overlay(frame, detection)
            _show_or_save(args, overlay)
            print(json.dumps(detection.to_dict(), ensure_ascii=False, sort_keys=True))
            return 0
        return run_camera(args.camera, config, vision, args)
    except Exception as exc:  # noqa: BLE001 - CLI should report readable errors.
        print(f"error: {exc}", file=sys.stderr)
        return 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Detect a target ArUco block from wrist RGB input.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path, help="Path to a single image file.")
    source.add_argument("--camera", type=int, help="OpenCV camera index.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="Path to wrist RGB ArUco YAML config.")
    parser.add_argument("--show", action="store_true", help="Show overlay window.")
    parser.add_argument("--save", type=Path, help="Save overlay image to this path.")
    parser.add_argument("--max-frames", type=int, default=0, help="Camera frame limit; 0 means run until q/Esc.")
    return parser


def run_camera(camera_index: int, config, vision: ArucoVision, args: argparse.Namespace) -> int:
    camera_config = dict(config.camera)
    camera_config["index"] = camera_index
    stream = CameraStream(camera_config, cv2_module=cv2)
    frame_index = 0
    try:
        while True:
            frame = stream.read_frame()
            detection = vision.detect(frame)
            overlay = vision.draw_overlay(frame, detection)
            print(
                json.dumps(
                    {"frame_index": frame_index, **detection.to_dict()},
                    ensure_ascii=False,
                    sort_keys=True,
                )
            )
            if args.save is not None:
                _write_image(args.save, overlay)
            if args.show:
                cv2.imshow("Wrist RGB ArUco", overlay)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
            frame_index += 1
            if args.max_frames > 0 and frame_index >= args.max_frames:
                break
    finally:
        stream.close()
        if args.show:
            cv2.destroyAllWindows()
    return 0


def _show_or_save(args: argparse.Namespace, overlay) -> None:
    if args.save is not None:
        _write_image(args.save, overlay)
    if args.show:
        cv2.imshow("Wrist RGB ArUco", overlay)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def _write_image(path: Path, image) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok = cv2.imwrite(str(path), image)
    if not ok:
        raise RuntimeError(f"Failed to write image: {path}")


def _require_opencv() -> None:
    if cv2 is None:
        raise RuntimeError(
            "Missing dependency OpenCV contrib. Install it with `python -m pip install opencv-contrib-python`."
        ) from _OPENCV_IMPORT_ERROR


if __name__ == "__main__":
    raise SystemExit(main())
