from __future__ import annotations

import copy
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from config_loader import parse_config


def valid_config() -> dict:
    return {
        "camera": {
            "index": 0,
            "width": 640,
            "height": 480,
            "fps": 30,
            "camera_matrix": [[800.0, 0.0, 320.0], [0.0, 800.0, 240.0], [0.0, 0.0, 1.0]],
            "dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
        },
        "marker": {
            "dictionary": "DICT_4X4_50",
            "id": 0,
            "size_mm": 30.0,
        },
        "observe_pose": {
            "x_mm": 180.0,
            "y_mm": 0.0,
            "z_mm": 80.0,
            "rx_deg": 0.0,
            "ry_deg": 0.0,
            "rz_deg": 0.0,
        },
        "place_pose": {
            "x_mm": 160.0,
            "y_mm": -90.0,
            "z_mm": 30.0,
            "rx_deg": 0.0,
            "ry_deg": 0.0,
            "rz_deg": 0.0,
        },
        "mapping": {
            "reference_camera_xy_mm": [0.0, 0.0],
            "camera_xy_to_base_xy": [[0.0, -1.0], [-1.0, 0.0]],
            "grasp_offset_base_xy_mm": [1.0, -2.0],
            "max_xy_offset_mm": 80.0,
        },
        "task": {
            "safe_z_mm": 60.0,
            "grasp_z_mm": -60.0,
            "grasp_pause_s": 0.0,
        },
        "loop": {"max_cycles": 2, "wait_for_user": True},
        "safety": {"require_marker_usable": True},
    }


class ConfigLoaderTests(unittest.TestCase):
    def test_valid_config_parses_typed_sections(self) -> None:
        config = parse_config(valid_config())

        self.assertEqual(config.marker.marker_id, 0)
        self.assertEqual(config.observe_pose.x_mm, 180.0)
        self.assertEqual(config.place_pose.y_mm, -90.0)
        self.assertEqual(config.mapping.camera_xy_to_base_xy[0], (0.0, -1.0))
        self.assertEqual(config.loop.max_cycles, 2)

    def test_camera_to_tool_config_parses(self) -> None:
        raw = valid_config()
        raw["mapping"]["camera_to_tool"] = {
            "translation_mm": [54.8, 0.0, 56.8],
            "rotation_deg": [0.0, 30.0, 0.0],
        }
        raw["mapping"]["opencv_to_camera"] = {
            "rotation_matrix": [[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]
        }

        config = parse_config(raw)

        self.assertIsNotNone(config.mapping.camera_to_tool)
        self.assertEqual(config.mapping.camera_to_tool.translation_mm, (54.8, 0.0, 56.8))
        self.assertEqual(config.mapping.camera_to_tool.rotation_deg, (0.0, 30.0, 0.0))
        self.assertEqual(
            config.mapping.opencv_to_camera_rotation,
            ((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
        )

    def test_camera_to_tool_config_does_not_require_legacy_matrix(self) -> None:
        raw = valid_config()
        raw["mapping"]["camera_to_tool"] = {
            "translation_mm": [54.8, 0.0, 56.8],
            "rotation_deg": [0.0, 30.0, 0.0],
        }
        raw["mapping"]["opencv_to_camera"] = {
            "rotation_matrix": [[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]
        }
        raw["mapping"].pop("reference_camera_xy_mm")
        raw["mapping"].pop("camera_xy_to_base_xy")

        config = parse_config(raw)

        self.assertIsNone(config.mapping.reference_camera_xy_mm)
        self.assertIsNone(config.mapping.camera_xy_to_base_xy)
        self.assertIsNotNone(config.mapping.camera_to_tool)

    def test_camera_to_tool_requires_opencv_to_camera_rotation(self) -> None:
        raw = valid_config()
        raw["mapping"]["camera_to_tool"] = {
            "translation_mm": [54.8, 0.0, 56.8],
            "rotation_deg": [0.0, 30.0, 0.0],
        }

        with self.assertRaisesRegex(ValueError, "opencv_to_camera.rotation_matrix"):
            parse_config(raw)

    def test_missing_place_pose_is_rejected(self) -> None:
        raw = valid_config()
        raw.pop("place_pose")

        with self.assertRaisesRegex(ValueError, "place_pose"):
            parse_config(raw)

    def test_invalid_matrix_is_rejected(self) -> None:
        raw = valid_config()
        raw["mapping"]["camera_xy_to_base_xy"] = [[1.0, 0.0]]

        with self.assertRaisesRegex(ValueError, "camera_xy_to_base_xy"):
            parse_config(raw)

    def test_non_finite_number_is_rejected(self) -> None:
        raw = valid_config()
        raw["observe_pose"]["x_mm"] = float("nan")

        with self.assertRaisesRegex(ValueError, "observe_pose.x_mm"):
            parse_config(raw)

    def test_height_order_is_rejected(self) -> None:
        raw = valid_config()
        raw["task"]["grasp_z_mm"] = 70.0

        with self.assertRaisesRegex(ValueError, "safe_z_mm > grasp_z_mm"):
            parse_config(raw)

    def test_loop_max_cycles_must_be_positive(self) -> None:
        raw = valid_config()
        raw["loop"]["max_cycles"] = 0

        with self.assertRaisesRegex(ValueError, "loop.max_cycles"):
            parse_config(raw)

    def test_parse_does_not_mutate_input(self) -> None:
        raw = valid_config()
        expected = copy.deepcopy(raw)
        parse_config(raw)
        self.assertEqual(raw, expected)


if __name__ == "__main__":
    unittest.main()
