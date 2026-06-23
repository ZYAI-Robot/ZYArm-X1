from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from config_loader import CameraToToolConfig, MappingConfig, Pose6D
from pose_mapping import TargetOffsetError, map_marker_to_target_xy


class PoseMappingTests(unittest.TestCase):
    def test_camera_xy_maps_to_base_xy_with_offsets(self) -> None:
        observe = Pose6D(180.0, 0.0, 80.0, 0.0, 0.0, 0.0)
        mapping = MappingConfig(
            reference_camera_xy_mm=(1.0, 2.0),
            camera_xy_to_base_xy=((0.0, -1.0), (-1.0, 0.0)),
            grasp_offset_base_xy_mm=(3.0, -4.0),
            max_xy_offset_mm=80.0,
        )

        target_x, target_y, delta_camera, delta_base = map_marker_to_target_xy(
            (11.0, 22.0, 100.0),
            observe_pose=observe,
            mapping=mapping,
        )

        self.assertEqual(delta_camera, (10.0, 20.0))
        self.assertEqual(delta_base, (-20.0, -10.0))
        self.assertEqual(target_x, 163.0)
        self.assertEqual(target_y, -14.0)

    def test_camera_to_tool_transform_uses_full_tvec(self) -> None:
        observe = Pose6D(200.0, 0.0, 200.0, 0.0, -30.0, 0.0)
        mapping = MappingConfig(
            reference_camera_xy_mm=None,
            camera_xy_to_base_xy=None,
            grasp_offset_base_xy_mm=(0.0, 0.0),
            max_xy_offset_mm=80.0,
            camera_to_tool=CameraToToolConfig(
                translation_mm=(54.8, 0.0, 56.8),
                rotation_deg=(0.0, 30.0, 0.0),
            ),
            opencv_to_camera_rotation=((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
        )

        target_x, target_y, delta_camera, delta_base = map_marker_to_target_xy(
            (20.0, -30.0, 200.0),
            observe_pose=observe,
            mapping=mapping,
        )

        self.assertEqual(delta_camera, (20.0, -30.0))
        self.assertAlmostEqual(delta_base[0], 49.058192, places=6)
        self.assertAlmostEqual(delta_base[1], -20.0, places=6)
        self.assertAlmostEqual(target_x, 249.058192, places=6)
        self.assertAlmostEqual(target_y, -20.0, places=6)

    def test_camera_to_tool_transform_uses_observe_pose_rotation(self) -> None:
        mapping = MappingConfig(
            reference_camera_xy_mm=None,
            camera_xy_to_base_xy=None,
            grasp_offset_base_xy_mm=(0.0, 0.0),
            max_xy_offset_mm=80.0,
            camera_to_tool=CameraToToolConfig(
                translation_mm=(54.8, 0.0, 56.8),
                rotation_deg=(0.0, 30.0, 0.0),
            ),
            opencv_to_camera_rotation=((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
        )

        target_x, _target_y, _delta_camera, delta_base = map_marker_to_target_xy(
            (0.0, 0.0, 200.0),
            observe_pose=Pose6D(200.0, 0.0, 200.0, 0.0, -20.0, 0.0),
            mapping=mapping,
        )

        self.assertAlmostEqual(delta_base[0], -2.661224, places=6)
        self.assertAlmostEqual(target_x, 197.338776, places=6)

    def test_offset_over_limit_is_rejected(self) -> None:
        observe = Pose6D(0.0, 0.0, 80.0, 0.0, 0.0, 0.0)
        mapping = MappingConfig(
            reference_camera_xy_mm=(0.0, 0.0),
            camera_xy_to_base_xy=((1.0, 0.0), (0.0, 1.0)),
            grasp_offset_base_xy_mm=(0.0, 0.0),
            max_xy_offset_mm=10.0,
        )

        with self.assertRaisesRegex(TargetOffsetError, "max_xy_offset"):
            map_marker_to_target_xy((11.0, 0.0, 100.0), observe_pose=observe, mapping=mapping)

    def test_non_finite_tvec_is_rejected(self) -> None:
        observe = Pose6D(0.0, 0.0, 80.0, 0.0, 0.0, 0.0)
        mapping = MappingConfig(
            reference_camera_xy_mm=(0.0, 0.0),
            camera_xy_to_base_xy=((1.0, 0.0), (0.0, 1.0)),
            grasp_offset_base_xy_mm=(0.0, 0.0),
            max_xy_offset_mm=10.0,
        )

        with self.assertRaisesRegex(TargetOffsetError, "finite"):
            map_marker_to_target_xy((float("inf"), 0.0, 100.0), observe_pose=observe, mapping=mapping)


if __name__ == "__main__":
    unittest.main()
