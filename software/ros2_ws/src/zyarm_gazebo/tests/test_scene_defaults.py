import os
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROBOT_BASE_Z_OFFSET_METERS = 0.1038


def _share_dir() -> Path:
    return Path(__file__).resolve().parents[1]


def test_world_config_places_robot_on_table_work_edge():
    config_path = _share_dir() / "config" / "world.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    table_x, table_y, table_z, *_ = config["table"]["pose"]
    table_size_x, table_size_y, table_size_z = config["table"]["size"]
    robot_x, robot_y, robot_z, *_ = config["robot_base_pose"]

    table_min_x = table_x - (table_size_x / 2.0)
    table_max_x = table_x + (table_size_x / 2.0)
    table_min_y = table_y - (table_size_y / 2.0)
    table_max_y = table_y + (table_size_y / 2.0)
    table_top_z = table_z + (table_size_z / 2.0)

    assert table_min_x <= robot_x <= table_max_x
    assert table_min_y <= robot_y <= table_max_y
    assert robot_z == table_top_z + ROBOT_BASE_Z_OFFSET_METERS


def test_pick_place_world_template_declares_default_gui_camera():
    world_template = (_share_dir() / "worlds" / "pick_place_world.sdf").read_text(encoding="utf-8")

    assert "<gui" in world_template
    assert "<camera name=\"user_camera\">" in world_template
    assert "<view_controller>orbit</view_controller>" in world_template


def test_pick_place_world_template_declares_contact_system_for_hybrid_grasp():
    world_template = (_share_dir() / "worlds" / "pick_place_world.sdf").read_text(encoding="utf-8")

    assert 'filename="gz-sim-contact-system"' in world_template
    assert 'name="gz::sim::systems::Contact"' in world_template


def test_pick_place_world_template_uses_high_cube_friction_for_grasp_stability():
    world_template = (_share_dir() / "worlds" / "pick_place_world.sdf").read_text(encoding="utf-8")

    assert "<mu>5.0</mu>" in world_template
    assert "<mu2>5.0</mu2>" in world_template


def test_pickup_cube_model_uses_high_friction_for_grasp_stability():
    cube_model = (_share_dir() / "models" / "pickup_cube" / "model.sdf").read_text(encoding="utf-8")

    assert "<mu>5.0</mu>" in cube_model
    assert "<mu2>5.0</mu2>" in cube_model


def test_package_manifest_declares_grasp_manager_runtime_dependencies():
    package_root = ET.parse(_share_dir() / "package.xml").getroot()
    dependencies = {element.text for element in package_root.findall("exec_depend")}

    assert "control_msgs" in dependencies
    assert "rclpy" in dependencies
    assert "ros_gz_interfaces" in dependencies
    assert "sensor_msgs" in dependencies
    assert "std_msgs" in dependencies
    assert "trajectory_msgs" in dependencies


def test_cmakelists_installs_grasp_manager_script():
    cmake_text = (_share_dir() / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "scripts/grasp_manager.py" in cmake_text


def test_grasp_manager_script_is_executable():
    script_path = _share_dir() / "scripts" / "grasp_manager.py"

    assert os.access(script_path, os.X_OK)
