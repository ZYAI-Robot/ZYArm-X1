import xml.etree.ElementTree as ET
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


def _load_launch_module():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "bringup_pick_place_world.launch.py"
    )
    spec = spec_from_file_location("zyarm_gazebo_bringup_launch", launch_path)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_build_gz_resource_path_includes_description_share_parent_and_existing_path():
    module = _load_launch_module()

    resource_path = module._build_gz_resource_path(
        "/tmp/rendered_models",
        "/workspace/install/share",
        existing_resource_path="/opt/ros/jazzy/share",
    )

    assert resource_path == "/tmp/rendered_models:/workspace/install/share:/opt/ros/jazzy/share"


def test_build_spawn_arguments_use_rendered_model_file_and_robot_base_pose():
    module = _load_launch_module()

    arguments = module._build_spawn_arguments(
        "/tmp/zyarm_x1_standard/model.sdf",
        [0.0, 0.1, 0.4, 0.0, 0.2, -1.57]
    )

    assert arguments == [
        "-file",
        "/tmp/zyarm_x1_standard/model.sdf",
        "-x",
        "0.0",
        "-y",
        "0.1",
        "-z",
        "0.4",
        "-R",
        "0.0",
        "-P",
        "0.2",
        "-Y",
        "-1.57",
    ]


def test_build_bridge_arguments_include_clock_and_dual_cameras():
    module = _load_launch_module()

    arguments = module._build_bridge_arguments()

    assert arguments[:3] == [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/camera_fixed/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        "/camera_wrist/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
    ]


def test_build_bridge_arguments_include_dual_touch_topics():
    module = _load_launch_module()

    arguments = module._build_bridge_arguments()

    assert (
        "/world/pick_place_world/model/zyarm_x1_standard/link/claw1/sensor/claw1_touch/contact"
        "@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
    ) in arguments
    assert (
        "/world/pick_place_world/model/zyarm_x1_standard/link/claw2/sensor/claw2_touch/contact"
        "@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
    ) in arguments
    assert "/zyarm/grasp/touched@std_msgs/msg/Bool[gz.msgs.Boolean" not in arguments


def test_build_bridge_arguments_include_attach_and_detach_topics():
    module = _load_launch_module()

    arguments = module._build_bridge_arguments()

    assert "/zyarm/grasp/attach@std_msgs/msg/Empty]gz.msgs.Empty" in arguments
    assert "/zyarm/grasp/detach@std_msgs/msg/Empty]gz.msgs.Empty" in arguments
    assert "/zyarm/grasp/state@std_msgs/msg/String[gz.msgs.StringMsg" in arguments


def test_build_bridge_parameters_make_grasp_state_publisher_transient_local():
    module = _load_launch_module()

    parameters = module._build_bridge_parameters()

    assert parameters == [
        {"qos_overrides./zyarm/grasp/state.publisher.durability": "transient_local"}
    ]


def test_build_grasp_manager_parameters_default_to_attach_mode():
    module = _load_launch_module()

    parameters = module._build_grasp_manager_parameters()[0]

    assert parameters["grasp_state_topic"] == "/zyarm/grasp/state"
    assert parameters["grasp_mode"] == "attach"
    assert parameters["attach_allowed_targets"] == ["pickup_cube::cube_link"]
    assert parameters["attach_contact_debounce_sec"] == 0.03
    assert parameters["detach_open_threshold"] == 0.028
    assert parameters["detach_open_debounce_sec"] == 0.05


def test_inject_dual_touch_features_rejects_missing_end_effector_parent_link():
    module = _load_launch_module()
    base_sdf = """
    <sdf version='1.11'>
      <model name='zyarm_x1_standard'>
        <link name='claw1'><collision name='claw1_collision'/></link>
        <link name='claw2'><collision name='claw2_collision'/></link>
      </model>
    </sdf>
    """

    try:
        module._inject_dual_touch_features(base_sdf)
    except ValueError as exc:
        assert "ee_link" in str(exc)
        assert "link6" in str(exc)
    else:
        raise AssertionError("Expected end-effector parent validation to raise ValueError")


def test_inject_dual_touch_features_adds_contact_sensors_to_both_claws():
    module = _load_launch_module()
    base_sdf = """
    <sdf version='1.11'>
      <model name='zyarm_x1_standard'>
        <link name='ee_link'/>
        <link name='claw1'>
          <collision name='claw1_collision'/>
        </link>
        <link name='claw2'>
          <collision name='claw2_collision'/>
        </link>
      </model>
    </sdf>
    """

    rendered = module._inject_dual_touch_features(base_sdf)
    root = ET.fromstring(rendered)

    left_sensor = root.find(".//link[@name='claw1']/sensor[@name='claw1_touch']")
    right_sensor = root.find(".//link[@name='claw2']/sensor[@name='claw2_touch']")
    assert left_sensor is not None
    assert right_sensor is not None
    assert left_sensor.findtext("./contact/collision") == "claw1_collision"
    assert right_sensor.findtext("./contact/collision") == "claw2_collision"
    assert root.findtext(".//link[@name='claw1']/collision[@name='claw1_collision']/surface/friction/ode/mu") == "5.0"
    assert root.findtext(".//link[@name='claw1']/collision[@name='claw1_collision']/surface/friction/ode/mu2") == "5.0"
    assert root.findtext(".//link[@name='claw2']/collision[@name='claw2_collision']/surface/friction/ode/mu") == "5.0"
    assert root.findtext(".//link[@name='claw2']/collision[@name='claw2_collision']/surface/friction/ode/mu2") == "5.0"


def test_inject_dual_touch_features_adds_detachable_joint_for_pickup_cube():
    module = _load_launch_module()
    base_sdf = """
    <sdf version='1.11'>
      <model name='zyarm_x1_standard'>
        <link name='ee_link'/>
        <link name='claw1'><collision name='claw1_collision'/></link>
        <link name='claw2'><collision name='claw2_collision'/></link>
      </model>
    </sdf>
    """

    rendered = module._inject_dual_touch_features(base_sdf)
    root = ET.fromstring(rendered)

    plugin = root.find(".//plugin[@name='gz::sim::systems::DetachableJoint']")
    assert plugin is not None
    assert plugin.attrib["filename"] == "gz-sim-detachable-joint-system"
    assert plugin.findtext("parent_link") == "ee_link"
    assert plugin.findtext("child_model") == "pickup_cube"
    assert plugin.findtext("child_link") == "cube_link"
    assert plugin.findtext("attach_topic") == "/zyarm/grasp/attach"
    assert plugin.findtext("detach_topic") == "/zyarm/grasp/detach"


def test_inject_dual_touch_features_falls_back_to_link6_when_ee_link_is_absent():
    module = _load_launch_module()
    base_sdf = """
    <sdf version='1.11'>
      <model name='zyarm_x1_standard'>
        <link name='link6'/>
        <link name='claw1'><collision name='claw1_collision'/></link>
        <link name='claw2'><collision name='claw2_collision'/></link>
      </model>
    </sdf>
    """

    rendered = module._inject_dual_touch_features(base_sdf)
    root = ET.fromstring(rendered)

    plugin = root.find(".//plugin[@name='gz::sim::systems::DetachableJoint']")
    assert plugin is not None
    assert plugin.findtext("parent_link") == "link6"


def test_build_grasp_manager_node_uses_expected_script():
    module = _load_launch_module()
    parameters = [{"sentinel": True}]
    calls = []

    def fake_build_grasp_manager_parameters():
        calls.append(True)
        return parameters

    module._build_grasp_manager_parameters = fake_build_grasp_manager_parameters
    node = module._build_grasp_manager_node()

    assert calls == [True]
    assert len(node._Node__parameters) == 1
    payload = node._Node__parameters[0]
    key = next(iter(payload))
    assert len(key) == 1
    assert key[0]._TextSubstitution__text == "sentinel"
    assert payload[key] is True


def test_prepare_rendered_assets_embed_robot_include_and_world_joint():
    module = _load_launch_module()
    share_dir = Path(__file__).resolve().parents[1]

    _render_root, world_path, config = module._prepare_rendered_assets(share_dir)
    rendered_world = world_path.read_text(encoding="utf-8")

    assert "<uri>model://zyarm_x1_standard</uri>" in rendered_world
    assert "<name>zyarm_x1_standard</name>" in rendered_world
    assert "<child>zyarm_x1_standard::base_link</child>" in rendered_world
    assert "<pose>0.12 0.0 0.5038 0.0 0.0 0.0</pose>" in rendered_world
