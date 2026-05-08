from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest


def _load_grasp_manager_module():
    script_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "grasp_manager.py"
    )
    spec = spec_from_file_location("zyarm_gazebo_grasp_manager", script_path)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _FakeCollision:
    def __init__(self, name):
        self.name = name


class _FakeContact:
    def __init__(self, collision1, collision2):
        self.collision1 = _FakeCollision(collision1)
        self.collision2 = _FakeCollision(collision2)


class _FakeContactsMessage:
    def __init__(self, contacts):
        self.contacts = contacts


def test_no_hold_without_touch_ready():
    module = _load_grasp_manager_module()

    action, candidate_since = module.evaluate_hold_transition(
        holding=False,
        touch_ready=False,
        now_sec=10.0,
        candidate_since_sec=None,
        debounce_sec=0.1,
    )

    assert action is None
    assert candidate_since is None


def test_hold_after_touch_ready_and_debounce():
    module = _load_grasp_manager_module()

    action, candidate_since = module.evaluate_hold_transition(
        holding=False,
        touch_ready=True,
        now_sec=10.2,
        candidate_since_sec=10.0,
        debounce_sec=0.1,
    )

    assert action == "hold"
    assert candidate_since == 10.0


def test_hold_touch_ready_accepts_single_touch_when_gripper_is_narrow():
    module = _load_grasp_manager_module()

    assert module.is_hold_touch_ready(
        left_touch=True,
        right_touch=False,
        joint6=0.020,
        joint7=0.021,
        single_touch_max_total_width=0.05,
    )


def test_hold_touch_ready_rejects_single_touch_when_gripper_is_still_wide():
    module = _load_grasp_manager_module()

    assert not module.is_hold_touch_ready(
        left_touch=True,
        right_touch=False,
        joint6=0.030,
        joint7=0.028,
        single_touch_max_total_width=0.05,
    )


def test_has_external_contact_accepts_non_ignored_collision():
    module = _load_grasp_manager_module()

    assert module.has_external_contact(
        [
            (
                "zyarm_x1_standard::claw1::claw1_collision",
                "pickup_cube::cube_link::collision",
            )
        ],
        ignored_name_fragments=("zyarm_x1_standard::", "table", "ground_plane", "goal_zone"),
    )

def test_has_external_contact_ignores_robot_and_table_contacts():
    module = _load_grasp_manager_module()

    assert not module.has_external_contact(
        [
            (
                "zyarm_x1_standard::claw1::claw1_collision",
                "table::link::collision",
            ),
            (
                "zyarm_x1_standard::claw1::claw1_collision",
                "zyarm_x1_standard::link6::collision",
            ),
        ],
        ignored_name_fragments=("zyarm_x1_standard::", "table", "ground_plane", "goal_zone"),
    )

def test_contacts_message_targets_and_touch_handle_fully_qualified_external_names():
    module = _load_grasp_manager_module()
    message = _FakeContactsMessage(
        [
            _FakeContact(
                "zyarm_x1_standard::claw1::claw1_collision",
                "pickup_cube::cube_link::collision",
            )
        ]
    )
    assert module._contacts_message_targets(
        message,
        ignored_name_fragments=("zyarm_x1_standard::", "table", "ground_plane", "goal_zone"),
    ) == ("pickup_cube::cube_link",)
    assert module._contacts_message_has_external_touch(
        message,
        ignored_name_fragments=("zyarm_x1_standard::", "table", "ground_plane", "goal_zone"),
    )


def test_contacts_message_targets_and_touch_pin_malformed_external_names_as_empty():
    module = _load_grasp_manager_module()
    message = _FakeContactsMessage(
        [
            _FakeContact(
                "zyarm_x1_standard::claw1::claw1_collision",
                "pickup_cube",
            )
        ]
    )

    assert module._contacts_message_targets(
        message,
        ignored_name_fragments=("zyarm_x1_standard::", "table", "ground_plane", "goal_zone"),
    ) == ()
    assert not module._contacts_message_has_external_touch(
        message,
        ignored_name_fragments=("zyarm_x1_standard::", "table", "ground_plane", "goal_zone"),
    )
def test_extract_external_contact_targets_returns_unique_model_link_names():
    module = _load_grasp_manager_module()

    targets = module.extract_external_contact_targets(
        [
            ("zyarm_x1_standard::claw1::claw1_collision", "pickup_cube::cube_link::collision"),
            ("pickup_cube::cube_link::collision", "zyarm_x1_standard::claw1::claw1_collision"),
            ("table::link::collision", "zyarm_x1_standard::claw1::claw1_collision"),
        ],
        ignored_name_fragments=("zyarm_x1_standard::", "table", "ground_plane", "goal_zone"),
    )

    assert targets == ("pickup_cube::cube_link",)


def test_select_attach_target_requires_dual_touch_match_and_allowed_target():
    module = _load_grasp_manager_module()

    assert module.select_attach_target(
        left_targets=("pickup_cube::cube_link",),
        right_targets=("pickup_cube::cube_link", "goal_zone::zone"),
        allowed_targets=("pickup_cube::cube_link",),
    ) == "pickup_cube::cube_link"

    assert module.select_attach_target(
        left_targets=("pickup_cube::cube_link",),
        right_targets=("goal_zone::zone",),
        allowed_targets=("pickup_cube::cube_link",),
    ) is None


def test_evaluate_attach_transition_requires_ready_state_and_debounce():
    module = _load_grasp_manager_module()

    action, candidate_since = module.evaluate_attach_transition(
        attached=False,
        attach_ready=True,
        now_sec=10.0,
        candidate_since_sec=None,
        debounce_sec=0.03,
    )
    assert action is None
    assert candidate_since == 10.0

    action, candidate_since = module.evaluate_attach_transition(
        attached=False,
        attach_ready=True,
        now_sec=10.04,
        candidate_since_sec=10.0,
        debounce_sec=0.03,
    )
    assert action == "attach"
    assert candidate_since == 10.0


def test_is_opening_command_requires_both_fingers_above_threshold():
    module = _load_grasp_manager_module()

    assert module.is_opening_command(
        reference6=0.030,
        reference7=0.031,
        detach_open_threshold=0.028,
    )
    assert not module.is_opening_command(
        reference6=0.030,
        reference7=0.010,
        detach_open_threshold=0.028,
    )


def test_evaluate_detach_transition_only_releases_after_dual_open_debounce():
    module = _load_grasp_manager_module()

    action, candidate_since = module.evaluate_detach_transition(
        attached=True,
        detach_ready=True,
        now_sec=12.0,
        candidate_since_sec=None,
        debounce_sec=0.05,
    )
    assert action is None
    assert candidate_since == 12.0

    action, candidate_since = module.evaluate_detach_transition(
        attached=True,
        detach_ready=True,
        now_sec=12.06,
        candidate_since_sec=12.0,
        debounce_sec=0.05,
    )
    assert action == "detach"
    assert candidate_since == 12.0

    action, candidate_since = module.evaluate_detach_transition(
        attached=True,
        detach_ready=False,
        now_sec=12.06,
        candidate_since_sec=12.0,
        debounce_sec=0.05,
    )
    assert action is None
    assert candidate_since is None


def test_evaluate_attach_mode_action_emits_attach_after_dual_touch_debounce():
    module = _load_grasp_manager_module()

    action, attach_candidate_since, detach_candidate_since = module.evaluate_attach_mode_action(
        attached=False,
        attach_target="pickup_cube::cube_link",
        closing_command=True,
        opening_command=False,
        now_sec=20.04,
        attach_candidate_since_sec=20.0,
        detach_candidate_since_sec=None,
        attach_contact_debounce_sec=0.03,
        detach_open_debounce_sec=0.05,
    )

    assert action == "attach"
    assert attach_candidate_since == 20.0
    assert detach_candidate_since is None


def test_evaluate_attach_mode_action_detaches_only_after_dual_open_debounce():
    module = _load_grasp_manager_module()

    action, attach_candidate_since, detach_candidate_since = module.evaluate_attach_mode_action(
        attached=True,
        attach_target="pickup_cube::cube_link",
        closing_command=False,
        opening_command=True,
        now_sec=21.06,
        attach_candidate_since_sec=None,
        detach_candidate_since_sec=21.0,
        attach_contact_debounce_sec=0.03,
        detach_open_debounce_sec=0.05,
    )

    assert action == "detach"
    assert attach_candidate_since is None
    assert detach_candidate_since == 21.0


def test_apply_grasp_state_feedback_marks_attached_from_authoritative_state():
    module = _load_grasp_manager_module()

    attached, attached_target = module.apply_grasp_state_feedback(
        attached=False,
        attached_target="pickup_cube::cube_link",
        grasp_state="attached",
    )

    assert attached is True
    assert attached_target == "pickup_cube::cube_link"


def test_apply_grasp_state_feedback_clears_target_on_authoritative_detach():
    module = _load_grasp_manager_module()

    attached, attached_target = module.apply_grasp_state_feedback(
        attached=True,
        attached_target="pickup_cube::cube_link",
        grasp_state="detached",
    )

    assert attached is False
    assert attached_target is None


def test_on_grasp_state_clears_startup_detach_pending_on_authoritative_detach():
    module = _load_grasp_manager_module()

    class _FakeManager:
        def __init__(self):
            self._attached = True
            self._attached_target = "pickup_cube::cube_link"
            self._startup_detach_pending = True
            self.update_calls = 0

        def _maybe_update_grasp_state(self):
            self.update_calls += 1

    class _FakeMessage:
        data = "detached"

    manager = _FakeManager()

    module.GraspManager._on_grasp_state(manager, _FakeMessage())

    assert manager._attached is False
    assert manager._attached_target is None
    assert manager._startup_detach_pending is False
    assert manager.update_calls == 1


def test_on_startup_detach_timer_publishes_initial_detach_while_pending():
    module = _load_grasp_manager_module()

    class _FakeClock:
        @property
        def nanoseconds(self):
            return 5_000_000_000

    class _FakeNow:
        @property
        def nanoseconds(self):
            return 5_000_000_000

    class _FakeClockWrapper:
        def now(self):
            return _FakeNow()

    class _FakeManager:
        def __init__(self):
            self._startup_detach_pending = True
            self._startup_detach_deadline_sec = None
            self.detach_calls = 0

        def get_clock(self):
            return _FakeClockWrapper()

        def _publish_detach_command(self):
            self.detach_calls += 1

    manager = _FakeManager()

    module.GraspManager._on_startup_detach_timer(manager)

    assert manager.detach_calls == 1
    assert manager._startup_detach_pending is True
    assert manager._startup_detach_deadline_sec is not None


def test_on_startup_detach_timer_stops_retrying_after_deadline():
    module = _load_grasp_manager_module()

    class _FakeNow:
        @property
        def nanoseconds(self):
            return 8_000_000_000

    class _FakeClockWrapper:
        def now(self):
            return _FakeNow()

    class _FakeManager:
        def __init__(self):
            self._startup_detach_pending = True
            self._startup_detach_deadline_sec = 7.0
            self.detach_calls = 0

        def get_clock(self):
            return _FakeClockWrapper()

        def _publish_detach_command(self):
            self.detach_calls += 1

    manager = _FakeManager()

    module.GraspManager._on_startup_detach_timer(manager)

    assert manager.detach_calls == 0
    assert manager._startup_detach_pending is False


def test_build_grasp_state_subscription_qos_uses_transient_local_durability():
    module = _load_grasp_manager_module()

    class _FakeQoSProfile:
        def __init__(self, *, depth, durability):
            self.depth = depth
            self.durability = durability

    class _FakeQoSDurabilityPolicy:
        TRANSIENT_LOCAL = "transient_local"

    module.QoSProfile = _FakeQoSProfile
    module.QoSDurabilityPolicy = _FakeQoSDurabilityPolicy

    qos = module._build_grasp_state_subscription_qos()

    assert qos.depth == 1
    assert qos.durability == "transient_local"


def test_publish_attach_command_does_not_flip_attached_without_confirmation():
    module = _load_grasp_manager_module()

    class _FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class _FakeManager:
        pass

    manager = _FakeManager()
    manager._attach_pub = _FakePublisher()
    manager._attached = False
    manager._attached_target = None
    manager._attach_candidate_since_sec = 20.0
    manager._detach_candidate_since_sec = 21.0
    manager._last_action = "none"

    module.GraspManager._publish_attach_command(manager, "pickup_cube::cube_link")

    assert manager._attached is False
    assert manager._attached_target == "pickup_cube::cube_link"
    assert manager._attach_candidate_since_sec is None
    assert manager._detach_candidate_since_sec is None
    assert manager._last_action == "attach:pickup_cube::cube_link"


def test_publish_detach_command_keeps_attached_until_authoritative_feedback():
    module = _load_grasp_manager_module()

    class _FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class _FakeManager:
        pass

    manager = _FakeManager()
    manager._detach_pub = _FakePublisher()
    manager._attached = True
    manager._attached_target = "pickup_cube::cube_link"
    manager._attach_candidate_since_sec = 20.0
    manager._detach_candidate_since_sec = 21.0
    manager._last_action = "attach:pickup_cube::cube_link"

    module.GraspManager._publish_detach_command(manager)

    assert manager._attached is True
    assert manager._attached_target == "pickup_cube::cube_link"
    assert manager._attach_candidate_since_sec is None
    assert manager._detach_candidate_since_sec is None
    assert manager._last_action == "detach"


def test_compute_symmetric_hold_target_applies_preload():
    module = _load_grasp_manager_module()

    hold_target = module.compute_symmetric_hold_target(
        joint6=0.006002918204974531,
        joint7=0.034000000001462354,
        preload=0.001,
        max_joint_position=0.034,
    )

    assert hold_target == pytest.approx(0.019001459103218444)


def test_symmetric_hold_reasserts_for_tighter_close_command():
    module = _load_grasp_manager_module()

    action = module.evaluate_symmetric_hold_action(
        reference6=0.0001,
        reference7=0.0001,
        hold_target=0.019001459103218444,
        release_margin=0.003,
        command_tolerance=1e-4,
    )

    assert action == "hold"


def test_symmetric_hold_releases_when_command_opens_past_hold_margin():
    module = _load_grasp_manager_module()

    action = module.evaluate_symmetric_hold_action(
        reference6=0.034,
        reference7=0.034,
        hold_target=0.019001459103218444,
        release_margin=0.003,
        command_tolerance=1e-4,
    )

    assert action == "release"


def test_should_log_asymmetry_snapshot_when_gap_exceeds_threshold():
    module = _load_grasp_manager_module()

    assert module.should_log_asymmetry_snapshot(
        feedback6=0.0,
        feedback7=0.0339,
        threshold=0.005,
        now_sec=10.0,
        last_logged_sec=None,
        cooldown_sec=1.0,
    )


def test_should_not_log_asymmetry_snapshot_during_cooldown_or_below_threshold():
    module = _load_grasp_manager_module()

    assert not module.should_log_asymmetry_snapshot(
        feedback6=0.01,
        feedback7=0.013,
        threshold=0.005,
        now_sec=10.0,
        last_logged_sec=None,
        cooldown_sec=1.0,
    )
    assert not module.should_log_asymmetry_snapshot(
        feedback6=0.0,
        feedback7=0.0339,
        threshold=0.005,
        now_sec=10.5,
        last_logged_sec=10.0,
        cooldown_sec=1.0,
    )


def test_format_asymmetry_snapshot_includes_core_diagnostics():
    module = _load_grasp_manager_module()

    snapshot = module.format_asymmetry_snapshot(
        now_sec=12.5,
        holding=True,
        left_touch=True,
        right_touch=True,
        hold_target=None,
        last_hold_command_sec=None,
        joint6=0.0,
        joint7=0.0339,
        reference6=0.0339,
        reference7=0.0339,
        error6=0.0339,
        error7=0.0,
        last_action="none",
    )

    assert "asymmetry snapshot" in snapshot
    assert "holding=True" in snapshot
    assert "left_touch=True" in snapshot
    assert "right_touch=True" in snapshot
    assert "joint=[0.000000, 0.033900]" in snapshot
    assert "reference=[0.033900, 0.033900]" in snapshot
    assert "last_action=none" in snapshot
