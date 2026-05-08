#!/usr/bin/env python3
from __future__ import annotations

from typing import Optional

try:
    from builtin_interfaces.msg import Duration
    from control_msgs.msg import JointTrajectoryControllerState
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile
    from ros_gz_interfaces.msg import Contacts
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Empty, String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except ImportError:  # pragma: no cover - allows pure-function tests without ROS sourced
    Duration = None
    JointTrajectoryControllerState = None
    rclpy = None
    Node = object
    QoSDurabilityPolicy = None
    QoSProfile = None
    Contacts = None
    JointState = None
    Empty = None
    String = None
    JointTrajectory = None
    JointTrajectoryPoint = None

_STARTUP_DETACH_PERIOD_SEC = 0.25
_STARTUP_DETACH_TIMEOUT_SEC = 5.0


def evaluate_hold_transition(
    *,
    holding: bool,
    touch_ready: bool,
    now_sec: float,
    candidate_since_sec: Optional[float],
    debounce_sec: float,
):
    if holding:
        return None, candidate_since_sec

    if not touch_ready:
        return None, None

    if candidate_since_sec is None:
        return None, now_sec

    if (now_sec - candidate_since_sec) >= debounce_sec:
        return "hold", candidate_since_sec

    return None, candidate_since_sec


def is_hold_touch_ready(
    *,
    left_touch: bool,
    right_touch: bool,
    joint6: float,
    joint7: float,
    single_touch_max_total_width: float,
):
    if left_touch and right_touch:
        return True

    if not (left_touch or right_touch):
        return False

    return (joint6 + joint7) <= single_touch_max_total_width


def has_external_contact(name_pairs, ignored_name_fragments):
    for collision1, collision2 in name_pairs:
        if not any(fragment in collision1 for fragment in ignored_name_fragments):
            return True
        if not any(fragment in collision2 for fragment in ignored_name_fragments):
            return True
    return False


def _collision_to_target(collision_name):
    parts = collision_name.split("::")
    if len(parts) < 2:
        return None
    return "::".join(parts[:2])


def extract_external_contact_targets(name_pairs, ignored_name_fragments):
    targets = []
    for collision1, collision2 in name_pairs:
        for collision_name in (collision1, collision2):
            if any(fragment in collision_name for fragment in ignored_name_fragments):
                continue
            target = _collision_to_target(collision_name)
            if target is None or target in targets:
                continue
            targets.append(target)
    return tuple(targets)


def select_attach_target(*, left_targets, right_targets, allowed_targets):
    for target in allowed_targets:
        if target in left_targets and target in right_targets:
            return target
    return None


def evaluate_attach_transition(
    *,
    attached: bool,
    attach_ready: bool,
    now_sec: float,
    candidate_since_sec: Optional[float],
    debounce_sec: float,
):
    if attached:
        return None, candidate_since_sec

    if not attach_ready:
        return None, None

    if candidate_since_sec is None:
        return None, now_sec

    if (now_sec - candidate_since_sec) >= debounce_sec:
        return "attach", candidate_since_sec

    return None, candidate_since_sec


def is_opening_command(*, reference6: float, reference7: float, detach_open_threshold: float):
    return reference6 >= detach_open_threshold and reference7 >= detach_open_threshold


def evaluate_detach_transition(
    *,
    attached: bool,
    detach_ready: bool,
    now_sec: float,
    candidate_since_sec: Optional[float],
    debounce_sec: float,
):
    if not attached:
        return None, None

    if not detach_ready:
        return None, None

    if candidate_since_sec is None:
        return None, now_sec

    if (now_sec - candidate_since_sec) >= debounce_sec:
        return "detach", candidate_since_sec

    return None, candidate_since_sec


def evaluate_attach_mode_action(
    *,
    attached: bool,
    attach_target: Optional[str],
    closing_command: bool,
    opening_command: bool,
    now_sec: float,
    attach_candidate_since_sec: Optional[float],
    detach_candidate_since_sec: Optional[float],
    attach_contact_debounce_sec: float,
    detach_open_debounce_sec: float,
):
    if attached:
        action, detach_candidate_since_sec = evaluate_detach_transition(
            attached=attached,
            detach_ready=opening_command,
            now_sec=now_sec,
            candidate_since_sec=detach_candidate_since_sec,
            debounce_sec=detach_open_debounce_sec,
        )
        return action, None, detach_candidate_since_sec

    action, attach_candidate_since_sec = evaluate_attach_transition(
        attached=attached,
        attach_ready=(attach_target is not None and closing_command),
        now_sec=now_sec,
        candidate_since_sec=attach_candidate_since_sec,
        debounce_sec=attach_contact_debounce_sec,
    )
    return action, attach_candidate_since_sec, None


def apply_grasp_state_feedback(
    *,
    attached: bool,
    attached_target: Optional[str],
    grasp_state: str,
):
    normalized_state = str(grasp_state).strip().lower()
    if normalized_state == "attached":
        return True, attached_target
    if normalized_state == "detached":
        return False, None
    return attached, attached_target


def _build_grasp_state_subscription_qos():
    if QoSProfile is None or QoSDurabilityPolicy is None:
        raise RuntimeError("Transient-local grasp-state QoS requires ROS 2 QoS support")

    return QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def compute_symmetric_hold_target(
    *,
    joint6: float,
    joint7: float,
    preload: float,
    max_joint_position: float,
):
    clamped_total_width = max(0.0, min(joint6 + joint7, max_joint_position * 2.0))
    return max(0.0, min((clamped_total_width / 2.0) - preload, max_joint_position))


def evaluate_symmetric_hold_action(
    *,
    reference6: float,
    reference7: float,
    hold_target: float,
    release_margin: float,
    command_tolerance: float,
):
    if (
        reference6 >= (hold_target + release_margin)
        and reference7 >= (hold_target + release_margin)
    ):
        return "release"

    if (
        reference6 < (hold_target - command_tolerance)
        or reference7 < (hold_target - command_tolerance)
    ):
        return "hold"

    return None


def should_log_asymmetry_snapshot(
    *,
    feedback6: float,
    feedback7: float,
    threshold: float,
    now_sec: float,
    last_logged_sec: Optional[float],
    cooldown_sec: float,
):
    if abs(feedback6 - feedback7) < threshold:
        return False

    if last_logged_sec is None:
        return True

    return (now_sec - last_logged_sec) >= cooldown_sec


def _format_optional_float(value: Optional[float]):
    if value is None:
        return "None"
    return f"{value:.6f}"


def format_asymmetry_snapshot(
    *,
    now_sec: float,
    holding: bool,
    left_touch: bool,
    right_touch: bool,
    hold_target: Optional[float],
    last_hold_command_sec: Optional[float],
    joint6: float,
    joint7: float,
    reference6: float,
    reference7: float,
    error6: float,
    error7: float,
    last_action: str,
):
    return (
        f"asymmetry snapshot t={now_sec:.3f} gap={abs(joint6 - joint7):.6f} "
        f"holding={holding} left_touch={left_touch} right_touch={right_touch} "
        f"hold_target={_format_optional_float(hold_target)} "
        f"last_hold_command={_format_optional_float(last_hold_command_sec)} "
        f"joint=[{joint6:.6f}, {joint7:.6f}] "
        f"reference=[{reference6:.6f}, {reference7:.6f}] "
        f"error=[{error6:.6f}, {error7:.6f}] "
        f"last_action={last_action}"
    )


def _lookup_joint_positions(joint_names, positions, joint6_name, joint7_name):
    if len(joint_names) != len(positions):
        return None

    named_positions = dict(zip(joint_names, positions))
    joint6 = named_positions.get(joint6_name)
    joint7 = named_positions.get(joint7_name)
    if joint6 is None or joint7 is None:
        return None
    return float(joint6), float(joint7)


def _contact_name_pairs(message):
    if message is None:
        return []

    return [
        (contact.collision1.name, contact.collision2.name)
        for contact in getattr(message, "contacts", [])
    ]


def _contacts_message_targets(message, ignored_name_fragments):
    return extract_external_contact_targets(
        _contact_name_pairs(message),
        ignored_name_fragments,
    )


def _contacts_message_has_external_touch(message, ignored_name_fragments):
    return bool(_contacts_message_targets(message, ignored_name_fragments))


def _is_closing_command(
    *,
    reference6: float,
    reference7: float,
    joint6: float,
    joint7: float,
    tolerance: float,
):
    return reference6 <= (joint6 + tolerance) and reference7 <= (joint7 + tolerance)


class GraspManager(Node):
    def __init__(self):
        super().__init__("zyarm_grasp_manager")
        self.declare_parameter(
            "left_contact_topic",
            "/world/pick_place_world/model/zyarm_x1_standard/link/claw1/sensor/claw1_touch/contact",
        )
        self.declare_parameter(
            "right_contact_topic",
            "/world/pick_place_world/model/zyarm_x1_standard/link/claw2/sensor/claw2_touch/contact",
        )
        self.declare_parameter("grasp_state_topic", "/zyarm/grasp/state")
        self.declare_parameter("grasp_mode", "attach")
        self.declare_parameter("attach_allowed_targets", ["pickup_cube::cube_link"])
        self.declare_parameter("attach_contact_debounce_sec", 0.03)
        self.declare_parameter("detach_open_threshold", 0.028)
        self.declare_parameter("detach_open_debounce_sec", 0.05)
        self.declare_parameter("contact_debounce_sec", 0.05)
        self.declare_parameter("joint6_name", "joint6")
        self.declare_parameter("joint7_name", "joint7")
        self.declare_parameter("joint_max_position", 0.034)
        self.declare_parameter("single_touch_max_total_width", 0.05)
        self.declare_parameter("hold_preload", 0.001)
        self.declare_parameter("hold_release_margin", 0.003)
        self.declare_parameter("hold_command_tolerance", 1e-4)
        self.declare_parameter("hold_command_duration_sec", 0.25)
        self.declare_parameter(
            "ignored_contact_name_fragments",
            ["zyarm_x1_standard::", "table", "ground_plane", "goal_zone"],
        )
        self.declare_parameter("diagnostic_asymmetry_threshold", 0.005)
        self.declare_parameter("diagnostic_cooldown_sec", 1.0)

        self._left_contact_topic = str(self.get_parameter("left_contact_topic").value)
        self._right_contact_topic = str(self.get_parameter("right_contact_topic").value)
        self._grasp_state_topic = str(self.get_parameter("grasp_state_topic").value)
        self._grasp_mode = str(self.get_parameter("grasp_mode").value)
        self._attach_allowed_targets = tuple(
            str(target) for target in self.get_parameter("attach_allowed_targets").value
        )
        self._attach_contact_debounce_sec = float(
            self.get_parameter("attach_contact_debounce_sec").value
        )
        self._detach_open_threshold = float(self.get_parameter("detach_open_threshold").value)
        self._detach_open_debounce_sec = float(
            self.get_parameter("detach_open_debounce_sec").value
        )
        self._contact_debounce_sec = float(self.get_parameter("contact_debounce_sec").value)
        self._joint6_name = str(self.get_parameter("joint6_name").value)
        self._joint7_name = str(self.get_parameter("joint7_name").value)
        self._joint_max_position = float(self.get_parameter("joint_max_position").value)
        self._single_touch_max_total_width = float(
            self.get_parameter("single_touch_max_total_width").value
        )
        self._hold_preload = float(self.get_parameter("hold_preload").value)
        self._hold_release_margin = float(self.get_parameter("hold_release_margin").value)
        self._hold_command_tolerance = float(self.get_parameter("hold_command_tolerance").value)
        self._hold_command_duration_sec = float(self.get_parameter("hold_command_duration_sec").value)
        self._ignored_name_fragments = tuple(
            str(fragment) for fragment in self.get_parameter("ignored_contact_name_fragments").value
        )
        self._diagnostic_asymmetry_threshold = float(
            self.get_parameter("diagnostic_asymmetry_threshold").value
        )
        self._diagnostic_cooldown_sec = float(self.get_parameter("diagnostic_cooldown_sec").value)

        self._holding = False
        self._hold_target = None
        self._hold_candidate_since_sec = None
        self._attached = False
        self._attached_target = None
        self._attach_candidate_since_sec = None
        self._detach_candidate_since_sec = None
        self._startup_detach_pending = self._grasp_mode == "attach"
        self._startup_detach_deadline_sec = None
        self._left_touch = False
        self._right_touch = False
        self._left_targets = ()
        self._right_targets = ()
        self._joint6 = None
        self._joint7 = None
        self._reference6 = None
        self._reference7 = None
        self._last_asymmetry_log_sec = None
        self._last_hold_command_sec = None
        self._last_action = "none"

        self._hold_pub = self.create_publisher(
            JointTrajectory,
            "/gripper_controller/joint_trajectory",
            10,
        )
        self._attach_pub = self.create_publisher(Empty, "/zyarm/grasp/attach", 10)
        self._detach_pub = self.create_publisher(Empty, "/zyarm/grasp/detach", 10)
        self.create_subscription(Contacts, self._left_contact_topic, self._on_left_contacts, 10)
        self.create_subscription(Contacts, self._right_contact_topic, self._on_right_contacts, 10)
        self.create_subscription(
            String,
            self._grasp_state_topic,
            self._on_grasp_state,
            _build_grasp_state_subscription_qos(),
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 30)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/gripper_controller/controller_state",
            self._on_controller_state,
            10,
        )
        self._startup_detach_timer = None
        if self._grasp_mode == "attach":
            self._startup_detach_timer = self.create_timer(
                _STARTUP_DETACH_PERIOD_SEC,
                self._on_startup_detach_timer,
            )

    def _on_left_contacts(self, message):
        self._left_targets = _contacts_message_targets(message, self._ignored_name_fragments)
        self._left_touch = bool(self._left_targets)
        self._maybe_update_grasp_state()

    def _on_right_contacts(self, message):
        self._right_targets = _contacts_message_targets(message, self._ignored_name_fragments)
        self._right_touch = bool(self._right_targets)
        self._maybe_update_grasp_state()

    def _on_grasp_state(self, message):
        grasp_state = getattr(message, "data", "")
        self._attached, self._attached_target = apply_grasp_state_feedback(
            attached=self._attached,
            attached_target=self._attached_target,
            grasp_state=grasp_state,
        )
        if str(grasp_state).strip().lower() == "detached":
            self._startup_detach_pending = False
        self._maybe_update_grasp_state()

    def _on_startup_detach_timer(self):
        if not self._startup_detach_pending:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self._startup_detach_deadline_sec is None:
            self._startup_detach_deadline_sec = now_sec + _STARTUP_DETACH_TIMEOUT_SEC

        if now_sec >= self._startup_detach_deadline_sec:
            self._startup_detach_pending = False
            return

        self._publish_detach_command()

    def _on_joint_state(self, message):
        joint_positions = _lookup_joint_positions(
            message.name,
            message.position,
            self._joint6_name,
            self._joint7_name,
        )
        if joint_positions is None:
            return

        self._joint6, self._joint7 = joint_positions
        self._maybe_update_grasp_state()

    def _on_controller_state(self, message):
        reference_positions = _lookup_joint_positions(
            message.joint_names,
            message.reference.positions,
            self._joint6_name,
            self._joint7_name,
        )
        if reference_positions is None:
            return

        self._reference6, self._reference7 = reference_positions
        error_positions = _lookup_joint_positions(
            message.joint_names,
            message.error.positions,
            self._joint6_name,
            self._joint7_name,
        )
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if (
            self._joint6 is not None
            and self._joint7 is not None
            and should_log_asymmetry_snapshot(
                feedback6=self._joint6,
                feedback7=self._joint7,
                threshold=self._diagnostic_asymmetry_threshold,
                now_sec=now_sec,
                last_logged_sec=self._last_asymmetry_log_sec,
                cooldown_sec=self._diagnostic_cooldown_sec,
            )
        ):
            if error_positions is None:
                error6, error7 = 0.0, 0.0
            else:
                error6, error7 = error_positions
            self.get_logger().warn(
                format_asymmetry_snapshot(
                    now_sec=now_sec,
                    holding=self._holding,
                    left_touch=self._left_touch,
                    right_touch=self._right_touch,
                    hold_target=self._hold_target,
                    last_hold_command_sec=self._last_hold_command_sec,
                    joint6=self._joint6,
                    joint7=self._joint7,
                    reference6=self._reference6,
                    reference7=self._reference7,
                    error6=error6,
                    error7=error7,
                    last_action=self._last_action,
                )
            )
            self._last_asymmetry_log_sec = now_sec

        if self._holding and self._hold_target is not None:
            action = evaluate_symmetric_hold_action(
                reference6=self._reference6,
                reference7=self._reference7,
                hold_target=self._hold_target,
                release_margin=self._hold_release_margin,
                command_tolerance=self._hold_command_tolerance,
            )
            if action == "hold":
                self._publish_hold_command()
            elif action == "release":
                self._holding = False
                self._hold_target = None
                self._hold_candidate_since_sec = None
                self._last_action = "release_hold"
                self.get_logger().info("Released symmetric hold")
            return

        self._maybe_update_grasp_state()

    def _maybe_update_grasp_state(self):
        if self._grasp_mode == "attach":
            self._maybe_update_attach_state()
            return

        self._maybe_enter_hold()

    def _maybe_update_attach_state(self):
        if self._joint6 is None or self._joint7 is None or self._reference6 is None or self._reference7 is None:
            return

        attach_target = select_attach_target(
            left_targets=self._left_targets,
            right_targets=self._right_targets,
            allowed_targets=self._attach_allowed_targets,
        )
        closing_command = _is_closing_command(
            reference6=self._reference6,
            reference7=self._reference7,
            joint6=self._joint6,
            joint7=self._joint7,
            tolerance=self._hold_command_tolerance,
        )
        opening_command = is_opening_command(
            reference6=self._reference6,
            reference7=self._reference7,
            detach_open_threshold=self._detach_open_threshold,
        )
        now_sec = self.get_clock().now().nanoseconds / 1e9
        action, attach_candidate_since_sec, detach_candidate_since_sec = evaluate_attach_mode_action(
            attached=self._attached,
            attach_target=attach_target,
            closing_command=closing_command,
            opening_command=opening_command,
            now_sec=now_sec,
            attach_candidate_since_sec=self._attach_candidate_since_sec,
            detach_candidate_since_sec=self._detach_candidate_since_sec,
            attach_contact_debounce_sec=self._attach_contact_debounce_sec,
            detach_open_debounce_sec=self._detach_open_debounce_sec,
        )
        self._attach_candidate_since_sec = attach_candidate_since_sec
        self._detach_candidate_since_sec = detach_candidate_since_sec

        if action == "attach" and attach_target is not None:
            self._publish_attach_command(attach_target)
        elif action == "detach":
            self._publish_detach_command()

    def _maybe_enter_hold(self):
        if (
            self._holding
            or self._joint6 is None
            or self._joint7 is None
            or self._reference6 is None
            or self._reference7 is None
        ):
            return

        closing_command = _is_closing_command(
            reference6=self._reference6,
            reference7=self._reference7,
            joint6=self._joint6,
            joint7=self._joint7,
            tolerance=self._hold_command_tolerance,
        )
        effective_left_touch = self._left_touch and closing_command
        effective_right_touch = self._right_touch and closing_command
        touch_ready = is_hold_touch_ready(
            left_touch=effective_left_touch,
            right_touch=effective_right_touch,
            joint6=self._joint6,
            joint7=self._joint7,
            single_touch_max_total_width=self._single_touch_max_total_width,
        )
        now_sec = self.get_clock().now().nanoseconds / 1e9
        action, candidate_since_sec = evaluate_hold_transition(
            holding=self._holding,
            touch_ready=touch_ready,
            now_sec=now_sec,
            candidate_since_sec=self._hold_candidate_since_sec,
            debounce_sec=self._contact_debounce_sec,
        )
        self._hold_candidate_since_sec = candidate_since_sec

        if action != "hold":
            return

        self._holding = True
        self._hold_target = compute_symmetric_hold_target(
            joint6=self._joint6,
            joint7=self._joint7,
            preload=self._hold_preload,
            max_joint_position=self._joint_max_position,
        )
        self._hold_candidate_since_sec = None
        self._last_action = "enter_hold"
        self._publish_hold_command()
        self.get_logger().info(
            f"Entered symmetric hold with target {self._hold_target:.6f}"
        )

    def _publish_attach_command(self, target_name):
        if Empty is not None:
            self._attach_pub.publish(Empty())
        self._attached_target = target_name
        self._attach_candidate_since_sec = None
        self._detach_candidate_since_sec = None
        self._last_action = f"attach:{target_name}"

    def _publish_detach_command(self):
        if Empty is not None:
            self._detach_pub.publish(Empty())
        self._attach_candidate_since_sec = None
        self._detach_candidate_since_sec = None
        self._last_action = "detach"

    def _publish_hold_command(self):
        if self._hold_target is None:
            return

        trajectory = JointTrajectory()
        trajectory.joint_names = [self._joint6_name, self._joint7_name]
        point = JointTrajectoryPoint()
        point.positions = [self._hold_target, self._hold_target]
        point.time_from_start = Duration(
            sec=int(self._hold_command_duration_sec),
            nanosec=int((self._hold_command_duration_sec % 1.0) * 1_000_000_000),
        )
        trajectory.points = [point]
        self._hold_pub.publish(trajectory)
        self._last_hold_command_sec = self.get_clock().now().nanoseconds / 1e9
        self._last_action = "hold"


def main():
    if rclpy is None:
        raise RuntimeError("rclpy is required to run grasp_manager.py")

    rclpy.init()
    node = GraspManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
