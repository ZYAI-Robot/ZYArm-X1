#!/usr/bin/env python3

import argparse
import time


def _missing_controller_names(required_controllers, available_controllers):
    available = set(available_controllers)
    return [controller_name for controller_name in required_controllers if controller_name not in available]


def _missing_controller_types(required_types, available_types):
    available = set(available_types)
    return [controller_type for controller_type in required_types if controller_type not in available]


def wait_for_required_controller_names(
    *,
    fetch_available_controllers,
    required_controllers,
    timeout_seconds,
    sleep_interval_seconds,
    monotonic=time.monotonic,
    sleep=time.sleep,
):
    deadline = monotonic() + timeout_seconds

    while monotonic() < deadline:
        missing = _missing_controller_names(required_controllers, fetch_available_controllers())
        if not missing:
            return
        sleep(sleep_interval_seconds)

    missing = _missing_controller_names(required_controllers, fetch_available_controllers())
    raise TimeoutError(
        "Timed out waiting for controllers: " + ", ".join(missing)
    )


def wait_for_required_controller_types(
    *,
    fetch_available_types,
    required_types,
    timeout_seconds,
    sleep_interval_seconds,
    monotonic=time.monotonic,
    sleep=time.sleep,
):
    deadline = monotonic() + timeout_seconds

    while monotonic() < deadline:
        missing = _missing_controller_types(required_types, fetch_available_types())
        if not missing:
            return
        sleep(sleep_interval_seconds)

    missing = _missing_controller_types(required_types, fetch_available_types())
    raise TimeoutError(
        "Timed out waiting for controller types: " + ", ".join(missing)
    )


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Wait until controller_manager reports the required controller names or types."
    )
    parser.add_argument(
        "--controller-manager",
        required=True,
        help="Controller manager node name, for example /zyarm_x1_standard_controller_manager.",
    )
    parser.add_argument(
        "--required-controller",
        action="append",
        dest="required_controllers",
        help="A controller name that must appear in list_controllers.",
    )
    parser.add_argument(
        "--required-type",
        action="append",
        dest="required_types",
        help="A controller type that must appear in list_controller_types.",
    )
    parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=30.0,
        help="How long to wait before exiting with an error.",
    )
    parser.add_argument(
        "--sleep-interval-seconds",
        type=float,
        default=0.5,
        help="Polling interval between controller type checks.",
    )
    args, _unknown = parser.parse_known_args(args=argv)
    if not args.required_controllers and not args.required_types:
        parser.error("at least one --required-controller or --required-type must be provided")
    return args


def _build_fetch_available_types(node, client):
    from controller_manager_msgs.srv import ListControllerTypes
    import rclpy

    def fetch_available_types():
        if not client.service_is_ready():
            return []

        future = client.call_async(ListControllerTypes.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            return []
        return list(future.result().types)

    return fetch_available_types


def _build_fetch_available_controllers(node, client):
    from controller_manager_msgs.srv import ListControllers
    import rclpy

    def fetch_available_controllers():
        if not client.service_is_ready():
            return []

        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            return []
        return [controller.name for controller in future.result().controller]

    return fetch_available_controllers


def main():
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from controller_manager_msgs.srv import ListControllerTypes

    args = _parse_args()
    rclpy.init()
    node = rclpy.create_node("controller_type_waiter")

    try:
        if args.required_types:
            service_name = f"{args.controller_manager.rstrip('/')}/list_controller_types"
            client = node.create_client(ListControllerTypes, service_name)
            fetch_available_types = _build_fetch_available_types(node, client)
            wait_for_required_controller_types(
                fetch_available_types=fetch_available_types,
                required_types=args.required_types,
                timeout_seconds=args.timeout_seconds,
                sleep_interval_seconds=args.sleep_interval_seconds,
            )
            node.get_logger().info(
                f"Controller types are ready on {service_name}: "
                f"{', '.join(args.required_types)}"
            )
        else:
            service_name = f"{args.controller_manager.rstrip('/')}/list_controllers"
            client = node.create_client(ListControllers, service_name)
            fetch_available_controllers = _build_fetch_available_controllers(node, client)
            wait_for_required_controller_names(
                fetch_available_controllers=fetch_available_controllers,
                required_controllers=args.required_controllers,
                timeout_seconds=args.timeout_seconds,
                sleep_interval_seconds=args.sleep_interval_seconds,
            )
            node.get_logger().info(
                f"Controllers are ready on {service_name}: "
                f"{', '.join(args.required_controllers)}"
            )
        return 0
    except TimeoutError as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
