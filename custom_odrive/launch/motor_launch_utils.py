import os
from typing import Any, Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def load_motors_config(config_filename: str) -> List[Dict[str, Any]]:
    package_share = get_package_share_directory("custom_odrive")
    config_path = os.path.join(package_share, "config", config_filename)

    with open(config_path, "r", encoding="utf-8") as config_file:
        data = yaml.safe_load(config_file) or {}

    motors = data.get("motors", [])
    if not isinstance(motors, list) or not motors:
        raise RuntimeError(f"{config_path}: 'motors' must be a non-empty list")

    return motors


def motor_nodes_from_config(config_filename: str) -> List[Node]:
    motors = load_motors_config(config_filename)
    nodes: List[Node] = []

    for index, motor in enumerate(motors):
        namespace = motor.get("namespace", f"odrive_axis{index}")
        if "node_id" not in motor:
            raise RuntimeError(f"Missing node_id for motor at index {index} ({namespace})")

        nodes.append(
            Node(
                package="custom_odrive",
                executable="custom_odrive_node",
                name="can_node",
                namespace=namespace,
                parameters=[
                    {
                        "node_id": int(motor["node_id"]),
                        "interface": motor.get("interface", "can0"),
                        "axis_idle_on_shutdown": bool(motor.get("axis_idle_on_shutdown", False)),
                        "control_message_in_radians": bool(motor.get("control_message_in_radians", False)),
                        "invert_direction": bool(motor.get("invert_direction", False)),
                    }
                ],
                output="screen",
            )
        )

    return nodes
