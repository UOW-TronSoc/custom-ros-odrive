"""Example: launch a single custom_odrive_node.

Loads shared defaults from config/custom_odrive_defaults.yaml, then sets the
required per-motor fields (namespace, node_id, interface). Add optional
parameter overrides in the dict only when they differ from the defaults.

Required per Node
-----------------
  namespace   — ROS namespace for topics/services (e.g. wheel_bl → /wheel_bl/...)
  node_id     — ODrive CAN Simple node_id (must match firmware axis0.config.can.node_id)
  interface   — SocketCAN iface already UP on the host (can0, can_core, …)

Copy this file into a rover package and edit for production hardware.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    defaults = os.path.join(
        get_package_share_directory("custom_odrive"),
        "config",
        "custom_odrive_defaults.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="custom_odrive",
                executable="custom_odrive_node",
                name="can_node",
                namespace="wheel_bl",
                parameters=[
                    defaults,
                    {
                        # Required (no defaults in yaml)
                        "node_id": 3,
                        "interface": "can0",
                        # Optional overrides of custom_odrive_defaults.yaml go here
                        # e.g. "invert_direction": True,
                    },
                ],
                output="screen",
            ),
        ]
    )
