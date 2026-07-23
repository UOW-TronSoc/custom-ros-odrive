"""Example: launch multiple custom_odrive_node instances on one CAN bus.

Same pattern as example_launch.py (defaults YAML + required/override dict), with
one Node block per motor. Copy a block to add another axis; only put required
fields and overrides in each dict.
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
                namespace="wheel_fl",
                parameters=[
                    defaults,
                    {
                        "node_id": 1,
                        "interface": "can0",
                        "invert_direction": True,
                    },
                ],
                output="screen",
            ),
            Node(
                package="custom_odrive",
                executable="custom_odrive_node",
                name="can_node",
                namespace="wheel_bl",
                parameters=[
                    defaults,
                    {
                        "node_id": 2,
                        "interface": "can0",
                        "invert_direction": True,
                    },
                ],
                output="screen",
            ),
            Node(
                package="custom_odrive",
                executable="custom_odrive_node",
                name="can_node",
                namespace="wheel_br",
                parameters=[
                    defaults,
                    {
                        "node_id": 3,
                        "interface": "can0",
                    },
                ],
                output="screen",
            ),
            Node(
                package="custom_odrive",
                executable="custom_odrive_node",
                name="can_node",
                namespace="wheel_fr",
                parameters=[
                    defaults,
                    {
                        "node_id": 4,
                        "interface": "can0",
                    },
                ],
                output="screen",
            ),
        ]
    )
