import os
import sys

from launch import LaunchDescription

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from motor_launch_utils import motor_nodes_from_config  # noqa: E402


def generate_launch_description():
    return LaunchDescription(motor_nodes_from_config("example_single.yaml"))
