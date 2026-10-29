"""Minimal teleoperation bring-up: simulator, wheel controller and UDP bridge."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("qkrt_drive_ros"), "config", "rover.yaml")
    return LaunchDescription([
        Node(package="qkrt_drive_ros", executable=n, name=n, parameters=[params], output="screen")
        for n in ("sim_node", "drive_controller_node", "localization_node", "teleop_bridge_node")
    ])
