"""Launch the QKRT drive stack. Set use_sim:=false on the real rover (motor controllers on CAN)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("qkrt_drive_ros"), "config", "rover.yaml")
    use_sim = LaunchConfiguration("use_sim")

    def node(name, **kw):
        return Node(package="qkrt_drive_ros", executable=name, name=name, parameters=[params], output="screen", **kw)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true"),
        node("sim_node", condition=IfCondition(use_sim)),
        node("drive_controller_node"),
        node("localization_node"),
        node("planner_node"),
        node("teleop_bridge_node"),
    ])
