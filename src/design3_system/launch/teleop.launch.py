import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    shared_dir = get_package_share_directory("design3_system")
    teleop_config = os.path.join(shared_dir, "config", "teleop.yaml")

    teleop_la = DeclareLaunchArgument("teleop_config", default_value=teleop_config)

    joy_node = Node(
        package="joy",
        executable="joy_node",
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[LaunchConfiguration("teleop_config")],
    )

    return LaunchDescription([teleop_la, joy_node, teleop_node])
