import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    shared_dir = get_package_share_directory("design3_system")

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(shared_dir, "launch"), "/lidar.launch.py"])
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(shared_dir, "launch"), "/camera.launch.py"])
    )

    motor_driver_node = Node(package="motor_driver", executable="motor_driver_node")

    # Twist mux
    twist_mux_config = os.path.join(
        get_package_share_directory("design3_system"),
        "config",
        "twist_mux.yaml",
    )

    twist_mux_la = DeclareLaunchArgument("twist_mux_config", default_value=twist_mux_config)

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux_node",
        parameters=[LaunchConfiguration("twist_mux_config")],
        remappings=[("cmd_vel_out", "cmd_vel")],
    )

    return LaunchDescription([lidar_launch, camera_launch, motor_driver_node, twist_mux_la, twist_mux_node])
