import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    shared_dir = get_package_share_directory("design3_system")

    # TF
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(shared_dir, "launch"), "/description.launch.py"])
    )

    # Lidar
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(shared_dir, "launch"), "/lidar.launch.py"])
    )

    # Camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(shared_dir, "launch"), "/camera.launch.py"])
    )

    # Aruco
    aruco_params = os.path.join(get_package_share_directory("design3_system"), "config", "aruco_detection.yaml")

    aruco_car_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        name="aruco_car_node",
        parameters=[aruco_params],
        remappings=[("aruco_poses", "aruco/car_poses"), ("aruco_markers", "aruco/car_markers")],
    )

    aruco_city_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        name="aruco_city_node",
        parameters=[aruco_params],
        remappings=[("aruco_poses", "aruco/city_poses"), ("aruco_markers", "aruco/city_markers")],
    )

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

    # Teleop
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(shared_dir, "launch"), "/teleop.launch.py"])
    )

    return LaunchDescription(
        [
            description_launch,
            lidar_launch,
            camera_launch,
            aruco_car_node,
            aruco_city_node,
            twist_mux_la,
            twist_mux_node,
            teleop_launch,
        ]
    )
