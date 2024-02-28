import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("depthai_ros_driver"), "launch"),
                "/camera.launch.py",
            ]
        )
    )

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

    return LaunchDescription([camera_launch, aruco_car_node, aruco_city_node])
