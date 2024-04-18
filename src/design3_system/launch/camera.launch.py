import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_path = os.path.join(get_package_share_directory("design3_system"), "config", "camera.yaml")
    launch_path = os.path.join(get_package_share_directory("depthai_ros_driver"), "launch", "camera.launch.py")

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={"camera_model": "OAK-D-LITE", "params_file": config_path}.items(),
    )

    return LaunchDescription([camera_launch])
