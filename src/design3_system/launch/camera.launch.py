from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          os.path.join(
          get_package_share_directory("depthai_ros_driver"), 
          'launch'),
          '/camera.launch.py'
        ])
    )

    return LaunchDescription([
        camera_launch, 
    ])
