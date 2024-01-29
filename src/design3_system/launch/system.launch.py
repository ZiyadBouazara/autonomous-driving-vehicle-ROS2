from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os

def generate_launch_description():    
    shared_dir = get_package_share_directory('design3_system')

    lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(shared_dir, 'launch'),
                '/lidar.launch.py'
            ])
    )

    camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(shared_dir, 'launch'),
                '/camera.launch.py'
            ])
    )

    motor_driver_node = Node(
        package='motor_driver',
        executable='motor_driver_node'
    )

    return LaunchDescription([
        lidar_launch,
        camera_launch,
        motor_driver_node
    ])
