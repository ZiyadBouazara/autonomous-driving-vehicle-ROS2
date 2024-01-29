from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():    
    lidar_shared_dir = get_package_share_directory('design3_system')
    laser_config = os.path.join(lidar_shared_dir, 'config', 'lidar.yaml')

    laser_la = DeclareLaunchArgument(
        'laser_config',
        default_value=laser_config
    )

    laser_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[LaunchConfiguration('laser_config')],
    )
    
    return LaunchDescription([
        laser_la,
        laser_node
    ])
