from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    optical_sensor_node = Node(
        package="perception",
        executable="optical_sensor_node",
    )

    localization_node = Node(
        package="perception",
        executable="localization_node",
        name="localization_node",
    )

    return LaunchDescription([optical_sensor_node, localization_node])
