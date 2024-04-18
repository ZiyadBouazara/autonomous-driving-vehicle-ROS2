from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motor_driver_node = Node(package="control", executable="motor_driver_node")

    null_command_sending_node = Node(
        package="control", executable="null_command_sending_node"
    )

    wall_following_node = Node(
        package="control",
        executable="wall_following_node",
    )

    intersection_turning_node = Node(package="control", executable="intersection_turning_node")

    return LaunchDescription(
        [motor_driver_node, null_command_sending_node, wall_following_node, intersection_turning_node]
    )
