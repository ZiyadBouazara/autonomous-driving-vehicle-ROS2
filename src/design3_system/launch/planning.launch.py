from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    objective_manager_node = Node(
        package="planning",
        executable="objective_manager_node",
    )

    path_planning_node = Node(
        package="planning",
        executable="path_planning_node",
    )

    path_following_node = Node(
        package="planning",
        executable="path_following_node",
    )

    return LaunchDescription([objective_manager_node, path_planning_node, path_following_node])
