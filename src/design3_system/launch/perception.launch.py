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

    car_detection_node = Node(
        package="perception",
        executable="car_detection_node",
    )

    drop_points_receiver_node = Node(
        package="perception", executable="drop_point_infos_receiver_node", name="drop_point_infos_receiver_node"
    )

    return LaunchDescription([optical_sensor_node, localization_node, drop_points_receiver_node, car_detection_node])
