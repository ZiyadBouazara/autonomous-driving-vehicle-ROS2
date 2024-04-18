from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ldlidar_node = Node(
        package="ldlidar_ros2",
        executable="ldlidar_ros2_node",
        name="ldlidar_ros2_node",
        output="screen",
        parameters=[
            {"product_name": "LDLiDAR_STL19P"},
            {"laser_scan_topic_name": "scan"},
            {"point_cloud_2d_topic_name": "pointcloud2d"},
            {"frame_id": "base_laser"},
            {"port_name": "/dev/sensors/lidar"},
            {"serial_baudrate": 230400},
            {"laser_scan_dir": True},
            {"enable_angle_crop_func": False},
            {"angle_crop_min": 135.0},  # unit is degrees
            {"angle_crop_max": 225.0},  # unit is degrees
            {"range_min": 0.03},  # unit is meter
            {"range_max": 12.0},  # unit is meter
        ],
    )

    return LaunchDescription([ldlidar_node])
