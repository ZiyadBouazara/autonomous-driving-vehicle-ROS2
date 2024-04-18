from typing import Union

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, Duration, TransformListener

SOURCE_FRAME = "oak-d-base-frame"
TARGET_FRAME = "front_bumper"


class CarDetectionNode(Node):
    def __init__(self):
        super().__init__("car_detection_node", parameter_overrides=[])
        self.publisher_car_poses = self.create_publisher(PoseArray, "car_poses", 10)
        self.get_logger().info("CarDetectionNode initialized.")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription_poses = self.create_subscription(
            ArucoMarkers, "aruco/car_markers", self.aruco_marker_callback, 10
        )

    def aruco_marker_callback(self, msg: ArucoMarkers):
        transformed_poses = PoseArray()
        poses = []
        for pose in msg.poses:
            transformed_pose = self.transform_pose(pose)
            if transformed_pose is not None:
                poses.append(transformed_pose)

        transformed_poses.poses = poses
        self.publisher_car_poses.publish(transformed_poses)

    def transform_pose(self, pose: Pose) -> Union[Pose, None]:
        try:
            transform = self.tf_buffer.lookup_transform(
                TARGET_FRAME,
                SOURCE_FRAME,
                self.get_clock().now().to_msg(),
                timeout=Duration(seconds=1),
            )
            transformed_pose = do_transform_pose(pose, transform)
            return transformed_pose
        except Exception as e:
            self.get_logger().error(f"Failed to transform pose from {SOURCE_FRAME} to {TARGET_FRAME}: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = CarDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
