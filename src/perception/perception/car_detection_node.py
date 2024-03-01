import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers

STOP_DISTANCE = 0.15  # in meters


class CarDetectionNode(Node):
    def __init__(self):
        super().__init__("car_detection_node", parameter_overrides=[])

        self.subscription_poses = self.create_subscription(
            ArucoMarkers, "aruco/car_markers", self.aruco_marker_callback, 10
        )

        self.publisher_car_vel = self.create_publisher(Twist, "cmd_detected_car", 10)

    def aruco_marker_callback(self, msg: ArucoMarkers):
        if self.is_close_car_detected(msg.poses):
            twist_msg = Twist()
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.publisher_car_vel.publish(twist_msg)

    def is_close_car_detected(self, poses):
        for pose in poses:
            if self.is_aruco_too_close(pose):
                return True

        return False

    def is_aruco_too_close(self, pose: Pose):
        distance = pose.position.z

        return distance < STOP_DISTANCE


def main(args=None):
    rclpy.init(args=args)
    node = CarDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
