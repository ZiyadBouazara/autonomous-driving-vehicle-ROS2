import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, Twist
from rclpy.node import Node
from std_msgs.msg import Bool

STOP_DISTANCE = 0.20  # m
HEIGHT = 0.199  # m


class CloseCarDetectionNode(Node):
    def __init__(self):
        super().__init__("close_car_detection_node", parameter_overrides=[])
        self.publisher_car_vel = self.create_publisher(Twist, "cmd_detected_car", 10)
        self.publisher_car_close = self.create_publisher(Bool, "is_car_close", 10)
        self.is_car_close = False
        self.subscription_car_poses = self.create_subscription(PoseArray, "car_poses", self.car_poses_callback, 10)
        self.get_logger().info("CloseCarDetectionNode initialized.")

    def car_poses_callback(self, msg: PoseArray):
        for pose in msg.poses:
            if self.is_close_car_detected(pose):
                self.on_close_car_detected()
        self.publisher_car_close.publish(Bool(data=self.is_car_close))
        self.is_car_close = False

    def is_close_car_detected(self, pose):
        if self.is_aruco_too_close(pose):
            return True
        return False

    def is_aruco_too_close(self, pose: Pose):
        distance_aruco = np.sqrt(pose.position.x**2 + pose.position.y**2 + pose.position.z**2)
        distance_sol = np.sqrt(distance_aruco**2 - HEIGHT**2)
        return distance_sol < STOP_DISTANCE

    def on_close_car_detected(self):
        twist_msg = Twist()
        twist_msg.angular.z = 0.0
        twist_msg.linear.x = 0.0
        self.publisher_car_vel.publish(twist_msg)
        self.is_car_close = True


def main(args=None):
    rclpy.init(args=args)
    node = CloseCarDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
