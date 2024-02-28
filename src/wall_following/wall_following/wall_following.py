# Python import
from __future__ import print_function

import math
import sys
import time

# ROS Imports
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

# PID CONTROL PARAMS
KP: float = 8.0
KD: float = 0
KI: float = 0

# WALL FOLLOW PARAMS
THETA_DEG: int = 50
DESIRED_DISTANCE_FROM_WALL: float = 0.15
LOOKAHEAD: float = 1.0


class WallFollow(Node):
    def __init__(self) -> None:
        super().__init__("wall_follow", parameter_overrides=[])

        self.last_time: float = time.time()
        self.integral: float = 0.0
        self.prev_error: float = 0.0

        lidarscan_topic: str = "/scan"
        drive_topic: str = "/drive_vel"

        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(Twist, drive_topic, 10)
        self.viz_error_pub = self.create_publisher(Float64, "/viz/wall_follow/error", 10)
        self.viz_steering_pub = self.create_publisher(Float64, "/viz/wall_follow/steering", 10)

    def lidar_callback(self, data: LaserScan) -> None:
        current_time: float = time.time()
        delta_time_s = current_time - self.last_time

        self.follow_right_wall(data, DESIRED_DISTANCE_FROM_WALL, delta_time_s)

        self.last_time = time.time()

    def follow_right_wall(self, data: LaserScan, dist_from_wall: float, delta_time: float) -> None:
        theta = math.radians(THETA_DEG)
        a = self.get_range_at_angle(data, theta)
        b = self.get_range_at_angle(data, 0)

        if math.isnan(a) or math.isnan(b) or math.isinf(a) or math.isinf(b):
            return

        alpha = math.atan((a * math.cos(theta) - b) / (a * math.sin(theta)))
        Dt = b * math.cos(alpha)
        Dt_plus_1 = Dt + LOOKAHEAD * math.sin(alpha)
        error = dist_from_wall - Dt_plus_1

        steering_angle = self.pid_control(error, delta_time)

        self.viz_error_pub.publish(Float64(data=error))
        self.viz_steering_pub.publish(Float64(data=steering_angle))

        if abs(steering_angle) <= math.radians(20):
            velocity = 1.0
        else:
            velocity = 0.5

        twist_msg = Twist()
        twist_msg.angular.z = steering_angle / delta_time
        twist_msg.linear.x = velocity
        self.drive_pub.publish(twist_msg)

    def get_range_at_angle(self, data: LaserScan, angle: float) -> float:
        if angle < data.angle_min or angle > data.angle_max:
            return float("inf")
        else:
            index = int((angle - data.angle_min) / data.angle_increment)
            return data.ranges[index]

    def pid_control(self, error: float, delta_time: float) -> float:
        self.integral += error
        derivative = (error - self.prev_error) / delta_time

        output = KP * error + KI * self.integral + KD * derivative

        self.prev_error = error

        return output


def main(args=None):
    rclpy.init(args=args)
    wf = WallFollow()
    rclpy.spin(wf)
    wf.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
