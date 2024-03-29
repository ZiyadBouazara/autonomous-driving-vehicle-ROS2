import math
import sys
import time
from typing import Union

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from design3_msgs.msg import Objective
from planning.utils.robot_state import RobotState

# ROS Imports

# PID CONTROL PARAMS
KP: float = 30.0
KD: float = 6.0
KI: float = 0.0

# WALL FOLLOW PARAMS
THETA_DEG: int = 50
DESIRED_DISTANCE_FROM_WALL: float = 0.15
LOOKAHEAD: float = 0.1
VELOCITY = 0.5


class WallFollowingNode(Node):
    def __init__(self) -> None:
        super().__init__("wall_following_node", parameter_overrides=[])

        self.last_time: float = time.time()
        self.integral: float = 0.0
        self.prev_error: float = 0.0

        self.lidar_sub = self.create_subscription(LaserScan, "scan", self.lidar_callback, 10)
        self.objective_sub = self.create_subscription(Objective, "objective", self.objective_callback, 10)

        self.drive_pub = self.create_publisher(Twist, "cmd_drive", 10)

        self.current_objective: Union[Objective, None] = None

        self.get_logger().info("Wall following node started")

    def objective_callback(self, msg: Objective) -> None:
        self.current_objective = msg
        self.get_logger().info(f"Received new objective: {msg.robot_state_description}")

    def lidar_callback(self, data: LaserScan) -> None:
        if self.current_objective is None:
            return

        # We only follow the wall when we are going to the depot or the drop point
        if (
            self.current_objective.robot_state != RobotState.GOING_TO_DEPOT.value
            and self.current_objective.robot_state != RobotState.GOING_TO_DROP_POINT.value
        ):
            return

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

        self.get_logger().info(f"Steering angle: {math.degrees(steering_angle)}")

        twist_msg = Twist()
        twist_msg.angular.z = steering_angle / delta_time
        twist_msg.linear.x = VELOCITY
        self.drive_pub.publish(twist_msg)

    def get_range_at_angle(self, data: LaserScan, angle: float) -> float:
        if angle < data.angle_min or angle > data.angle_max:
            raise ValueError("Angle out of range")

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
    wf = WallFollowingNode()
    rclpy.spin(wf)
    wf.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
