import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Empty
from tf_transformations import euler_from_quaternion

from design3_msgs.msg import Edge, Localization

VELOCITY = 0.2
ANGULAR_VELOCITY = 4.5

DISTANCE_TO_TRAVEL_FORWARD = 0.68
DISTANCE_TO_TRAVEL_FORWARD_FOR_LEFT_TURN = 0.46
DISTANCE_TO_TRAVEL_FORWARD_FOR_RIGHT_TURN = 0.32


class IntersectionTurningNode(Node):
    def __init__(self):
        super().__init__("intersection_turning_node", parameter_overrides=[])

        self.create_subscription(Edge, "current_edge", self.edge_callback, 10)
        self.create_subscription(Twist, "odom", self.odom_callback, 10)
        self.create_subscription(Localization, "localization", self.localization_callback, 10)

        self.drive_pub = self.create_publisher(Twist, "cmd_intersection", 10)
        self.reset_odom_pub = self.create_publisher(Empty, "reset_odom", 10)

        self.reset_intersection()

        self.get_logger().info("Intersection turning node has started")

    def reset_intersection(self):
        self.current_edge = None
        self.start_intersection_time = None
        self.current_aruco_orientation = 0.0
        self.is_adjustment_finished = False

    def localization_callback(self, msg: Localization):
        orientation = msg.pose.orientation
        (_, pitch, _) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        self.current_aruco_orientation = pitch

    def is_intersection(self, loc_id: str) -> bool:
        return loc_id.startswith("A")

    def edge_callback(self, msg: Edge) -> None:
        if self.is_intersection(msg.origin):
            self.current_edge = msg
            self.reset_odom_pub.publish(Empty())
            self.get_logger().info("Starting intersection sequence")
            self.start_intersection_time = time.time()

    def odom_callback(self, odom: Twist):
        if (
            self.current_edge is None
            or self.start_intersection_time is None
            or not self.is_intersection(self.current_edge.origin)
        ):
            return

        # Adjust angle to face Aruco marker
        #if not self.is_adjustment_finished and abs(self.current_aruco_orientation) > math.radians(20):
        #    self.get_logger().info("Adjusting")
        #    self.is_adjustment_finished = self.adjust_orientation(odom)
        #    return

        # Stop for 1 second
        current_time = time.time()
        if current_time - self.start_intersection_time < 1:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.drive_pub.publish(twist)
            return

        direction = self.current_edge.direction

        is_sequence_finished: bool = False
        if direction == "l":
            is_sequence_finished = self.move_forward_sequence(odom)
        elif direction == "g":
            is_sequence_finished = self.turn_sequence(odom, direction)
        elif direction == "d":
            is_sequence_finished = self.turn_sequence(odom, direction)
        else:
            self.get_logger().error(f"Received invalid direction '{direction}'")

        if is_sequence_finished:
            self.reset_intersection()
            self.get_logger().info("Intersection sequence is done")

    def adjust_orientation(self, odom) -> bool:
        target_angle = self.current_aruco_orientation

        if abs(odom.angular.z) < abs(target_angle):
            twist_msg = Twist()
            twist_msg.angular.z = ANGULAR_VELOCITY if target_angle > 0.0 else -ANGULAR_VELOCITY
            self.drive_pub.publish(twist_msg)

            return False

        self.reset_odom_pub.publish(Empty())
        return True

    def move_forward_sequence(self, odom: Twist) -> bool:
        if odom.linear.x < DISTANCE_TO_TRAVEL_FORWARD:
            twist_msg = Twist()
            twist_msg.linear.x = VELOCITY
            self.drive_pub.publish(twist_msg)

            return False

        return True

    def turn_sequence(self, odom: Twist, direction: str) -> bool:
        is_left_turn: bool = direction == "g"
        target_angle = math.pi / 2.0 if is_left_turn else -math.pi / 2.0
        distance_to_travel = (
            DISTANCE_TO_TRAVEL_FORWARD_FOR_LEFT_TURN if is_left_turn else DISTANCE_TO_TRAVEL_FORWARD_FOR_RIGHT_TURN
        )

        if odom.linear.x < distance_to_travel:
            twist_msg = Twist()
            twist_msg.linear.x = VELOCITY
            self.drive_pub.publish(twist_msg)
            return False
        elif abs(odom.angular.z) < abs(target_angle):
            twist_msg = Twist()
            twist_msg.angular.z = ANGULAR_VELOCITY if is_left_turn else -ANGULAR_VELOCITY
            self.drive_pub.publish(twist_msg)
            return False
        elif odom.linear.x < distance_to_travel + 0.15:
            twist_msg = Twist()
            twist_msg.linear.x = VELOCITY
            self.drive_pub.publish(twist_msg)
            return False

        return True


def main(args=None):
    rclpy.init(args=args)

    intersection_turning_node = IntersectionTurningNode()

    rclpy.spin(intersection_turning_node)

    intersection_turning_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
