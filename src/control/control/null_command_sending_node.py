import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class NullCommandSendingNode(Node):
    def __init__(self):
        super().__init__("null_command_sending_node", parameter_overrides=[])

        self.create_subscription(LaserScan, "scan", self.lidar_callback, 10)

        self.cmd_null_pub = self.create_publisher(Twist, "cmd_null", 10)

        self.get_logger().info("Null command sending node started")

    def lidar_callback(self, _):
        null_twist = Twist()
        null_twist.angular.z = 0.0
        null_twist.linear.x = 0.0
        self.cmd_null_pub.publish(null_twist)


def main(args=None):
    rclpy.init(args=args)

    null_command_sending_node = NullCommandSendingNode()

    rclpy.spin(null_command_sending_node)

    null_command_sending_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
