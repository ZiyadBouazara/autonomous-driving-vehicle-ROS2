import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from control.null_command_sending_node import NullCommandSendingNode


@pytest.fixture(autouse=True)
def init_null_command_node():
    rclpy.init(args=[])
    yield NullCommandSendingNode()
    rclpy.shutdown()


class NullCommandSub(Node):
    def __init__(self):
        super().__init__("null_command_sub", parameter_overrides=[])
        self.subscription = self.create_subscription(Twist, "/cmd_null", self.null_command_callback, 10)
        self.publisher = self.create_publisher(LaserScan, "scan", 10)
        self.publisher.publish(LaserScan())
        self.null_command_received = False

    def null_command_callback(self, msg: Twist):
        self.null_command_received = True


def test_given_null_command_sending_node_when_lidar_callback_then_publish_null_command(
    init_null_command_node: NullCommandSendingNode,
):
    null_command_sub = NullCommandSub()

    rclpy.spin_once(null_command_sub, timeout_sec=1)

    assert null_command_sub.null_command_received

    null_command_sub.destroy_node()
