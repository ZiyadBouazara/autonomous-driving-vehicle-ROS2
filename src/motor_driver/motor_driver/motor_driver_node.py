import math

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node

MAX_SPEED = 1.4  # in m/s
WHEEL_BASE = 0.055  # in m


def clamp(value: int, min_value: int, max_value: int):
    return max(min_value, min(value, max_value))


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node", parameter_overrides=[])

        self.cmd_vel_topic = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, twist_msg):
        v_x = twist_msg.linear.x
        omega_z = twist_msg.angular.z

        if math.isnan(v_x) or math.isnan(omega_z):
            return

        v_left = v_x - omega_z * WHEEL_BASE / 2.0
        v_right = v_x + omega_z * WHEEL_BASE / 2.0

        val_left = clamp(int(255.0 * v_left / MAX_SPEED), -255, 255)
        val_right = clamp(int(255.0 * v_right / MAX_SPEED), -255, 255)

        json_command = '{"T":1,"L":' + str(val_left) + ',"R":' + str(val_right) + "}"

        ser = serial.Serial("/dev/esp32", baudrate=1000000)

        try:
            ser.write(json_command.encode())
        except:
            print(f"Failed to send motor command")


def main(args=None):
    rclpy.init(args=args)

    motor_driver_node = MotorDriverNode()

    rclpy.spin(motor_driver_node)

    motor_driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
