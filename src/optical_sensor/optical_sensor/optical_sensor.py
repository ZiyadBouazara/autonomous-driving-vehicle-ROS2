import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import Bool

OPTICAL_SENSOR_PIN = 7
TIMER_PERIOD_S = 1  # in seconds


class optical_sensor_node(Node):
    def __init__(self):
        super().__init__("optical_sensor", parameter_overrides=[])
        self.publisher = self.create_publisher(Bool, "optical_sensor", 10)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(OPTICAL_SENSOR_PIN, GPIO.IN)

        self.timer = self.create_timer(TIMER_PERIOD_S, self.timer_callback)

    def timer_callback(self):
        value = not GPIO.input(OPTICAL_SENSOR_PIN)
        msg = Bool()
        msg.data = bool(value)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    optical_sensor = optical_sensor_node()
    rclpy.spin(optical_sensor)


if __name__ == "__main__":
    main()
