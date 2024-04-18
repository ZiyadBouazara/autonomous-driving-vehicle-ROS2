import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import Bool

OPTICAL_SENSOR_PIN = 7
TIMER_PERIOD_S = 1  # in seconds


class OpticalSensorNode(Node):
    def __init__(self):
        super().__init__("optical_sensor_node", parameter_overrides=[])

        self.publisher = self.create_publisher(Bool, "is_car_loaded", 10)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(OPTICAL_SENSOR_PIN, GPIO.IN)

        GPIO.add_event_detect(OPTICAL_SENSOR_PIN, GPIO.FALLING, callback=self.edge_callback, bouncetime=10)

        self.get_logger().info("Optical sensor node has been started.")

    def edge_callback(self, channel):
        value = not GPIO.input(OPTICAL_SENSOR_PIN)
        msg = Bool()
        msg.data = bool(value)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    optical_sensor = OpticalSensorNode()
    rclpy.spin(optical_sensor)


if __name__ == "__main__":
    main()
