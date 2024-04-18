import pytest
import rclpy
from std_msgs.msg import Bool
from unittest.mock import MagicMock
from perception.optical_sensor_node import OpticalSensorNode


@pytest.fixture(autouse=True)
def init_optical_sensor_node():
    rclpy.init(args=[])
    yield OpticalSensorNode()
    rclpy.shutdown()


def test_given_optical_sensor_node_when_timer_callback_then_publish_optical_sensor_data(
    init_optical_sensor_node: OpticalSensorNode, mocker
):
    received_optical_sensor_value = None

    def optical_sensor_callback(msg: Bool):
        nonlocal received_optical_sensor_value
        received_optical_sensor_value = msg.data

    # Mocking GPIO input
    gpio_mock = MagicMock()
    gpio_mock.input.return_value = True  # Simulate sensor returning True
    mocker.patch("RPi.GPIO.input", gpio_mock.input)

    optical_sensor_subscriber = init_optical_sensor_node.create_subscription(
        Bool, "is_car_loaded", optical_sensor_callback, 10
    )

    rclpy.spin_once(init_optical_sensor_node, timeout_sec=1)

    assert received_optical_sensor_value is not None
