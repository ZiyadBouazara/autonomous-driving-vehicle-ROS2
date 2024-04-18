import math

import pytest
import rclpy
from geometry_msgs.msg import Twist

from control.motor_driver_node import MotorDriverNode, clamp


@pytest.fixture(autouse=True)
def init_motor_driver():
    rclpy.init(args=[])
    yield MotorDriverNode()
    rclpy.shutdown()


def test_given_val_left_and_val_right_when_create_json_command_then_return_command(
    init_motor_driver: MotorDriverNode,
):
    val_left = 10
    val_right = 10

    command = init_motor_driver.create_json_command(val_left, val_right)

    expected_command = '{"T":1,"L":' + str(val_left) + ',"R":' + str(val_right) + "}"
    assert command == expected_command


def test_given_twist_message_when_get_left_and_right_speed_then_return_speeds(
    init_motor_driver: MotorDriverNode,
):
    msg = Twist()
    msg.linear.x = 1.0
    msg.angular.z = 0.0

    left, right = init_motor_driver.get_left_and_right_speed(msg)

    assert left == right == 182


def test_given_twist_message_with_infinite_x_when_get_left_and_right_speed_then_return_0(
    init_motor_driver: MotorDriverNode,
):
    msg = Twist()
    msg.linear.x = math.inf
    msg.angular.z = 0.0

    left, right = init_motor_driver.get_left_and_right_speed(msg)

    assert left == right == 0


def test_given_twist_message_with_infinite_angular_when_get_left_and_right_speed_then_return_0(
    init_motor_driver: MotorDriverNode,
):
    msg = Twist()
    msg.linear.x = 1.0
    msg.angular.z = math.inf

    left, right = init_motor_driver.get_left_and_right_speed(msg)

    assert left == right == 0


def test_given_twist_message_with_nan_x_when_get_left_and_right_speed_then_return_0(
    init_motor_driver: MotorDriverNode,
):
    msg = Twist()
    msg.linear.x = math.nan
    msg.angular.z = 0.0

    left, right = init_motor_driver.get_left_and_right_speed(msg)

    assert left == right == 0


def test_given_twist_message_with_nan_angular_when_get_left_and_right_speed_then_return_0(
    init_motor_driver: MotorDriverNode,
):
    msg = Twist()
    msg.linear.x = 1.0
    msg.angular.z = math.nan

    left, right = init_motor_driver.get_left_and_right_speed(msg)

    assert left == right == 0


def test_given_in_between_value_when_clamp_then_return_low_value(
    init_motor_driver: MotorDriverNode,
):
    value, min_value, max_value = 100, 0, 255

    clamped_value = clamp(value, min_value, max_value)

    assert clamped_value == value


def test_given_high_value_when_clamp_then_return_max_value(
    init_motor_driver: MotorDriverNode,
):
    value, min_value, max_value = 1000, 0, 255

    clamped_value = clamp(value, min_value, max_value)

    assert clamped_value == max_value


def test_given_low_value_when_clamp_then_return_low_value(
    init_motor_driver: MotorDriverNode,
):
    value, min_value, max_value = -1000, 0, 255

    clamped_value = clamp(value, min_value, max_value)

    assert clamped_value == min_value
