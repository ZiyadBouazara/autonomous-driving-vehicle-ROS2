import pytest
import rclpy
from geometry_msgs.msg import Pose

from perception.car_detection_node import CarDetectionNode


@pytest.fixture(autouse=True)
def init_car_detection():
    rclpy.init(args=[])
    yield CarDetectionNode()
    rclpy.shutdown()


def test_given_long_distance_then_aruco_close_should_return_false(init_car_detection: CarDetectionNode):
    pose = Pose()
    pose.position.z = 100.0

    result = init_car_detection.is_aruco_too_close(pose)

    assert result is False


def test_given_short_distance_then_aruco_close_should_return_true(init_car_detection: CarDetectionNode):
    pose = Pose()
    pose.position.z = 0.01

    result = init_car_detection.is_aruco_too_close(pose)

    assert result is True


def test_given_no_poses_then_close_car_detected_should_return_false(init_car_detection: CarDetectionNode):
    poses = []

    result = init_car_detection.is_close_car_detected(poses)

    assert result is False


def test_given_far_poses_then_close_car_detected_should_return_false(init_car_detection: CarDetectionNode):
    pose = Pose()
    pose.position.z = 100.0
    poses = [pose]

    result = init_car_detection.is_close_car_detected(poses)

    assert result is False


def test_given_close_poses_then_close_car_detected_should_return_true(init_car_detection: CarDetectionNode):
    pose = Pose()
    pose.position.z = 0.01
    poses = [pose]

    result = init_car_detection.is_close_car_detected(poses)

    assert result is True
