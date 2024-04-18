import pytest
import rclpy
from sensor_msgs.msg import LaserScan

from control.wall_following_node import WallFollowingNode


@pytest.fixture(autouse=True)
def init_wall_following():
    rclpy.init(args=[])
    yield WallFollowingNode()
    rclpy.shutdown()


def given_angle_too_small_when_get_range_at_angle_then_raise_value_error(
    init_wall_following: WallFollowingNode,
):
    laser_scan = LaserScan()
    laser_scan.angle_min = 0

    with pytest.raises(ValueError):  # type: ignore
        init_wall_following.get_range_at_angle(laser_scan, -1)


def given_angle_too_large_when_get_range_at_angle_then_raise_value_error(
    init_wall_following: WallFollowingNode,
):
    laser_scan = LaserScan()
    laser_scan.angle_max = 0

    with pytest.raises(ValueError):  # type: ignore
        init_wall_following.get_range_at_angle(laser_scan, 1)


def given_angle_in_range_when_get_range_at_angle_then_return_correct_value(
    init_wall_following: WallFollowingNode,
):
    laser_scan = LaserScan()
    laser_scan.angle_min = 0
    laser_scan.angle_max = 1
    laser_scan.angle_increment = 0.1
    laser_scan.ranges = [1, 2, 3, 4, 5]

    result = init_wall_following.get_range_at_angle(laser_scan, 0.2)

    assert result == 2
