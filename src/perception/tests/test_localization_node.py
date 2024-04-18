import pytest
import rclpy
from geometry_msgs.msg import Pose
from mock import MagicMock
from perception.localization_node import LocalizationNode


@pytest.fixture(autouse=True)
def init_localization_node():
    rclpy.init(args=[])
    yield LocalizationNode()
    rclpy.shutdown()


def test_given_no_aruco_mapping_then_aruco_marker_callback_should_skip_localization(
    init_localization_node: LocalizationNode,
):
    init_localization_node.get_logger = MagicMock()  # Mocking get_logger method
    init_localization_node.aruco_mapping = None

    init_localization_node.aruco_marker_callback(None)

    init_localization_node.get_logger.warn.assert_called_once_with(
        "No aruco mapping available. Skipping localization."
    )


def test_given_empty_aruco_markers_then_aruco_marker_callback_should_not_publish(
    init_localization_node: LocalizationNode,
):
    init_localization_node.publish_localization = (
        MagicMock()
    )  # Mocking publish_localization method

    init_localization_node.aruco_marker_callback(None)

    init_localization_node.publish_localization.assert_not_called()


def test_given_valid_aruco_markers_then_aruco_marker_callback_should_publish_localization(
    init_localization_node: LocalizationNode,
):
    init_localization_node.publish_localization = (
        MagicMock()
    )  # Mocking publish_localization method
    init_localization_node.aruco_mapping = {1: "city_loc_1"}  # Mocking aruco_mapping

    # Creating a valid ArucoMarkers message
    aruco_markers_msg = MagicMock()
    aruco_markers_msg.poses = [Pose()]
    aruco_markers_msg.marker_ids = [1]

    init_localization_node.aruco_marker_callback(aruco_markers_msg)

    init_localization_node.publish_localization.assert_called_once()


def test_given_invalid_aruco_markers_then_aruco_marker_callback_should_not_publish_localization(
    init_localization_node: LocalizationNode,
):
    init_localization_node.publish_localization = (
        MagicMock()
    )  # Mocking publish_localization method
    init_localization_node.aruco_mapping = {1: "city_loc_1"}  # Mocking aruco_mapping

    # Creating an invalid ArucoMarkers message
    aruco_markers_msg = MagicMock()
    aruco_markers_msg.poses = [Pose()]
    aruco_markers_msg.marker_ids = [2]  # Invalid marker ID

    init_localization_node.aruco_marker_callback(aruco_markers_msg)

    init_localization_node.publish_localization.assert_not_called()


def test_given_invalid_transform_then_transform_pose_should_return_none(
    init_localization_node: LocalizationNode,
):
    # Mocking tf_buffer.lookup_transform to raise an exception
    init_localization_node.tf_buffer.lookup_transform = MagicMock(
        side_effect=Exception("Test exception")
    )

    result = init_localization_node.transform_pose(Pose())

    assert result is None
