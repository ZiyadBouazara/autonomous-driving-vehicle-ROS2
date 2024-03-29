import math
from typing import Union

import numpy as np
import rclpy
import tf2_geometry_msgs
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf2_ros import Buffer, TransformListener

from design3_msgs.msg import Localization, MapInfo

SOURCE_FRAME = "oak-d-base-frame"
TARGET_FRAME = "front_bumper"

# Seuil de distance pour la localisation près des zones spéciales
STOPPING_DISTANCE_THRESHOLD = 0.61  # m
ARUCO_HEIGHT = 0.24  # m
TOLERANCE = 0.02  # m


def euler_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("localization_node", parameter_overrides=[])

        # Initialise le buffer et le listener TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.city_marker_sub = self.create_subscription(
            ArucoMarkers, "/aruco/city_markers", self.aruco_marker_callback, 10
        )
        self.map_info_sub = self.create_subscription(MapInfo, "/map_info", self.map_info_callback, 10)

        # Publishers
        self.localization_pub = self.create_publisher(Localization, "/localization", 10)

        self.aruco_mapping: Union[dict[int, str], None] = None

        self.get_logger().info("Localization node has been initialized.")

    def map_info_callback(self, msg: MapInfo):
        self.aruco_mapping = {}

        msg.aruco_mapping: list[KeyValue] = msg.aruco_mapping  # type: ignore
        for mapping in msg.aruco_mapping:
            key: int = int(mapping.key)  # aruco_id
            value: str = mapping.value  # city_loc_id

            self.aruco_mapping[key] = value

        self.get_logger().info("City aruco mapping has been received.")

    def aruco_marker_callback(self, msg: ArucoMarkers):
        if self.aruco_mapping is None:
            self.get_logger().warn("No aruco mapping available. Skipping localization.")
            return

        if msg.marker_ids is None or len(msg.marker_ids) == 0:
            return

        result = self.find_closest_marker(msg)
        if result is None:  # Si find_closest_marker retourne None, on arrête là.
            return

        closest_pose, distance_to_closest, closest_id = result  # Déballage des variables si result n'est pas None
        if abs(distance_to_closest - STOPPING_DISTANCE_THRESHOLD) > TOLERANCE:
            self.get_logger().info(f"Distance: {distance_to_closest * 100} cm")
            return

        self.publish_localization(closest_pose, closest_id)

    def find_closest_marker(self, msg: ArucoMarkers) -> Union["tuple[Pose, float, int]", None]:
        closest_index = None
        min_distance = 999999
        closest_id = -1

        poses: list[Pose] = msg.poses  # type: ignore
        ids: list[int] = msg.marker_ids  # type: ignore

        for i, pose in enumerate(poses):
            # (roll, pitch, yaw) = euler_from_quaternion(pose.orientation)

            # if abs(pitch) > math.radians(20):
            #     continue

            distance = np.sqrt(pose.position.x**2 + pose.position.y**2 + pose.position.z**2)
            if distance < min_distance:
                closest_index = i
                min_distance = distance
                closest_id = ids[i]

        if closest_index is None:
            return None

        # Transforming the closest one to the front_bumper
        pose: Pose = poses[closest_index]
        transformed_pose = self.transform_pose(pose)
        if transformed_pose is None:
            return None

        # Protect against negative sqrt
        if min_distance < ARUCO_HEIGHT:
            return None

        distance = math.sqrt(min_distance**2 - ARUCO_HEIGHT**2)

        return transformed_pose, distance, closest_id

    def transform_pose(self, pose: Pose) -> Union[Pose, None]:
        try:
            transform = self.tf_buffer.lookup_transform(
                TARGET_FRAME,
                SOURCE_FRAME,
                self.get_clock().now().to_msg(),
                timeout=Duration(seconds=1),
            )

            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)

            return transformed_pose
        except Exception as e:
            self.get_logger().error(f"Failed to transform pose from {SOURCE_FRAME} to {TARGET_FRAME}: {e}")
            return None

    def publish_localization(self, transformed_pose: Pose, closest_id: int):
        if self.aruco_mapping is None:
            return

        city_loc_id: Union[str, None] = self.aruco_mapping.get(closest_id)
        if city_loc_id is None:
            self.get_logger().error(f"City loc id not found for aruco id {closest_id}")
            return

        self.get_logger().info(f"Localized at {city_loc_id}")

        localization_msg = Localization()
        localization_msg.city_loc_id = city_loc_id
        localization_msg.pose = transformed_pose
        self.localization_pub.publish(localization_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
