from typing import Union

import rclpy
from rclpy.node import Node

from design3_msgs.msg import Edge, Localization, Path


class PathFollowingNode(Node):
    def __init__(self):
        super().__init__("path_following_node", parameter_overrides=[])

        self.create_subscription(Path, "path", self.path_callback, 10)
        self.create_subscription(Localization, "localization", self.localization_callback, 10)

        self.current_edge_pub = self.create_publisher(Edge, "current_edge", 10)

        self.path: Union[Path, None] = None
        self.current_edge: Union[Edge, None] = None

        self.get_logger().info("Path following node started")

    def reset_path(self):
        self.path = None
        self.current_edge = None

    def path_callback(self, path: Path):
        self.reset_path()

        if len(path.edges) == 0:
            self.get_logger().warn("Empty path received, nothing to follow")
            return

        edges: list[Edge] = path.edges  # type: ignore
        origin = edges[0].origin
        destination = edges[-1].destination
        if origin == destination:
            self.get_logger().warn("Path with same origin and destination received, nothing to follow")
            return

        self.path = path
        self.current_edge = edges[0]
        self.publish_current_edge()

        self.get_logger().info(
            f"New path received {origin} -> {destination}, now following edge {self.current_edge.origin} -- {self.current_edge.direction} --> {self.current_edge.destination}"
        )

    def localization_callback(self, localization: Localization):
        # If there is no path to follow yet, do nothing
        if self.path is None or self.current_edge is None:
            return

        # If we have arrived at the destination of the current edge, move to the next edge
        if localization.city_loc_id == self.current_edge.destination:
            # If there are no more edges to follow, reset the path
            if len(self.path.edges) == 1:
                self.get_logger().info(f"Arrived at {localization.city_loc_id}, no more edges to follow")
                self.reset_path()
                return

            edges: list[Edge] = self.path.edges  # type: ignore
            edges.pop(0)
            self.current_edge = edges[0]

            self.publish_current_edge()

            self.get_logger().info(
                f"Arrived at {localization.city_loc_id}, now following new edge {self.current_edge.origin} -- {self.current_edge.direction} --> {self.current_edge.destination}"
            )

    def publish_current_edge(self):
        if self.current_edge is None:
            return

        self.current_edge_pub.publish(self.current_edge)


def main(args=None):
    rclpy.init(args=args)

    path_following_node = PathFollowingNode()

    rclpy.spin(path_following_node)

    path_following_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
