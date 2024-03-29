from typing import Tuple, Union

import networkx as nx
import rclpy
from rclpy.node import Node

from design3_msgs.msg import Edge, MapInfo, Objective, Path
from planning.utils.robot_state import RobotState


def build_graph(map_info: MapInfo) -> "Tuple[nx.DiGraph, dict[Tuple[str, str], str]]":
    G = nx.DiGraph()

    dic_edge_labels = {}
    graph: list[Edge] = map_info.graph  # type: ignore

    for row in graph:
        start_point = row.origin
        direction = row.direction
        end_point = row.destination

        dic_edge_labels[(start_point, end_point)] = direction

        G.add_edge(start_point, end_point, edge_label=direction, weight=1)

    return G, dic_edge_labels


def build_edges(shortest_path: "list[str]", dic_edge_labels: "dict[Tuple[str, str], str]"):
    if len(shortest_path) < 2:
        return []

    edges: list[Edge] = []
    for i in range(len(shortest_path) - 1):
        edge = Edge()
        edge.origin = shortest_path[i]
        edge.destination = shortest_path[i + 1]
        edge.direction = dic_edge_labels[(shortest_path[i], shortest_path[i + 1])]

        edges.append(edge)

    return edges


class PathPlanningNode(Node):
    def __init__(self):
        super().__init__("path_planning_node", parameter_overrides=[])

        self.create_subscription(MapInfo, "map_info", self.map_info_callback, 10)
        self.create_subscription(Objective, "objective", self.objective_callback, 10)

        self.path_pub = self.create_publisher(Path, "path", 10)

        self.graph: Union[nx.DiGraph, None] = None
        self.dic_edge_labels: dict[Tuple[str, str], str] = {}

        self.get_logger().info("Path planning node started")

    def map_info_callback(self, map_info: MapInfo):
        self.graph, self.dic_edge_labels = build_graph(map_info)
        self.get_logger().info("Map received and graph successfully built")

    def objective_callback(self, objective: Objective):
        # If we are not in one of these state, no path to compute
        if (
            objective.robot_state != RobotState.GOING_TO_DEPOT.value
            and objective.robot_state != RobotState.GOING_TO_DROP_POINT.value
        ):
            return

        if self.graph is None or self.dic_edge_labels is None:
            self.get_logger().error("Can't find path, because there is no graph!")
            return

        origin = objective.from_loc_id
        destination = objective.destination_loc_id

        if origin == "" or destination == "":
            self.get_logger().error("Can't find path, because origin or destination is empty!")
            return
        elif origin == destination:
            self.get_logger().error(f"Origin and destination are the same: {origin}")
            return

        self.get_logger().info(f"Computing path {origin} -> {destination}")

        try:
            shortest_path: list[str] = nx.shortest_path(self.graph, source=origin, target=destination)  # type: ignore
            self.publish_path(shortest_path)
        except nx.NetworkXNoPath:
            self.get_logger().error(f"No path found between {origin} and {destination}.")
        except nx.NodeNotFound:
            self.get_logger().error(f"Node not found in the graph.")
        except:
            self.get_logger().error("An error occurred while computing the path.")

    def publish_path(self, shortest_path: "list[str]"):
        path = Path()
        path.edges = build_edges(shortest_path, self.dic_edge_labels)

        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)

    path_planning_node = PathPlanningNode()

    rclpy.spin(path_planning_node)

    path_planning_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
