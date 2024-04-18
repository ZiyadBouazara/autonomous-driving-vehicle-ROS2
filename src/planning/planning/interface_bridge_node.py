import rclpy
import requests
from rclpy.node import Node
from std_msgs.msg import Bool

from design3_msgs.msg import Edge, Objective

INTERFACE_URLS = ["http://192.168.12.56:5000", "http://192.168.12.81:5000", "http://192.168.12.124:5000"]


class InterfaceBridgeNode(Node):
    def __init__(self):
        super().__init__("interface_bridge_node", parameter_overrides=[])
        self.create_subscription(Objective, "/objective", self.objective_callback, 10)
        self.create_subscription(Edge, "/current_edge", self.edge_callback, 10)
        self.create_subscription(Bool, "/is_car_loaded", self.car_loaded_callback, 10)
        self.create_subscription(Bool, "/is_car_close", self.detected_car_callback, 10)

        self.healthy_interfaces = []
        self.healthy_interfaces_timer()
        self.timer = self.create_timer(15, self.healthy_interfaces_timer)
        self.get_logger().info("interface bridge node started")

    def objective_callback(self, msg: Objective):
        for interface_url in self.healthy_interfaces:
            try:
                requests.post(
                    interface_url + "/objective",
                    json={
                        "robot_state": msg.robot_state,
                        "destination_loc_id": msg.destination_loc_id,
                        "from_loc_id": msg.from_loc_id,
                        "robot_state_description": msg.robot_state_description,
                    },
                    timeout=0.5,
                )
            except requests.exceptions.RequestException as e:
                continue

    def detected_car_callback(self, msg: Bool):
        for interface_url in self.healthy_interfaces:
            try:
                requests.post(interface_url + "/detected_car", json={"message": msg.data}, timeout=0.5)
                self.get_logger().info("sent detected car")
            except requests.exceptions.RequestException as e:
                continue

    def edge_callback(self, msg: Edge):
        for interface_url in self.healthy_interfaces:
            try:
                requests.post(
                    interface_url + "/edge",
                    json={"direction": msg.direction, "origin": msg.origin, "destination": msg.destination},
                    timeout=0.5,
                )
            except requests.exceptions.RequestException as e:
                continue

    def car_loaded_callback(self, msg: Bool):
        for interface_url in self.healthy_interfaces:
            try:
                requests.post(
                    interface_url + "/car_loaded",
                    json={"message": msg.data},
                    headers={"Content-Type": "application/json"},
                    timeout=0.5,
                )
            except requests.exceptions.RequestException as e:
                continue

    def healthy_interfaces_timer(self):
        for interface_url in INTERFACE_URLS:
            try:
                requests.get(interface_url + "/health", timeout=0.5)
                if interface_url not in self.healthy_interfaces:
                    self.get_logger().info(f"Interface {interface_url} is healthy")
                    self.healthy_interfaces.append(interface_url)
            except requests.exceptions.RequestException as e:
                if interface_url in self.healthy_interfaces:
                    self.healthy_interfaces.remove(interface_url)


def main(args=None):
    rclpy.init(args=args)
    interface_bridge = InterfaceBridgeNode()
    rclpy.spin(interface_bridge)
    interface_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
