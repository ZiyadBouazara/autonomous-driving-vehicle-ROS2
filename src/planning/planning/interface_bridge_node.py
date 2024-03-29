import rclpy
import requests
from rclpy.node import Node

from design3_msgs.msg import Objective

INTERFACE_URL = "URL à mettre ici"


class InterfaceBridgeNode(Node):
    def __init__(self):
        super().__init__("interface_bridge_node", parameter_overrides=[])
        self.subscription = self.create_subscription(Objective, "/objective", self.objective_callback, 10)

    def objective_callback(self, msg: Objective):
        try:
            response = requests.post(
                INTERFACE_URL, data={"robot_state": msg.robot_state, "destination_loc_id": msg.destination_loc_id}
            )
            self.get_logger().info("Requête POST envoyée")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Erreur lors de l'envoi de la requête POST à l'interface : {e}")


def main(args=None):
    rclpy.init(args=args)
    interface_bridge = InterfaceBridgeNode()
    rclpy.spin(interface_bridge)
    interface_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
