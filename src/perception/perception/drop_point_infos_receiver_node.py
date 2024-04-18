import rclpy
import requests
from rclpy.node import Node

from design3_msgs.msg import DropPointInfo, DropPointInfos, MapInfo, Objective
from planning.utils.robot_state import RobotState

TIMER_PERIOD_S = 1

URL = "http://production.eba-fkhzhyn3.ca-central-1.elasticbeanstalk.com:80"


class DropPointInfosReceiverNode(Node):
    def __init__(self):
        super().__init__("drop_point_infos_receiver_node", parameter_overrides=[])
        self.publisher = self.create_publisher(DropPointInfos, "/drop_point_infos", 10)

        self.create_subscription(Objective, "/objective", self._objective_callback, 10)
        self.create_subscription(MapInfo, "/map_info", self._map_info_callback, 10)
        self.timer = None
        self.map_info = None
        self.get_logger().info("Drop point infos receiver node started")

    def _objective_callback(self, objective: Objective):
        if objective.robot_state == RobotState.WAITING_DROP_POINT_INFOS.value and self.timer is None:
            self.timer = self.create_timer(TIMER_PERIOD_S, self._timer_callback)
        elif self.timer is not None and objective.robot_state != RobotState.WAITING_DROP_POINT_INFOS.value:
            self.timer.cancel()
            del self.timer
            self.timer = None

    def _map_info_callback(self, map_info: MapInfo):
        self.map_info = map_info
        self.get_logger().info("received map info")

    def _timer_callback(self):
        try:
            infos = requests.get(f"{URL}/vehicle").json()
            self.get_logger().info("Requête GET envoyée")
            drop_point_infos = DropPointInfos()
            for key in infos["vehicle_data"]:
                info_id = int(key.strip("zone"))
                # verifying if drop point is on map
                is_drop_point_on_map = False
                if self.map_info is not None:
                    for edge in self.map_info.aruco_mapping:
                        if edge.value[0:2] == "ZC" and edge.value[2:] == str(info_id):
                            is_drop_point_on_map = True
                            break
                    if not is_drop_point_on_map:
                        continue
                info_msg = DropPointInfo()
                info_msg.id = info_id
                info_msg.nb_merchandises = infos["vehicle_data"][key]
                drop_point_infos.drop_points.append(info_msg)  # type: ignore
            self.publisher.publish(drop_point_infos)
            self.get_logger().info("Infos envoyés")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Erreur lors de l'envoi de la requête GET à l'interface : {e}")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la réception des données: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = DropPointInfosReceiverNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
