import time
from typing import Union

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty

from design3_msgs.msg import DropPointInfo, DropPointInfos, Localization, MapInfo, Objective
from planning.utils.robot_state import RobotState

INITIAL_STATE = RobotState.WAITING_MAP_INFO
INITIAL_HOME_DEPOT = ""


def choose_drop_point(drop_point_infos: DropPointInfos) -> Union[DropPointInfo, None]:
    if len(drop_point_infos.drop_points) == 0:
        return None

    drop_points: list[DropPointInfo] = drop_point_infos.drop_points  # type: ignore

    # Choose the one with the most merchandises
    return max(drop_points, key=lambda x: x.nb_merchandises)


class ObjectiveManagerNode(Node):
    def __init__(self):
        super().__init__("objective_manager_node", parameter_overrides=[])

        self.create_subscription(MapInfo, "map_info", self.map_info_callback, 10)
        self.create_subscription(DropPointInfos, "drop_point_infos", self.drop_point_infos_callback, 10)
        self.create_subscription(Bool, "is_car_loaded", self.optical_sensor_callback, 10)
        self.create_subscription(Localization, "localization", self.localization_callback, 10)
        self.create_subscription(Empty, "navigation_start", self.navigation_start_callback, 10)
        self.create_subscription(Objective, "change_state", self.change_state_callback, 10)

        self.objective_pub = self.create_publisher(Objective, "objective", 10)

        self.home_depot: str = INITIAL_HOME_DEPOT
        self.last_loc_id: str = ""
        self.last_start_time = time.time()

        self.get_logger().info("Objective manager node started")

        self.change_state(INITIAL_STATE)

    def change_state(
        self,
        new_state: RobotState,
        new_destination_id: str = "",
        current_loc_id: str = "",
    ):
        self.current_state = new_state
        self.get_logger().info(
            f"Changed state: {new_state.name} {current_loc_id} {'->' if new_destination_id != '' else ''} {new_destination_id}"
        )

        objective = Objective()
        objective.robot_state = new_state.value
        objective.robot_state_description = new_state.name
        objective.from_loc_id = current_loc_id
        objective.destination_loc_id = new_destination_id
        self.objective_pub.publish(objective)

        self.last_objective = objective

    def change_state_callback(self, objective: Objective):
        self.get_logger().warn("Manual state change requested...")
        self.change_state(
            RobotState(objective.robot_state),
            objective.destination_loc_id,
            objective.from_loc_id,
        )

    def map_info_callback(self, _):
        # Once map is received, we wait for navigation start
        if self.current_state.value == RobotState.WAITING_MAP_INFO.value:
            self.change_state(RobotState.WAITING_NAVIGATION_START)

    def navigation_start_callback(self, _):
        # Once navigation start is received, we wait for drop point infos
        if self.current_state.value == RobotState.WAITING_NAVIGATION_START.value:
            if self.home_depot == "":
                self.get_logger().error("No home depot received, can't start navigation")
                return

            self.change_state(RobotState.WAITING_DROP_POINT_INFOS)

    def drop_point_infos_callback(self, drop_point_infos: DropPointInfos):
        # If we are not waiting for drop point infos, we ignore them
        if self.current_state.value != RobotState.WAITING_DROP_POINT_INFOS.value:
            return

        if len(drop_point_infos.drop_points) == 0:
            self.get_logger().error("No drop points received")
            return

        chosen_drop_point = choose_drop_point(drop_point_infos)
        if chosen_drop_point is None:
            self.get_logger().error("No drop point chosen")
            return

        depot_loc_id = f"ZC{chosen_drop_point.id}"
        self.change_state(RobotState.GOING_TO_DROP_POINT, depot_loc_id, self.last_loc_id)
        self.last_start_time = time.time()

    def localization_callback(self, localization: Localization):
        self.last_loc_id = localization.city_loc_id

        # Localization received when waiting navigation start will be our home depot
        if self.current_state.value == RobotState.WAITING_NAVIGATION_START.value:
            self.home_depot = localization.city_loc_id
            self.get_logger().info(f"Home depot: {self.home_depot}")
            return

        # If we are going to a drop point and we are at the drop point, change state
        elif (
            self.current_state.value == RobotState.GOING_TO_DROP_POINT.value
            and localization.city_loc_id == self.last_objective.destination_loc_id
        ):
            self.change_state(RobotState.ARRIVED_AT_DROP_POINT)
            return

        # If we are going to the depot and we are at the depot, change state
        elif (
            self.current_state.value == RobotState.GOING_TO_DEPOT.value and localization.city_loc_id == self.home_depot
        ):
            self.change_state(RobotState.ARRIVED_AT_DEPOT)
            return
        # If we see back our depot at one of these state, we restart the loop
        elif (
            self.current_state.value == RobotState.GOING_TO_DROP_POINT.value
            or self.current_state.value == RobotState.ARRIVED_AT_DROP_POINT.value
            or self.current_state.value == RobotState.GOING_TO_DEPOT.value
        ) and localization.city_loc_id == self.home_depot:
            current_time = time.time()

            SECOND_THRESOLD = 5
            if current_time - self.last_start_time > SECOND_THRESOLD:
                self.get_logger().warn("Back to depot :( Relocalizing...")
                self.change_state(RobotState.WAITING_DROP_POINT_INFOS)
                return

    def optical_sensor_callback(self, is_car_loaded: Bool):
        # If we are at the drop point and the car is loaded, go to the depot
        if self.current_state.value == RobotState.ARRIVED_AT_DROP_POINT.value and is_car_loaded.data:
            self.change_state(RobotState.GOING_TO_DEPOT, self.home_depot, self.last_loc_id)
            return

        # If we are at the depot and the car is unloaded, go to the drop point
        elif self.current_state.value == RobotState.ARRIVED_AT_DEPOT.value and not is_car_loaded.data:
            self.change_state(RobotState.WAITING_DROP_POINT_INFOS)
            return


def main(args=None):
    rclpy.init(args=args)

    objective_manager_node = ObjectiveManagerNode()

    rclpy.spin(objective_manager_node)

    objective_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
