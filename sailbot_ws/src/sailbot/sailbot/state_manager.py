import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from enum import Enum

class BoatState(Enum):
    INACTIVE=1
    INITIALIZING = 2
    IDLE = 3
    STATION_KEEPING=4
    WAYPOINT_FOLLOWING=5

class StateManager(Node):
    current_state = BoatState.INACTIVE
    client_state_getters = {}
    client_state_setters = {}
    def __init__(self):
        super().__init__("state_manager")
        self.client_stata_getters["ballast_control"] = self.create_client(GetState, "ballast_control/get_state")
        self.client_state_setters["ballast_control"] = self.create_client(ChangeState, "ballast_control/change_state")
    


def main(args=None):
    rclpy.init(args=args)
    state_manager = StateManager()
    rclpy.spin(state_manager)
    rclpy.shutdown()

if __name__ == "__main__":
    main()