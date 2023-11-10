import rclpy
from rclpy.node import Node
from rclpy.client import Client
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import State, Transition
from enum import Enum
import typing
from functools import partial
import asyncio
from asyncio import Future

class BoatState(Enum):
    INACTIVE=1
    INITIALIZING = 2
    IDLE = 3
    STATION_KEEPING=4
    WAYPOINT_FOLLOWING=5

class StateManager(Node):
    current_state = BoatState.INACTIVE
    client_state_getters: typing.Dict[str, Client] = {}
    client_state_setters: typing.Dict[str, Client] = {}
    def __init__(self):
        super().__init__("state_manager")
        self.get_logger().info("starting manager")
        self.client_state_getters["ballast_control"] = self.create_client(GetState, "ballast_control/get_state")
        self.client_state_setters["ballast_control"] = self.create_client(ChangeState, "ballast_control/change_state")
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.configure())
        #self.timer = self.create_timer(2, self.timer_callback)

    async def configure(self):
        self.get_logger().info("in configure")
        await self.changeNodeState("ballast_control", Transition.TRANSITION_CONFIGURE)

    async def getNodeState(self, node_name: str, timeout_seconds=3):
        if(not node_name in self.client_state_getters):
            self.get_logger().error("Incorrect or nonexistant node name provided: "+node_name)
            return State.PRIMARY_STATE_UNKNOWN
        if(self.client_state_getters[node_name].wait_for_service(timeout_seconds) is False):
            self.get_logger().error("Client service not available for node: "+node_name)
            return State.PRIMARY_STATE_UNKNOWN
        
        request = GetState.Request()
        self.get_logger().info("awaiting get state service")
        result = self.client_state_getters[node_name].call_async(request)
        partial_callback = partial(self.get_node_state_future_callback, node_name=node_name)
        result.add_done_callback(partial_callback)

    def get_node_state_future_callback(self, future: Future, node_name: str):
        self.get_logger().info("in callback")
        result: GetState.Response = future.result()
        if(result):
            self.get_logger().info("Node "+node_name+" is in state: "+result.current_state.label)
        else:
            self.get_logger().info("Request failed for GetState: "+node_name)

    async def changeNodeState(self, node_name: str, transition_id: int, timeout_seconds=3):
        self.get_logger().info("In change state")
        if(not node_name in self.client_state_setters):
            self.get_logger().error("Incorrect or nonexistant node name provided: "+node_name)
            return
        if(self.client_state_setters[node_name].wait_for_service(timeout_seconds) is False):
            self.get_logger().error("Client service not available for node: "+node_name)
            return
        request = ChangeState.Request()
        request.transition.id = transition_id
        self.get_logger().info("awaiting change state service")
        result = self.client_state_setters[node_name].call_async(request)
        partial_callback = partial(self.change_node_state_future_callback, node_name=node_name, transition_id=transition_id)
        result.add_done_callback(partial_callback)

    def change_node_state_future_callback(self, future: Future, node_name: str, transition_id: int):
        self.get_logger().info("in change state callback")
        result: ChangeState.Response = future.result()
        if(result):
            if(result.success):
                self.get_logger().info("State change "+str(transition_id)+" successful for node: "+node_name)
            else:
                self.get_logger().warn("State change "+str(transition_id)+" unsuccessful for node: "+node_name)
        else:
            self.get_logger().error("Request "+str(transition_id)+" failed for ChangeState: "+node_name)

    async def timer_callback(self):
        self.get_logger().info("Getting state")
        await self.getNodeState("ballast_control")
            

def main(args=None):
    rclpy.init(args=args)
    state_manager = StateManager()
    rclpy.spin(state_manager)
    rclpy.shutdown()

if __name__ == "__main__":
    main()