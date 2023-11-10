import rclpy
from rclpy.node import Node
from rclpy.client import Client
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import State, Transition
from enum import Enum
import typing
import types
import functools
from functools import partial
import asyncio
from asyncio import Future

def copy_func(f):
    """Based on http://stackoverflow.com/a/6528148/190597 (Glenn Maynard)"""
    g = types.FunctionType(f.__code__, f.__globals__, name=f.__name__,
                           argdefs=f.__defaults__,
                           closure=f.__closure__)
    g = functools.update_wrapper(g, f)
    g.__kwdefaults__ = f.__kwdefaults__
    return g

class BoatState(Enum):
    INACTIVE=1
    INITIALIZING = 2
    IDLE = 3
    STATION_KEEPING=4
    WAYPOINT_FOLLOWING=5

#node_names = ["airmar_reader", "ballast_control", "battery_monitor", "computer_vision", "control_system", "computer_vision", "control_system", "network_comms", "pwm_controller", "trim_tab_comms"]
node_names = ["airmar_reader","ballast_control"] 

class StateManager(Node):
    current_state = BoatState.INACTIVE
    client_state_getters: typing.Dict[str, Client] = {}
    client_state_setters: typing.Dict[str, Client] = {}
    def __init__(self):
        super().__init__("state_manager")
        self.get_logger().info("starting manager")

        #create service clients for each node
        for name in node_names:
            self.client_state_getters[name] = self.create_client(GetState, name+"/get_state")
            self.client_state_setters[name] = self.create_client(ChangeState, name+"/change_state")
        
        #run async function to move all nodes to configured state
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.configure())
        #self.timer = self.create_timer(2, self.timer_callback)

    async def configure(self):
        self.get_logger().info("in configure")
        #assemble and run list of async configure transition calls
        func_list = [self.changeNodeState(node_name, Transition.TRANSITION_CONFIGURE) for node_name in node_names]
        results = await asyncio.gather(*func_list)
        #retry any which failed because the service was unavailable (timing issue, any other way to resolve?)
        retry_task_names = [name for name, result in zip(node_names, results) if result is False]
        while retry_task_names:
            retry_tasks = [self.changeNodeState(node_name, Transition.TRANSITION_CONFIGURE) for node_name in retry_task_names]
            results = await asyncio.gather(*retry_tasks)
            retry_task_names = [name for name, result in zip(node_names, results) if result is False]
        # loop = asyncio.get_event_loop()
        # func_list = [asyncio.ensure_future(self.get_changeNodeState()(node_name, Transition.TRANSITION_CONFIGURE)) for node_name in node_names]
        # self.get_logger().info("running as_compelted")
        # for f in asyncio.as_completed(func_list):
        #     result = await f
        #     await self.get_change_node_state_future_callback()(result, "test", -1)
        # loop.close()

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
            return None
        if(self.client_state_setters[node_name].wait_for_service(timeout_seconds) is False):
            self.get_logger().error("Client service not available for node: "+node_name)
            return False
        request = ChangeState.Request()
        request.transition.id = transition_id
        self.get_logger().info("awaiting change state service")
        future: Future = self.client_state_setters[node_name].call_async(request)
        partial_callback = partial(self.get_change_node_state_future_callback(), self=self, node_name=node_name, transition_id=transition_id)
        future.add_done_callback(partial_callback)
        return None

    def get_change_node_state_future_callback(this):
        async def change_node_state_future_callback(future: Future, self, node_name: str, transition_id: int):
            self.get_logger().info("in change state callback")
            result: ChangeState.Response = future.result()
            if(result):
                if(result.success):
                    self.get_logger().info("State change "+str(transition_id)+" successful for node: "+node_name)
                else:
                    self.get_logger().warn("State change "+str(transition_id)+" unsuccessful for node: "+node_name)
            else:
                self.get_logger().error("Request "+str(transition_id)+" failed for ChangeState: "+node_name)
        
        return change_node_state_future_callback

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