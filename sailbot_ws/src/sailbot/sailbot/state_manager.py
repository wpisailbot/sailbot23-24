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

#node_names = ["airmar_reader", "ballast_control", "battery_monitor", "computer_vision", "control_system", "computer_vision", "control_system", "network_comms", "pwm_controller", "trim_tab_comms"]
#node_names = ["network_comms","airmar_reader","ballast_control"] 
node_names = ["network_comms", "ballast_control", "pwm_controller", "airmar_reader", "trim_tab_comms"]

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
        self.configure_all_nodes()
        self.activate_all_nodes()
        #self.timer = self.create_timer(2, self.timer_callback)

    async def transitionAllNodes(self, transition_id: int):
        #assemble and run list of async configure transition calls
        failed_names = node_names.copy()
        while failed_names:
            self.get_logger().info("Failed names: "+str(failed_names))
            func_list = [self.changeNodeState(node_name, transition_id) for node_name in failed_names]
            results = await asyncio.gather(*func_list)
            zipped = zip(results, failed_names)
            new_failed_names = []
            for result, name in zipped:
                self.get_logger().info(name+", "+str(result))
                if not result:
                    new_failed_names.append(name)
                    self.get_logger().info("Failed: "+name)
            failed_names = new_failed_names

        self.get_logger().info("returning from transition")
        return True
        #retry any which failed because the service was unavailable (timing issue, any other way to resolve?)
        

    def configure_all_nodes(self):
        self.get_logger().info("Configuring all nodes")
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.transitionAllNodes(Transition.TRANSITION_CONFIGURE))
        self.get_logger().info("returning from configure")
    
    def activate_all_nodes(self):
        self.get_logger().info("activating all nodes")
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.transitionAllNodes(Transition.TRANSITION_ACTIVATE))

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

    async def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0)

    async def changeNodeState(self, node_name: str, transition_id: int, timeout_seconds=3):
        if(not node_name in self.client_state_setters):
            self.get_logger().error("Incorrect or nonexistant node name provided: "+node_name)
            return None
        if(self.client_state_setters[node_name].wait_for_service(timeout_seconds) is False):
            self.get_logger().error("Client service not available for node: "+node_name)
            return False
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.client_state_setters[node_name].call_async(request)
        #TODO: There has to be a better way of doing truly async service calls
        while(future.done() is False):
            await self.spin_once()
            await asyncio.sleep(0.0)
        result = future.result()
        if(result):
            if(result.success):
                self.get_logger().info("State change "+str(transition_id)+" successful for node: "+node_name)
                return True
            else:
                self.get_logger().warn("State change "+str(transition_id)+" unsuccessful for node: "+node_name)
                return False
        else:
            self.get_logger().error("Request "+str(transition_id)+" failed for ChangeState: "+node_name)
            return False

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