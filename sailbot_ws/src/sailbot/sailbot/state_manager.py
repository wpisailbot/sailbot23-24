import rclpy
from rclpy.node import Node
from rclpy.client import Client
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import State, Transition
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from sailbot_msgs.srv import RestartNode
from enum import Enum
import typing
from functools import partial
import asyncio
from asyncio import Future
from threading import Event
import threading

class BoatState(Enum):
    INACTIVE=1
    INITIALIZING = 2
    IDLE = 3
    STATION_KEEPING=4
    WAYPOINT_FOLLOWING=5

#node_names = ["airmar_reader", "ballast_control", "battery_monitor", "computer_vision", "control_system", "computer_vision", "control_system", "network_comms", "pwm_controller", "trim_tab_comms"]

class StateManager(Node):
    early_node_names = ["network_comms"]
    #node_names = ["ballast_control", "pwm_controller", "airmar_reader", "trim_tab_comms"]
    node_names = []
    current_state = BoatState.INACTIVE
    client_state_getters: typing.Dict[str, Client] = {}
    client_state_setters: typing.Dict[str, Client] = {}
    def __init__(self):
        super().__init__("state_manager")
        self.get_logger().info("starting manager")
        self.declare_parameter('managed_nodes')
        self.node_names = self.get_parameter('managed_nodes').get_parameter_value().string_array_value
        self.get_logger().info(f'Managed nodes: {self.node_names}')
        self.callback_group_input = ReentrantCallbackGroup()
        self.callback_group_state = ReentrantCallbackGroup()

        #This service will not work, I believe because of a bug in RCLPY service calls which call lifecycle state transition services. Leaving it here for future fix.
        self.restart_node_srv = self.create_service(RestartNode, 'state_manager/restart_node', self.restart_lifecycle_node_callback, callback_group=self.callback_group_input)

        #create service clients for each node
        for name in self.early_node_names:
            self.client_state_getters[name] = self.create_client(GetState, name+"/get_state")
            self.client_state_setters[name] = self.create_client(ChangeState, name+"/change_state")
        for name in self.node_names:
            self.client_state_getters[name] = self.create_client(GetState, name+"/get_state")
            self.client_state_setters[name] = self.create_client(ChangeState, name+"/change_state")
        
        #run async function to move nodes to configured state
        self.get_logger().info("Configuring")
        self.configure_nodes(self.early_node_names)
        self.get_logger().info("Activating")
        self.activate_nodes(self.early_node_names)
        self.configure_nodes(self.node_names)
        self.activate_nodes(self.node_names)
        #self.timer = self.create_timer(2, self.timer_callback)

    def transitionNodes(self, node_names: list, transition_id: int):
        #assemble and run list of async configure transition calls
        failed_names = node_names.copy()
        #retry any which failed because the service was unavailable (timing issue, any other way to resolve?)
        while failed_names:
            self.get_logger().info("Failed names: "+str(failed_names))
            func_list = [self.change_node_state(node_name, transition_id) for node_name in failed_names]
            results = [f for f in func_list]#await asyncio.gather(*func_list)
            zipped = zip(results, failed_names)
            new_failed_names = []
            for result, name in zipped:
                self.get_logger().info(name+", "+str(result))
                if not result:
                    new_failed_names.append(name)
                    self.get_logger().info("Failed: "+name)
            failed_names = new_failed_names

        return True
    
    def restart_lifecycle_node_callback(self, request, response):
        node_name = request.node_name
        try:
            #self.restart_node(node_name)
            response.success = True
            response.message = f"Node {node_name} restarted successfully."
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    def restart_node(self, node_name):
        current_state = self.get_node_state(node_name)

        if current_state in ['unconfigured', 'inactive', 'active']:
            self.change_node_state(node_name, Transition.TRANSITION_DEACTIVATE)
            self.change_node_state(node_name, Transition.TRANSITION_CLEANUP)
            self.change_node_state(node_name, Transition.TRANSITION_CONFIGURE)
            self.change_node_state(node_name, Transition.TRANSITION_ACTIVATE)
        elif current_state == 'finalized' or current_state is None:
            self.change_node_state(node_name, Transition.TRANSITION_CONFIGURE)
            self.change_node_state(node_name, Transition.TRANSITION_ACTIVATE)
        else:
            self.get_logger().info(f'Node {node_name} is in an unknown state: {current_state}')

        

    def configure_nodes(self, node_names: list):
        self.get_logger().info(f"Configuring {len(node_names)} nodes")
        self.transitionNodes(node_names, Transition.TRANSITION_CONFIGURE)
    
    def activate_nodes(self, node_names: list):
        self.get_logger().info(f"activating {len(node_names)} nodes")
        self.transitionNodes(node_names, Transition.TRANSITION_ACTIVATE)

    def get_node_state(self, node_name):
        # Create a service client for GetState
        cli = self.client_state_getters[node_name]#self.create_client(GetState, f'{node_name}/get_state')
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {node_name} GetState service...')
        
        request = GetState.Request()
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().current_state.label
        else:
            self.get_logger().error('Failed to call get_state service')
            return None
    

    async def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0)

    def change_node_state(self, node_name, transition):
        # Create a service client for ChangeState
        cli = self.client_state_setters[node_name]#self.create_client(ChangeState, f'{node_name}/change_state')
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {node_name} ChangeState service...')
        
        request = ChangeState.Request()
        request.transition.id = transition
        self.get_logger().info("Before call")
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'State changed successfully in {node_name}')
            return True
        else:
            self.get_logger().error('Failed to call change_state service')
            return False
            

def main(args=None):
    rclpy.init(args=args)
    state_manager = StateManager()

    # Use the SingleThreadedExecutor to spin the node.
    executor = SingleThreadedExecutor()
    executor.add_node(state_manager)

    try:
        # Spin the node to execute callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        state_manager.get_logger().fatal(f'Unhandled exception: {e}')
    finally:
        # Shutdown and cleanup the node
        executor.shutdown()
        state_manager.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()