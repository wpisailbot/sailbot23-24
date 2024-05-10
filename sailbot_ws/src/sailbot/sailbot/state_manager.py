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
import traceback

class StateManager(Node):
    """
    A ROS2 node that manages the lifecycle states of other nodes within the system. It provides services for restarting
    nodes and functions to transition nodes between different lifecycle states. This manager supports nodes defined
    as managed and can ensure that they are initialized in the correct state during system startup.

    :ivar early_node_names: A list of node names that should be managed before others upon system start.
    :ivar node_names: A list of all other node names to be managed.
    :ivar client_state_getters: Dictionary mapping node names to ROS2 clients for getting current state.
    :ivar client_state_setters: Dictionary mapping node names to ROS2 clients for setting state.

    **Service Servers**:

    - 'restart_node_srv': A service that allows external requests to restart managed nodes.

    **Methods**:

    - 'configure_nodes': Configures a list of nodes using their transition services.
    - 'activate_nodes': Activates a list of nodes, bringing them from the configured state to active.
    - 'transitionNodes': Manages the state transitions for a list of nodes based on specified transition IDs.
    - 'restart_lifecycle_node_callback': Callback for the service that handles requests to restart nodes.
    - 'restart_node': Handles the actual restart of a node by managing its state transitions.
    - 'get_node_state': Retrieves the current state of a node.
    - 'change_node_state': Asynchronously changes the state of a node.
    - 'change_node_state_sync': Synchronously changes the state of a node, suitable for calling from other ROS2 callbacks.

    **Usage**:

    - This node is initialized and run using a ROS2 launch file and interacts with other nodes through service calls to manage their states according to system requirements or external inputs.

    **Notes**:

    - The manager must be aware of all nodes it needs to manage, configured through the managed_nodes parameter.

    """
    early_node_names = ["network_comms"]
    node_names = []
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

    # Can sometimes get stuck? It's rare, but maybe just move to sequential, or find source of deadlock
    def transitionNodes(self, node_names: list, transition_id: int):
        #assemble and run list of async configure transition calls
        failed_names = node_names.copy()
        #retry any which failed because the service was unavailable (timing issue, any other way to resolve?)
        while failed_names:
            self.get_logger().info("Failed names: "+str(failed_names))
            func_list = [self.change_node_state(node_name, transition_id) for node_name in failed_names]
            results = [f for f in func_list]
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
            self.restart_node(node_name)
            response.success = True
            response.message = f"Node {node_name} restarted successfully."
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    def restart_node(self, node_name):
        """
        Manages the state transitions of a specified lifecycle node to effectively restart it. This function checks the
        current state of the node and applies the necessary transitions to bring it to an active state from any other state.

        :param node_name: The name of the node whose state needs to be managed.

        :return: None. This function directly affects the state of the node by sending transition commands based on its current state.

        Function behavior includes:
        - Retrieving the current state of the node.
        - Logging the current state for monitoring and debugging purposes.
        - Applying a series of state transitions to restart the node:
        - If the node is 'active', it will be deactivated, cleaned up, configured, and reactivated.
        - If the node is 'inactive', it will be cleaned up, configured, and activated.
        - If the node is in 'finalized' or 'unconfigured' state, it will be configured and activated.
        - If the node is in an unknown state, a log message will be generated indicating this.
        - These transitions are designed to ensure that the node can recover from any stable state to active status.

        The function relies on the 'change_node_state_sync' method to perform the state transitions, which sends requests to
        the node to change its state according to the ROS2 lifecycle state machine. Each transition is logged to provide a clear
        trace of the actions taken.
        """
        self.get_logger().info("Getting node state")
        current_state = self.get_node_state(node_name)
        self.get_logger().info(f"Current state: {current_state}")
        self.get_logger().info("Setting node state")
        if current_state == 'active':
            self.change_node_state_sync(node_name, Transition.TRANSITION_DEACTIVATE)
            self.change_node_state_sync(node_name, Transition.TRANSITION_CLEANUP)
            self.change_node_state_sync(node_name, Transition.TRANSITION_CONFIGURE)
            self.change_node_state_sync(node_name, Transition.TRANSITION_ACTIVATE)
        elif current_state == 'inactive':
            self.change_node_state_sync(node_name, Transition.TRANSITION_CLEANUP)
            self.change_node_state_sync(node_name, Transition.TRANSITION_CONFIGURE)
            self.change_node_state_sync(node_name, Transition.TRANSITION_ACTIVATE)
        elif current_state == 'finalized' or current_state == 'unconfigured':
            self.change_node_state_sync(node_name, Transition.TRANSITION_CONFIGURE)
            self.change_node_state_sync(node_name, Transition.TRANSITION_ACTIVATE)
        else:
            self.get_logger().info(f'Node {node_name} is in an unknown state: {current_state}')

        

    def configure_nodes(self, node_names: list):
        self.get_logger().info(f"Configuring {len(node_names)} nodes")
        self.transitionNodes(node_names, Transition.TRANSITION_CONFIGURE)
    
    def activate_nodes(self, node_names: list):
        self.get_logger().info(f"activating {len(node_names)} nodes")
        self.transitionNodes(node_names, Transition.TRANSITION_ACTIVATE)

    def get_node_state(self, node_name):
        """
        Retrieves the current state of a specified node using a ROS2 service call. If the service is not available within three seconds,
        returns 'unknown'.

        :param node_name: The name of the node whose state is to be retrieved.

        :return: The label of the current state of the node as a string, or 'unknown' if the service is not available within the timeout.

        Function behavior includes:
        - Checking the availability of the 'GetState' service for the specified node up to three times, with one second wait each.
        - If the service becomes available, sending a request to obtain the current state of the node.
        - If the service is not available within three attempts, returning 'unknown'.
        - Logging each attempt to connect to the service and the outcome.
        """
        # Create a service client for GetState
        cli = self.client_state_getters[node_name]#self.create_client(GetState, f'{node_name}/get_state')
        attempts = 0
        while not cli.wait_for_service(timeout_sec=1.0):
            attempts += 1
            self.get_logger().info(f'Attempt {attempts}: Waiting for {node_name} GetState service...')
            if attempts >= 3:
                self.get_logger().warn('GetState service not available!.')
                return 'unknown'
        
        request = GetState.Request()
        result = cli.call(request)
        return result.current_state.label


    def change_node_state(self, node_name, transition):
        """
        Sends a request to change the state of a specified node via a ROS2 service call. This function manages node transitions
        using the ROS2 lifecycle protocol by interacting with a service dedicated to changing node states. 
        This function should NOT be used from ROS2 callbacks- timers, subscriptions, service callbacks, etc, as the async spin 
        WILL deadlock this node. This is used at startup to launch many nodes simultaneously. Callbacks should instead use change_node_state_sync.

        :param node_name: The name of the node whose state is to be changed.
        :param transition: The transition ID that needs to be applied to the node. This ID should correspond to one of the
                        predefined transition states in the ROS2 lifecycle.

        :return: A boolean value indicating the success or failure of the state change. Returns True if the state transition was
                successful, False otherwise.

        Function behavior includes:
        - Checking for the availability of the service that allows state changes for the specified node. If the service is not
        available immediately, it waits up to one second for it to become available.
        - Creating and sending a request to change the state of the node based on the specified transition.
        - Using asynchronous service calls to request the transition and waiting for the call to complete.
        - Logging the process and outcome of the service call, including waiting for the service, the attempt to change state,
        and the result of the call.

        This function assumes that a client for the node-specific 'ChangeState' service has already been set up in 'client_state_setters'.
        It handles possible delays in service availability and logs significant actions and outcomes to aid in debugging and monitoring.
        """
        # Create a service client for ChangeState
        cli = self.client_state_setters[node_name]
        attempts = 0
        while not cli.wait_for_service(timeout_sec=1.0):
            attempts += 1
            self.get_logger().info(f'Attempt {attempts}: Waiting for {node_name} ChangeState service...')
            if attempts >= 3:
                self.get_logger().warn('ChangeState service not available!.')
                return False
        
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
    
    def change_node_state_sync(self, node_name, transition):
        """
        Sends a request to change the state of a specified node via a ROS2 service call. This function manages node transitions
        using the ROS2 lifecycle protocol by interacting with a service dedicated to changing node states. 
        This function is intended for use in callbacks, and will not deadlock this node.

        :param node_name: The name of the node whose state is to be changed.
        :param transition: The transition ID that needs to be applied to the node. This ID should correspond to one of the
                        predefined transition states in the ROS2 lifecycle.

        :return: A boolean value indicating the success or failure of the state change. Returns True if the state transition was
                successful, False otherwise.

        Function behavior includes:
        - Checking for the availability of the service that allows state changes for the specified node. If the service is not
        available immediately, it waits up to one second for it to become available.
        - Creating and sending a request to change the state of the node based on the specified transition.
        - Using synchronous service calls to request the transition.
        - Logging the process and outcome of the service call, including waiting for the service, the attempt to change state,
        and the result of the call.

        This function assumes that a client for the node-specific 'ChangeState' service has already been set up in 'client_state_setters'.
        It handles possible delays in service availability and logs significant actions and outcomes to aid in debugging and monitoring.
        """
        # Create a service client for ChangeState
        cli = self.client_state_setters[node_name]
        attempts = 0
        while not cli.wait_for_service(timeout_sec=1.0):
            attempts += 1
            self.get_logger().info(f'Attempt {attempts}: Waiting for {node_name} ChangeState service...')
            if attempts >= 3:
                self.get_logger().warn('ChangeState service not available!.')
                return False
        
        request = ChangeState.Request()
        request.transition.id = transition
        self.get_logger().info("Before call")
        result = cli.call(request)

        if result is not None:
            self.get_logger().info(f'State changed successfully in {node_name}')
            return True
        else:
            self.get_logger().error('Failed to call change_state service')
            return False
            

def main(args=None):
    rclpy.init(args=args)
    state_manager = StateManager()

    # Use the SingleThreadedExecutor to spin the node.
    executor = MultiThreadedExecutor()
    executor.add_node(state_manager)

    try:
        # Spin the node to execute callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        trace = traceback.format_exc()
        state_manager.get_logger().fatal(f'Unhandled exception: {e}\n{trace}')
    finally:
        # Shutdown and cleanup the node
        executor.shutdown()
        state_manager.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()