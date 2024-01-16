#!/usr/bin/env python3
import sys
import rclpy
from typing import Optional
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.subscription import Subscription
from std_msgs.msg import String,  Int8, Int16, Empty, Float64
from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.msg import State as StateMsg
from sensor_msgs.msg import NavSatFix
from sailbot_msgs.msg import Wind
import grpc
from concurrent import futures
import json
import math
import time

from sailbot_msgs.srv import RestartNode

import telemetry_messages.python.boat_state_pb2 as boat_state_pb2
import telemetry_messages.python.boat_state_pb2_grpc as boat_state_pb2_rpc
import telemetry_messages.python.control_pb2 as control_pb2
import telemetry_messages.python.control_pb2_grpc as control_pb2_grpc
import telemetry_messages.python.node_restart_pb2 as node_restart_pb2
import telemetry_messages.python.node_restart_pb2_grpc as node_restart_pb2_grpc

def make_json_string(json_msg):
    json_str = json.dumps(json_msg)
    message = String()
    message.data = json_str
    return message

def get_state(state_id: int):
    if state_id == StateMsg.PRIMARY_STATE_ACTIVE:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_ACTIVE
    if state_id == StateMsg.PRIMARY_STATE_INACTIVE:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_INACTIVE
    if state_id == StateMsg.PRIMARY_STATE_FINALIZED:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_FINALIZED
    if state_id == StateMsg.PRIMARY_STATE_UNCONFIGURED:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_UNCONFIGURED
    if state_id == StateMsg.PRIMARY_STATE_UNKNOWN:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_UNKNOWN
    if state_id == StateMsg.TRANSITION_STATE_ACTIVATING:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_ACTIVATING
    if state_id == StateMsg.TRANSITION_STATE_CLEANINGUP:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_CLEANING_UP
    if state_id == StateMsg.TRANSITION_STATE_CONFIGURING:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_CONFIGURING
    if state_id == StateMsg.TRANSITION_STATE_DEACTIVATING:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_DEACTIVATING
    if state_id == StateMsg.TRANSITION_STATE_ERRORPROCESSING:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_ERROR_PROCESSING
    if state_id == StateMsg.TRANSITION_STATE_SHUTTINGDOWN:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_SHUTTINGDOWN
    return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_UNKNOWN

class NetworkComms(LifecycleNode):

    current_boat_state = boat_state_pb2.BoatState()

    def __init__(self):
        super().__init__('network_comms')
        self.pwm_control_publisher: Optional[Publisher]
        self.ballast_position_publisher: Optional[Publisher]
        self.trim_tab_control_publisher: Optional[Publisher]
        self.trim_tab_angle_publisher: Optional[Publisher]

        self.rot_subscription: Optional[Subscription]
        self.navsat_subscription: Optional[Subscription]
        self.track_degrees_true_subscription: Optional[Subscription]
        self.track_degrees_magnetic_subscription: Optional[Subscription]
        self.speed_knots_subscription: Optional[Subscription]
        self.speed_kmh_subscription: Optional[Subscription]
        self.heading_subscription: Optional[Subscription]
        self.true_wind_subscription: Optional[Subscription]
        self.apparent_wind_subscription: Optional[Subscription]
        self.roll_subscription: Optional[Subscription]
        self.pitch_subscription: Optional[Subscription]
        self.pwm_heartbeat_subscription: Optional[Subscription]
        self.control_system_subscription: Optional[Subscription]

        #receives state updates from other nodes
        self.airmar_reader_lifecycle_state_subscriber: Optional[Subscription]

        self.callback_group_state = MutuallyExclusiveCallbackGroup()
        
    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")
        self.pwm_control_publisher = self.create_lifecycle_publisher(String, 'pwm_control', 10)
        self.ballast_position_publisher = self.create_lifecycle_publisher(Float64, 'ballast_position', 10)
        self.trim_tab_control_publisher = self.create_lifecycle_publisher(Int8, 'tt_control', 10)
        self.trim_tab_angle_publisher = self.create_lifecycle_publisher(Int16, 'tt_angle', 10)

        self.rot_subscription = self.create_subscription(
            Float64,
            'airmar_data/rate_of_turn',
            self.rate_of_turn_callback,
            10)
        
        self.navsat_subscription = self.create_subscription(
            NavSatFix,
            'airmar_data/lat_long',
            self.lat_long_callback,
            10)
        
        self.track_degrees_true_subscription = self.create_subscription(
            Float64,
            'airmar_data/track_degrees_true',
            self.track_degrees_true_callback,
            10)
        
        self.track_degrees_magnetic_subscription = self.create_subscription(
            Float64,
            'airmar_data/track_degrees_magnetic',
            self.track_degrees_magnetic_callback,
            10)
        
        self.speed_knots_subscription = self.create_subscription(
            Float64,
            'airmar_data/speed_knots',
            self.speed_knots_callback,
            10)
        
        self.speed_kmh_subscription = self.create_subscription(
            Float64,
            'airmar_data/speed_kmh',
            self.speed_kmh_callback,
            10)
        
        self.heading_subscription = self.create_subscription(
            Float64,
            'airmar_data/heading',
            self.heading_callback,
            10)
        
        self.true_wind_subscription = self.create_subscription(
            Wind,
            'airmar_data/true_wind',
            self.true_wind_callback,
            10)
        
        self.apparent_wind_subscription = self.create_subscription(
            Wind,
            'airmar_data/apparent_wind',
            self.apparent_wind_callback,
            10)
        
        self.roll_subscription = self.create_subscription(
            Float64,
            'airmar_data/roll',
            self.roll_callback,
            10)
        
        self.pitch_subscription = self.create_subscription(
            Float64,
            'airmar_data/pitch',
            self.pitch_callback,
            10)

        self.airmar_reader_lifecycle_subscription = self.create_subscription(
            TransitionEvent,
            '/airmar_reader/transition_event',
            self.airmar_lifecycle_callback,
            10)

        self.ballast_control_lifecycle_subscription = self.create_subscription(
            TransitionEvent,
            '/ballast_control/transition_event',
            self.ballast_lifecycle_callback,
            10)
        
        self.pwm_controller_lifecycle_subscription = self.create_subscription(
            TransitionEvent,
            '/pwm_controller/transition_event',
            self.pwm_lifecycle_callback,
            10)
        
        self.tt_lifecycle_subscription = self.create_subscription(
            TransitionEvent,
            '/trim_tab_comms/transition_event',
            self.tt_lifecycle_callback,
            10)
        
        self.restart_node_client = self.create_client(RestartNode, 'state_manager/restart_node', callback_group=self.callback_group_state)
        #initial dummy values, for testing
        self.current_boat_state.latitude = 5
        self.current_boat_state.longitude = 4
        self.current_boat_state.current_heading = 12
        self.current_boat_state.track_degrees_true = 0
        self.current_boat_state.track_degrees_magnetic = 0
        self.current_boat_state.speed_knots = 12
        self.current_boat_state.speed_kmh = 12
        self.current_boat_state.rate_of_turn = 2
        self.current_boat_state.true_wind.speed = 12
        self.current_boat_state.true_wind.direction = 5
        self.current_boat_state.apparent_wind.speed = 4
        self.current_boat_state.apparent_wind.direction = 6
        self.current_boat_state.pitch = 3
        self.current_boat_state.roll = 2
        self.node_indices = {}
        node_names = ["airmar_reader", "ballast_control", "battery_monitor", "control_system", "network_comms", "pwm_controller", "trim_tab_comms"]
        i=0
        for name in node_names:
            node_info = boat_state_pb2.NodeInfo()
            node_info.name = name
            node_info.status = boat_state_pb2.NodeStatus.NODE_STATUS_OK
            node_info.info = ""
            self.current_boat_state.node_states.append(node_info)
            self.node_indices[name]=i
            i+=1
        self.current_boat_state.current_autonomous_mode = boat_state_pb2.AutonomousMode.AUTONOMOUS_MODE_NONE
        a = boat_state_pb2.Point()
        a.latitude = 5.1
        a.longitude = 4.1
        b=boat_state_pb2.Point()
        b.latitude = 5.2
        b.longitude = 4.1
        self.current_boat_state.current_path.points.append(a)
        self.current_boat_state.current_path.points.append(b)
        c = boat_state_pb2.Point()
        c.latitude = 4.9
        c.longitude = 3.9
        d=boat_state_pb2.Point()
        d.latitude = 4.8
        d.longitude = 3.9
        e=boat_state_pb2.Point()
        e.latitude = 4.7
        e.longitude = 3.8
        self.current_boat_state.previous_positions.points.append(c)
        self.current_boat_state.previous_positions.points.append(d)
        self.current_boat_state.previous_positions.points.append(e)
        
        self.last_pwm_heartbeat = -1
        self.last_ctrl_heartbeat = -1
        self.last_tt_heartbeat = -1
        self.create_grpc_server()

        self.pwm_heartbeat_subscription = self.create_subscription(
            Empty,
            'heartbeat/pwm_controler',
            self.pwm_controller_heartbeat,
            1)
        
        self.control_system_heartbeat_subscription = self.create_subscription(
            Empty,
            'heartbeat/control_system',
            self.control_system_heartbeat,
            1)
        
        self.trim_tab_heartbeat_subscription = self.create_subscription(
            Empty,
            'heartbeat/trim_tab_comms',
            self.trim_tab_comms_heartbeat,
            1)
        #super().on_configure()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")
        # Start publishers or timers
        #self.ballast_position_publisher.on_activate()
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")
        super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")
        # Destroy subscribers, publishers, and timers
        self.destroy_lifecycle_publisher(self.pwm_control_publisher)
        self.destroy_lifecycle_publisher(self.ballast_position_publisher)
        self.destroy_lifecycle_publisher(self.trim_tab_control_publisher)
        self.destroy_lifecycle_publisher(self.trim_tab_angle_publisher)

        self.destroy_subscription(self.rot_subscription)
        self.destroy_subscription(self.navsat_subscription)
        self.destroy_subscription(self.track_degrees_true_subscription)
        self.destroy_subscription(self.track_degrees_magnetic_subscription)
        self.destroy_subscription(self.speed_knots_subscription)
        self.destroy_subscription(self.speed_kmh_subscription)
        self.destroy_subscription(self.heading_subscription)
        self.destroy_subscription(self.true_wind_subscription)
        self.destroy_subscription(self.apparent_wind_subscription)
        self.destroy_subscription(self.roll_subscription)
        self.destroy_subscription(self.pitch_subscription)
        self.destroy_subscription(self.pwm_heartbeat_subscription)
        self.destroy_subscription(self.control_system_heartbeat_subscription)
        self.destroy_subscription(self.trim_tab_heartbeat_subscription)

        #return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        # Perform final cleanup if necessary
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Error caught!")
        return super().on_error(state)
     
    #end lifecycle callbacks

    def restart_lifecycle_node_by_name(self, node_name):
        # Create a service client to send lifecycle state change requests
        client = self.create_client(ChangeState, f'{node_name}/change_state')

        # Wait for the service to be available
        i=0
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {node_name} lifecycle service...')
            i+=1
            if i > 5:
                return False

        # Deactivate the node
        deactivate_request = ChangeState.Request()
        deactivate_request.transition.id = Transition.TRANSITION_DEACTIVATE
        client.call_async(deactivate_request)
        self.get_logger().info(f'Sent deactivate request to {node_name}')

        # Reactivate the node
        activate_request = ChangeState.Request()
        activate_request.transition.id = Transition.TRANSITION_ACTIVATE
        client.call_async(activate_request)
        self.get_logger().info(f'Sent activate request to {node_name}')
        return True

    def airmar_lifecycle_callback(self, msg: TransitionEvent):
        self.get_logger().info("Received airmar update!")
        self.current_boat_state.node_states[self.node_indices["airmar_reader"]].lifecycle_state = get_state(msg.goal_state.id)

    def ballast_lifecycle_callback(self, msg: TransitionEvent):
        self.get_logger().info("Received ballast update!")
        self.current_boat_state.node_states[self.node_indices["ballast_control"]].lifecycle_state = get_state(msg.goal_state.id)
    
    def pwm_lifecycle_callback(self, msg: TransitionEvent):
        self.get_logger().info("Received pwm update!")
        self.current_boat_state.node_states[self.node_indices["pwm_controller"]].lifecycle_state = get_state(msg.goal_state.id)
    
    def tt_lifecycle_callback(self, msg: TransitionEvent):
        self.get_logger().info("Received trimtab update!")
        self.current_boat_state.node_states[self.node_indices["trim_tab_comms"]].lifecycle_state = get_state(msg.goal_state.id)

    def rate_of_turn_callback(self, msg: Float64):
        self.current_boat_state.rate_of_turn = msg.data

    def lat_long_callback(self, msg: NavSatFix):
        self.current_boat_state.latitude = msg.latitude
        self.current_boat_state.longitude = msg.longitude

    def track_degrees_true_callback(self, msg: Float64):
        self.current_boat_state.track_degrees_true = msg.data

    def track_degrees_magnetic_callback(self, msg: Float64):
        self.current_boat_state.track_degrees_magnetic = msg.data

    def speed_knots_callback(self, msg: Float64):
        self.current_boat_state.speed_knots = msg.data

    def speed_kmh_callback(self, msg: Float64):
        self.current_boat_state.speed_kmh = msg.data
    
    def heading_callback(self, msg: Float64):
        self.current_boat_state.current_heading = msg.data

    def true_wind_callback(self, msg: Wind):
        self.current_boat_state.true_wind.speed = msg.speed.data
        self.current_boat_state.true_wind.direction = msg.direction.data

    def apparent_wind_callback(self, msg: Wind):
        self.current_boat_state.apparent_wind.speed = msg.speed.data
        self.current_boat_state.apparent_wind.direction = msg.direction.data

    def roll_callback(self, msg: Float64):
        self.current_boat_state.roll = msg.data
    
    def pitch_callback(self, msg: Float64):
        self.current_boat_state.pitch = msg.data

    #new server code
    def create_grpc_server(self): 
        self.get_logger().info("Creating gRPC server")
        self.grpc_server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        control_pb2_grpc.add_ExecuteRudderCommandServiceServicer_to_server(self, self.grpc_server)
        control_pb2_grpc.add_ExecuteTrimTabCommandServiceServicer_to_server(self, self.grpc_server)
        control_pb2_grpc.add_ExecuteBallastCommandServiceServicer_to_server(self, self.grpc_server)
        control_pb2_grpc.add_ExecuteAutonomousModeCommandServiceServicer_to_server(self, self.grpc_server)
        control_pb2_grpc.add_ExecuteSetPathCommandServiceServicer_to_server(self, self.grpc_server)
        boat_state_pb2_rpc.add_SendBoatStateServiceServicer_to_server(self, self.grpc_server)
        node_restart_pb2_grpc.add_RestartNodeServiceServicer_to_server(self, self.grpc_server)

        #connect_pb2_grpc.add_ConnectToBoatServiceServicer_to_server(self, self.grpc_server)
        self.grpc_server.add_insecure_port('[::]:50051')
        self.grpc_server.start()

    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteRudderCommand(self, command: control_pb2.RudderCommand, context):
        #center = 75 degrees, full right=40, full left = 113
        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_ERROR
        #rudder commands are radians, map to degrees
        degrees = int(command.rudder_control_value*(180/math.pi))
        degrees_scaled =  (((degrees - -90) * (113 - 40)) / (90 - -90)) + 40
        #self.get_logger().info("input angle: {}", degrees)
        rudder_json = {"channel": "8", "angle": degrees_scaled}
        #self.get_logger().info("Publishing rudder command: "+str(degrees))
        #self.get_logger().info("Publishing rudder command")

        self.pwm_control_publisher.publish(make_json_string(rudder_json))
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteTrimTabCommand(self, command: control_pb2.TrimTabCommand, context):
        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_ERROR
        state_msg = Int8()
        state_msg.data = 5
        angle_msg = Int16()
        angle_msg.data = int((command.trimtab_control_value+math.pi/2)*180/math.pi)
        self.trim_tab_control_publisher.publish(state_msg)
        self.trim_tab_angle_publisher.publish(angle_msg)
        #self.get_logger().info("Publishing trimtab command")
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteBallastCommand(self, command: control_pb2.BallastCommand, context):
        response = control_pb2.ControlResponse()
        self.get_logger().info("Publishing ballast command")
        position_msg = Float64()
        position_msg.data = command.ballast_control_value
        self.ballast_position_publisher.publish(position_msg)

        # current_target = 80 + ((110 - 80) / (1.0 - -1.0)) * (command.ballast_control_value - -1.0)
        # ballast_json = {"channel": "12", "angle": current_target}
        # self.pwm_control_publisher.publish(make_json_string(ballast_json))

        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_ERROR
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteAutonomousModeCommand(self, command: control_pb2.AutonomousModeCommand, context):
        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_ERROR
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteSetPathCommand(self, command: control_pb2.SetPathCommand, context):
        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_ERROR
        self.current_boat_state.current_path.ClearField("points")# = command.new_path
        self.current_boat_state.current_path.points.extend(command.new_path.points)
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def SendBoatState(self, command: boat_state_pb2.BoatStateRequest(), context):
        return self.current_boat_state
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def RestartNode(self, command: node_restart_pb2.RestartNodeRequest(), context):
        self.get_logger().info("Received restart command for: "+command.node_name)
        # restart_node_request = RestartNode.Request()
        # restart_node_request.node_name = command.node_name
        # if(self.restart_node_client.wait_for_service(3) is False):
        #     self.get_logger().error("Client service not available for state manager!")
        #     response.success = False
        #     return response
        # future = self.restart_node_client.call_async(restart_node_request)
        # self.get_logger().info("About to spin")
        # rclpy.spin_until_future_complete(self, future)
        # self.get_logger().info("completed command")
        result = self.restart_lifecycle_node_by_name(command.node_name)
        response = node_restart_pb2.RestartNodeResponse()
        response.success = result
        self.get_logger().info("Restart node: "+str(result))
        return response
    
    def pwm_controller_heartbeat(self, message):
        self.get_logger().info("Got pwm heartbeat")
        self.last_pwm_heartbeat = time.time()

    def control_system_heartbeat(self, message):
        self.get_logger().info("Got control heartbeat")
        self.last_ctrl_heartbeat = time.time()

    def trim_tab_comms_heartbeat(self, message):
        #self.get_logger().info("Got trimtab heartbeat")
        self.last_tt_heartbeat = time.time()
    
    def update_node_status_timer_callback(self):
        current_time = time.time()
        if(current_time-self.last_pwm_heartbeat>1):
            self.current_boat_state.node_states[self.node_indices["pwm_controller"]].node_status = boat_state_pb2.NodeStatus.NODE_STATUS_ERROR
        else:
            self.current_boat_state.node_states[self.node_indices["pwm_controller"]].node_status = boat_state_pb2.NodeStatus.NODE_STATUS_OK
        
        if(current_time-self.last_ctrl_heartbeat>1):
            self.current_boat_state.node_states[self.node_indices["control_system"]].node_status = boat_state_pb2.NodeStatus.NODE_STATUS_ERROR
        else:
            self.current_boat_state.node_states[self.node_indices["control_system"]].node_status = boat_state_pb2.NodeStatus.NODE_STATUS_OK


def main(args=None):
    rclpy.init(args=args)
    network_comms = NetworkComms()

    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(network_comms)

    try:
        # Spin the node to execute callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        network_comms.get_logger().fatal(f'Unhandled exception: {e.with_traceback(e.__traceback__)}')
    finally:
        # Shutdown and cleanup the node
        executor.shutdown()
        network_comms.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()