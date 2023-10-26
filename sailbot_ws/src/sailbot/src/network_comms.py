#!/usr/bin/env python3
import socket
import select
from time import sleep
import rclpy
from rclpy.node import Node
import grpc
from concurrent import futures

import telemetry_messages.python.boat_state_pb2 as boat_state_pb2
import telemetry_messages.python.boat_state_pb2_grpc as boat_state_pb2_rpc
import telemetry_messages.python.control_pb2 as control_pb2
import telemetry_messages.python.control_pb2_grpc as control_pb2_grpc
import telemetry_messages.python.node_restart_pb2 as node_restart_pb2
import telemetry_messages.python.node_restart_pb2_grpc as node_restart_pb2_grpc



class NetworkComms(Node):

    current_boat_state = boat_state_pb2.BoatState()

    def __init__(self):
        super().__init__('control_system')
        
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
        node_names = ["airmar_reader", "battery_monitor", "control_system", "debug_interface", "network_comms", "pwm_controller", "serial_rc_receiver", "trim_tab_comms"]
        for name in node_names:
            node_info = boat_state_pb2.NodeInfo()
            node_info.name = name
            node_info.status = boat_state_pb2.NodeStatus.NODE_STATUS_WARN
            node_info.info = ""
            self.current_boat_state.node_states.append(node_info)
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

        self.create_grpc_server()

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
        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_ERROR
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteTrimTabCommand(self, command: control_pb2.TrimTabCommand, context):
        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_ERROR
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteBallastCommand(self, command: control_pb2.BallastCommand, context):
        response = control_pb2.ControlResponse()
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
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def SendBoatState(self, command: boat_state_pb2.BoatStateRequest(), context):
        return self.current_boat_state
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def RestartNode(self, command: node_restart_pb2.RestartNodeRequest(), context):
        response = node_restart_pb2.RestartNodeResponse()
        response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    network_comms = NetworkComms()
    rclpy.spin(network_comms)
    rclpy.shutdown()

if __name__ == "__main__":
    main()