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
    client_stubs = {}
    # Create a socket server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    control_message_byte_size = 0

    def __init__(self):
        super().__init__('control_system')
        #self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        #self.server_socket.bind(("0.0.0.0", 1111))  # Bind to a specific address and port        self.server_socket.listen(1)  # Listen for incoming connections
        #self.server_socket.listen(1)  # Listen for incoming connections
        #self.get_logger().info("Server is listening for incoming connections...")
        self.create_timer(0.25, self.update_clients)
        #client_registration_thread = threading.Thread(target=self.register_clients_loop, daemon=True)
        #client_registration_thread.start()
        
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
        control_pb2_grpc.add_ExecuteControlCommandServiceServicer_to_server(self, self.grpc_server)
        boat_state_pb2_rpc.add_SendBoatStateServiceServicer_to_server(self, self.grpc_server)
        node_restart_pb2_grpc.add_RestartNodeServiceServicer_to_server(self, self.grpc_server)

        #connect_pb2_grpc.add_ConnectToBoatServiceServicer_to_server(self, self.grpc_server)
        self.grpc_server.add_insecure_port('[::]:50051')
        self.grpc_server.start()

    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteControlCommand(self, command: control_pb2.ControlCommand, context):
        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_ERROR
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def SendBoatState(self, command: boat_state_pb2.BoatStateRequest(), context):
        return self.current_boat_state
    
    def RestartNode(self, command: node_restart_pb2.RestartNodeRequest(), context):
        response = node_restart_pb2.RestartNodeResponse()
        response.success = False
        return response


    def update_clients(self):
        del_list = []
        for host in self.client_stubs.keys():
            try:
                self.get_logger().info(f"Sending boat state data")
                self.client_stubs[host].ReceiveBoatState(self.current_boat_state)
                self.get_logger().info(f"Sent data")
            except:
                self.get_logger().info(f"Lost connection to client: {str(host)}")
                del_list.append(host)
        for host in del_list:
            del self.client_stubs[host]

def main(args=None):
    rclpy.init(args=args)
    network_comms = NetworkComms()
    rclpy.spin(network_comms)
    rclpy.shutdown()

if __name__ == "__main__":
    main()