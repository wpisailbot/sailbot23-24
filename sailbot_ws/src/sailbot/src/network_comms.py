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



class NetworkComms(Node):

    current_boat_state = boat_state_pb2.BoatState()
    client_sockets = {}
    # Create a socket server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    control_message_byte_size = 0

    def __init__(self):
        super().__init__('control_system')
        #self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        #self.server_socket.bind(("0.0.0.0", 1111))  # Bind to a specific address and port        self.server_socket.listen(1)  # Listen for incoming connections
        #self.server_socket.listen(1)  # Listen for incoming connections
        #self.get_logger().info("Server is listening for incoming connections...")
        #self.create_timer(0.25, self.update_clients)
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
            node_info.status = boat_state_pb2.NodeStatus.WARN
            node_info.info = ""
            self.current_boat_state.node_states.append(node_info)
        
        control_command = control_pb2.ControlCommand()
        control_command.control_type = control_pb2.ControlType.RUDDER
        control_command.control_value = 1.0
        self.control_message_byte_size = control_command.ByteSize()
        self.get_logger().info("control command is "+str(self.control_message_byte_size)+" bytes")
        self.get_logger().info("boat state is "+str(self.current_boat_state.ByteSize())+" bytes")

    #new server code
    def create_grpc_server(self):
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        control_pb2_grpc.add_ExecuteControlCommandServiceServicer_to_server(self, self.server)
        self.server.add_insecure_port('[::]:50051')
        self.server.start()
        self.server.wait_for_termination()

    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteControlCommand(self, command: control_pb2.ControlCommand(), context):
        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_ERROR
        return response

    
    #old server code
    def register_clients(self):
        read_sockets, write_sockets, error_sockets = select.select([self.server_socket] , [], [], 0.2)
        for sock in read_sockets:
            client_socket, client_address = self.server_socket.accept()
            self.client_sockets[client_address[0]]=client_socket
            self.get_logger().info(f"Accepted connection from {client_address}")


    def update_clients(self):
        #self.get_logger().info(f"Num clients: {len(self.client_sockets)}")
        data_bytes = self.current_boat_state.SerializeToString()
        #self.get_logger().info("data size: "+str(len(data_bytes)))
        #self.get_logger().info(str(self.current_boat_state))
        #data_bytes = bytes("Hello", "utf-8")
        del_list = []
        for host in self.client_sockets.keys():
            try:
                self.get_logger().info(f"Sending data")
                self.client_sockets[host].send(data_bytes)
            except:
                self.get_logger().info(f"Lost connection to client: {str(host)}")
                del_list.append(host)
        for host in del_list:
            del self.client_sockets[host]
    
    def receive_commands(self):
        while True:
            rlist, _, _ = select.select(self.client_sockets.values(), [], [], 0.2)
            for sock in rlist:
                try:
                    self.get_logger().info("Trying to receive")
                    received_bytes = sock.recv(1024)
                    # if(len(received_bytes)==0):
                    #     continue #no data
                    received_data = control_pb2.ControlCommand()
                    received_data.ParseFromString(received_bytes)
                    self.get_logger().info("Received command")
                    #received_data = pickle.loads(received_bytes)
                except Exception as e:
                    self.get_logger().info(e)
                    continue #connection lost, should be re-established by main thread
                match received_data:
                    case control_pb2.ControlCommand():
                        match received_data.control_type:
                            case control_pb2.ControlType.TRIM_TAB:
                                self.get_logger().info("Received trimtab control command: "+str(received_data.control_value))
                            case control_pb2.ControlType.RUDDER:
                                self.get_logger().info("Received rudder control command: "+str(received_data.control_value))
                            case _:
                                self.get_logger().info("Unknown control command received")
                    case _:
                        self.get_logger().info("Unknown command received")
                pass
        
    def register_clients_loop(self):
        while True:
            self.register_clients()

def main(args=None):
    rclpy.init(args=args)
    network_comms = NetworkComms()
    #receive_thread = threading.Thread(target=network_comms.receive_commands, daemon=True)
    #receive_thread.start()
    rclpy.spin(network_comms)
    rclpy.shutdown()

if __name__ == "__main__":
    main()