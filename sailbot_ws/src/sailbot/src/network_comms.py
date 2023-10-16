#!/usr/bin/env python3
import pickle
import socket
import select
from time import sleep
import rclpy
from rclpy.node import Node
import threading
from telemetry_messages.messages import *


class NetworkComms(Node):

    current_boat_state = BoatState()
    client_sockets = {}
    # Create a socket server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __init__(self):
        super().__init__('control_system')
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.server_socket.bind(("0.0.0.0", 1111))  # Bind to a specific address and port        self.server_socket.listen(1)  # Listen for incoming connections
        self.server_socket.listen(1)  # Listen for incoming connections
        self.get_logger().info("Server is listening for incoming connections...")
        self.create_timer(0.25, self.update_clients)
        client_registration_thread = threading.Thread(target=self.register_clients_loop, daemon=True)
        client_registration_thread.start()

    def register_clients(self):
        read_sockets, write_sockets, error_sockets = select.select([self.server_socket] , [], [], 0.2)
        for sock in read_sockets:
            client_socket, client_address = self.server_socket.accept()
            self.client_sockets[client_address[0]]=client_socket
            self.get_logger().info(f"Accepted connection from {client_address}")


    def update_clients(self):
        #self.get_logger().info(f"Num clients: {len(self.client_sockets)}")
        data_bytes = pickle.dumps(self.current_boat_state)

        for host in self.client_sockets.keys():
            try:
                self.client_sockets[host].send(data_bytes)
            except:
                self.get_logger().info(f"Lost connection to client: {str(host)}")
    
    def receive_commands(self):
        while True:
            rlist, _, _ = select.select(self.client_sockets.values(), [], [], 0.2)
            for sock in rlist:
                try:
                    received_bytes = sock.recv(1024)
                    received_data = pickle.loads(received_bytes)
                except:
                    continue #connection lost, should be re-established by main thread
                match received_data:
                    case ControlCommand():
                        match received_data.control_type:
                            case ControlType.TRIM_TAB:
                                self.get_logger().info("Received trimtab control command: "+str(received_data.control_value))
                            case ControlType.RUDDER:
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
    receive_thread = threading.Thread(target=network_comms.receive_commands, daemon=True)
    receive_thread.start()
    rclpy.spin(network_comms)
    rclpy.shutdown()

if __name__ == "__main__":
    main()