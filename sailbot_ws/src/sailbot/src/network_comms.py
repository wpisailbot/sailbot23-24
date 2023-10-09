#!/usr/bin/env python3
import pickle
import socket
import select
from time import sleep
import rclpy
from rclpy.node import Node
import threading


class Wind:
    speed = 0
    direction = 0

    def __init__(self, speed, direction):
        self.speed = speed
        self.direction = direction


class BoatState:
    latitude = 0
    latitude_direction = ""
    longitude = 0
    longitude_direction = ""
    current_heading = 0
    magnetic_deviation = 0
    magnetic_deviation_direction = ""
    magnetic_variation = 0
    magnetic_variation_direction = 0
    track_degrees_true = 0
    track_degrees_magnetic = 0
    speed_knots = 0
    speed_kmh = 0
    rate_of_turn = 0
    outside_temp = 0
    atmospheric_pressure = 0
    true_wind = Wind(0, 0)
    apparent_wind = Wind(0, 0)
    pitch = 0
    roll = 0


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
        self.get_logger().info(f"Num clients: {len(self.client_sockets)}")
        data_bytes = pickle.dumps(self.current_boat_state)

        for host in self.client_sockets.keys():
            try:
                self.client_sockets[host].send(data_bytes)
            except:
                self.get_logger().info(f"Lost connection to client: {str(host)}")
        
    def register_clients_loop(self):
        while True:
            self.register_clients()

def main(args=None):
    rclpy.init(args=args)
    network_comms = NetworkComms()
    rclpy.spin(network_comms)
    rclpy.shutdown()

if __name__ == "__main__":
    main()