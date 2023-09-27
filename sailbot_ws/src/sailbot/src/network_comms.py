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
    client_addresses = set()
    # Create a socket server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __init__(self):
        super().__init__('control_system')
        self.server_socket.bind(("0.0.0.0", 1111))  # Bind to a specific address and port
        self.server_socket.listen(1)  # Listen for incoming connections
        self.get_logger().info("Server is listening for incoming connections...")
        self.create_timer(0.25, self.update_clients)
        client_registration_thread = threading.Thread(target=self.register_clients_loop, daemon=True)
        client_registration_thread.start()

    def register_clients(self):
        read_sockets, write_sockets, error_sockets = select.select([self.server_socket] , [], [])
        for sock in read_sockets:
            client_socket, client_address = self.server_socket.accept()
            self.client_addresses.add(client_address[0])
            self.get_logger().info(f"Accepted connection from {client_address}")
            # Data to be sent to the client
            data_to_send = {"registered"}
            #  Serialize and send the data
            data_bytes = pickle.dumps(data_to_send)
            client_socket.send(data_bytes)
            # Close the client socket
            client_socket.close()

    def update_single_client(self, address, data):
        self.get_logger().info(address)
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.setblocking(0)
        client_socket.settimeout(0.1)
        try:
            client_socket.connect((address, 1111))
            client_socket.send(data)
            client_socket.close()
        except:
             self.get_logger().info(f"Lost connection to client: {address}")

    def update_clients(self):
        self.get_logger().info(f"Num clients: {len(self.client_addresses)}")
        data_bytes = pickle.dumps(self.current_boat_state)
        for address in self.client_addresses:
            update_client_thread = threading.Thread(target=self.update_single_client, args=(address, data_bytes))
            update_client_thread.start()
        
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