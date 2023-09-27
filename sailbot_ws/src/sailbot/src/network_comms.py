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
        print("Server is listening for incoming connections...")

    def register_clients(self):
        read_sockets, write_sockets, error_sockets = select.select([self.server_socket] , [], [])
        for sock in read_sockets:
            client_socket, client_address = self.server_socket.accept()
            self.client_addresses.add(client_address[0])
            print(f"Accepted connection from {client_address}")
            # Data to be sent to the client
            data_to_send = {"registered"}
            #  Serialize and send the data
            data_bytes = pickle.dumps(data_to_send)
            client_socket.send(data_bytes)
            # Close the client socket
            client_socket.close()

    def update_clients(self):
        for address in self.client_addresses:
            self.get_logger().info(address)
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((address, 1111))
            data_bytes = pickle.dumps(self.current_boat_state)
            client_socket.send(data_bytes)
            client_socket.close()
        

    def register_and_update_clients(self):
        self.register_clients()
        self.update_clients()

def main(args=None):
    rclpy.init(args=args)

    network_comms = NetworkComms()
    
    thread = threading.Thread(target=rclpy.spin, args=(network_comms, ), daemon=True)
    thread.start()
        
    rate = network_comms.create_rate(4)
    try:
        while rclpy.ok():
            network_comms.register_and_update_clients()
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()

if __name__ == "__main__":
    main()