import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from sailbot_msgs.msg import GeoPath
from random import gauss, uniform
import time

class FakeMovement(Node):
    def __init__(self):
        super().__init__('fake_movement')
        self.subscription = self.create_subscription(
            GeoPath,
            'current_path',
            self.path_callback,
            10)
        self.publisher = self.create_publisher(NavSatFix, 'airmar_data/lat_long', 10)
        self.points = []
        self.current_index = 0
        self.timer = self.create_timer(0.03, self.publish_position)
        self.progress = 0.0  # Progress along the current segment
        self.increment = 0.01  # Incremental progress per timer call

    def path_callback(self, msg):
        self.points = msg.points
        self.get_logger().info('Received new path points.')
        self.current_index = 0
        self.progress = 0.0
        self.lat_error = 0.0
        self.lon_error = 0.0
        self.error_decay = 0.95  # Determines how quickly the error converges back to zero
        self.error_magnitude = 0.00001  # Magnitude of maximum deviation per step

    def publish_position(self):
        if not self.points or self.current_index >= len(self.points) - 1:
            return
        
        # Calculate next position
        start_point = self.points[self.current_index]
        end_point = self.points[self.current_index + 1]
        lat = (1 - self.progress) * start_point.latitude + self.progress * end_point.latitude
        lon = (1 - self.progress) * start_point.longitude + self.progress * end_point.longitude

        # Update and apply the walking error
        self.lat_error = self.lat_error * self.error_decay + uniform(-self.error_magnitude, self.error_magnitude)
        self.lon_error = self.lon_error * self.error_decay + uniform(-self.error_magnitude, self.error_magnitude)
        lat += self.lat_error
        lon += self.lon_error

        # Publish with additional noise
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps'
        fix.latitude = lat + gauss(0, 0.000005)
        fix.longitude = lon + gauss(0, 0.000005)
        fix.altitude = gauss(0, 0.1)
        self.publisher.publish(fix)

        # Update progress
        self.progress += self.increment
        if self.progress >= 1.0:
            self.progress = 0.0
            self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    fake_movement = FakeMovement()
    rclpy.spin(fake_movement)
    fake_movement.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()