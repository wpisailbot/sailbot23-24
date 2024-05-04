import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from sailbot_msgs.msg import GeoPath
from random import gauss
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

    def publish_position(self):
        if not self.points or self.current_index >= len(self.points) - 1:
            return
        
        # Get current and next point
        start_point = self.points[self.current_index]
        end_point = self.points[self.current_index + 1]

        # Linear interpolation
        lat = (1 - self.progress) * start_point.latitude + self.progress * end_point.latitude
        lon = (1 - self.progress) * start_point.longitude + self.progress * end_point.longitude
        
        # Publish interpolated position with noise
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps'
        fix.latitude = lat + gauss(0, 0.00003)
        fix.longitude = lon + gauss(0, 0.00003)
        fix.altitude = gauss(0, 0.1)
        self.publisher.publish(fix)

        # Update progress and check if it's time to move to the next segment
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