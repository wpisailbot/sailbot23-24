#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String, Float64, Int16, Header
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose, Quaternion
from sailbot_msgs.msg import Path
from typing import Optional
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.subscription import Subscription
from ament_index_python.packages import get_package_share_directory
import os
import cv2
import re
import numpy as np

from geopy.distance import great_circle

def get_resource_dir():
    package_path = get_package_share_directory('sailbot')
    resource_path = os.path.join(package_path, 'maps')
    return resource_path

def calculate_look_ahead_distance(base_distance, speed_factor, current_speed):
    return base_distance + (speed_factor * current_speed)

def find_look_ahead_point(path, current_position, current_speed):
    base_distance = 5  # Minimum look-ahead distance in meters
    speed_factor = 1   # How much the look-ahead distance increases per knot of speed
    
    look_ahead_distance = calculate_look_ahead_distance(base_distance, speed_factor, current_speed)
    
    look_ahead_point = None
    for i, point in enumerate(path):
        distance = great_circle(current_position, (point.latitude, point.longitude)).meters
        if distance < closest_distance:
            closest_distance = distance
            closest_index = i
    
    # Starting from the closest point, find the look ahead point
    for point in path[closest_index:]:
        distance = great_circle(current_position, (point.latitude, point.longitude)).meters
        if distance >= look_ahead_distance:
            look_ahead_point = point
            break
    
    return look_ahead_point

def find_and_load_image(directory, location):
    """
    Find an image by location and load it along with its bounding box coordinates.

    Parameters:
    - directory: The directory to search in.
    - location: The location to match.

    Returns:
    - A tuple containing the loaded image and a dictionary with the bounding box coordinates.
    """
    # Regular expression to match the filename format and capture coordinates
    pattern = re.compile(rf"{location}:(-?\d+\.?\d*):(-?\d+\.?\d*):(-?\d+\.?\d*):(-?\d+\.?\d*)\.png")

    for filename in os.listdir(directory):
        match = pattern.match(filename)
        if match:
            # Extract the bounding box coordinates
            south, west, north, east = map(float, match.groups())

            # Load the image
            image_path = os.path.join(directory, filename)
            image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError(f"Unable to load image at {image_path}")

            return image, {"south": south, "west": west, "north": north, "east": east}

    # Return None if no matching file is found
    return None, None

class PathFollower(LifecycleNode):
    heading = 0
    latitude = 0
    longitude = 0
    speed_knots = 0
    current_path = Path()

    def __init__(self):
        super().__init__('path_follower')
        self.target_position_publisher: Optional[Publisher]
        self.airmar_heading_subscription: Optional[Subscription]
        self.airmar_position_subscription: Optional[Subscription]
        self.airmar_speed_knots_subscription: Optional[Subscription]
        self.timer: Optional[Timer]

        self.declare_parameter('map_name')
        map_name = self.get_parameter('map_name').get_parameter_value().string_value
        self.get_logger().info(f'Map name: {map_name}')
        image, bbox = find_and_load_image(get_resource_dir(), "quinsigamond")
        occupancy_grid_values = np.clip(image, 0, 255)
        occupancy_grid_values = ((255 - occupancy_grid_values) * 100 / 255).astype(np.int8)
        grid_msg = OccupancyGrid()
        grid_msg.header = Header(frame_id="map")
        grid_msg.info.resolution = 0.0001
        grid_msg.info.width = occupancy_grid_values.shape[1]
        grid_msg.info.height = occupancy_grid_values.shape[0]
        grid_msg.info.origin = Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

        grid_msg.data = occupancy_grid_values.ravel().tolist()

        self.get_logger().info("Map setup done")


    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")

        self.target_position_publisher = self.create_lifecycle_publisher(NavSatFix, 'target_position', 10)

        self.airmar_heading_subscription = self.create_subscription(
            Float64,
            '/airmar_data/heading',
            self.airmar_heading_callback,
            10)
        self.airmar_position_subscription = self.create_subscription(
            NavSatFix,
            '/airmar_data/lat_long',
            self.airmar_heading_callback,
            10)
        self.airmar_speed_knots_subscription = self.create_subscription(
            Float64,
            '/airmar_data/speed_knots',
            self.airmar_speed_knots_callback,
            10)
        self.waypoints_subscriber = self.node.create_subscription(
            Path, 'waypoints', self.waypoints_callback, 10)
        #self.timer = self.create_timer(0.1, self.control_loop_callback)
        self.get_logger().info("Ballast node configured")
        #super().on_configure(state)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")
        # Start publishers or timers
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")
        super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")
        # Destroy subscribers, publishers, and timers
        self.destroy_timer(self.timer)
        self.destroy_lifecycle_publisher(self.target_position_publisher)
        self.destroy_subscription(self.airmar_heading_subscription)
        self.destroy_subscription(self.airmar_position_subscription)

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        # Perform final cleanup if necessary
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Error caught!")
        return super().on_error(state)
    
    #end callbacks

    def airmar_heading_callback(self, msg: Float64):
        self.heading = msg.data
    
    def airmar_position_callback(self, msg: NavSatFix):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.find_look_ahead()


    def airmar_speed_knots_callback(self, msg: Float64):
        self.speed_knots = msg.data
        self.find_look_ahead()

    
    def waypoints_callback(self, msg: Path):
        self.path = msg

        self.find_look_ahead()
    
    def find_look_ahead(self):
        look_ahead_point = find_look_ahead_point(self.path.points, (self.latitude, self.longitude), self.speed_knots)
        self.get_logger().info(f"Calulated lookAhead point: {look_ahead_point.latitude}, {look_ahead_point.longitude}")
        self.target_position_publisher.publish(look_ahead_point)



def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()

    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(path_follower)

    try:
        # Spin the node to execute callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        path_follower.get_logger().fatal(f'Unhandled exception: {e}')
    finally:
        # Shutdown and cleanup the node
        executor.shutdown()
        path_follower.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()