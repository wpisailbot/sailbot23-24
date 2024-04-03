#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String, Float64, Int16, Header
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose, Quaternion
from geographic_msgs.msg import GeoPoint
from sailbot_msgs.msg import Path
from sailbot_msgs.srv import SetMap, GetPath
from typing import Optional
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.subscription import Subscription
from rclpy.node import ParameterType
from ament_index_python.packages import get_package_share_directory
import os
import cv2
import re
import numpy as np
from pyproj import Transformer

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
    closest_distance = float('inf')
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
            image = 255-cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError(f"Unable to load image at {image_path}")

            return image, {"south": south, "west": west, "north": north, "east": east}

    # Return None if no matching file is found
    return None, None

def grid_to_latlong_proj(x, y, bbox, image_width, image_height, src_proj='EPSG:4326', dest_proj='EPSG:3857'):
    """
    Convert grid cell coordinates in an image to latitude/longitude coordinates.

    Parameters:
    - x, y: The pixel positions in the image.
    - bbox: A dictionary with keys 'north', 'south', 'east', 'west' representing the bounding box.
    - image_width, image_height: The dimensions of the image in pixels.
    - src_proj: Source projection (latitude/longitude).
    - dest_proj: Destination projection for the image.

    Returns:
    - A tuple (latitude, longitude) representing the geographic coordinates.
    """
    
    # Transform the bounding box to the destination projection
    north_east = (bbox['north'], bbox['east'])
    south_west = (bbox['south'], bbox['west'])
    
    # Calculate the geographical coordinates from the pixel positions
    long_pct = x / image_width
    lat_pct = y / image_height
    
    # Interpolate the latitude and longitude within the bounding box
    latitude = north_east[0] - lat_pct * (north_east[0] - south_west[0])
    longitude = south_west[1] + long_pct * (north_east[1] - south_west[1])
    
    return latitude, longitude

class PathFollower(LifecycleNode):
    heading = 0
    latitude = 0
    longitude = 0
    speed_knots = 0
    waypoints = Path()
    current_path = []
    current_grid_cell = (16, 51)
    wind_angle_deg = 0

    def __init__(self):
        super().__init__('path_follower')
        # Using different callback groups for subscription and service client
        self.subscription_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.service_client_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.target_position_publisher: Optional[Publisher]
        self.current_path_publisher: Optional[Publisher]
        self.airmar_heading_subscription: Optional[Subscription]
        self.airmar_position_subscription: Optional[Subscription]
        self.airmar_speed_knots_subscription: Optional[Subscription]
        self.timer: Optional[Timer]
        self.declare_parameter('map_name', 'quinsigamond')
        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value
        self.get_logger().info(f'Map name: {self.map_name}')
        self.get_logger().info("Getting map image")
        image, self.bbox = find_and_load_image(get_resource_dir(), self.map_name)
        cv2.imwrite("/home/sailbot/after_load.jpg", image)


        occupancy_grid_values = np.clip(image, 0, 1)

        #occupancy_grid_values = ((255 - occupancy_grid_values) * 100 / 255).astype(np.int8)
        grid_msg = OccupancyGrid()
        grid_msg.header = Header(frame_id="map")
        grid_msg.info.resolution = 0.0001
        grid_msg.info.width = occupancy_grid_values.shape[1]
        self.image_width = occupancy_grid_values.shape[1]
        grid_msg.info.height = occupancy_grid_values.shape[0]
        self.image_height = occupancy_grid_values.shape[0]
        self.get_logger().info(f"map width: {self.image_width}, height: {self.image_height}")

        grid_msg.info.origin = Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        self.get_logger().info(f"{occupancy_grid_values}")
        grid_msg.data = occupancy_grid_values.flatten().tolist()

        self.get_logger().info("Getting SetMap service")
        self.set_map_cli = self.create_client(SetMap, 'set_map', callback_group=self.service_client_callback_group)
        while not self.set_map_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_map service not available, waiting again...')
        set_map_req = SetMap.Request()
        set_map_req.map = grid_msg
        self.grid_msg = grid_msg
        self.get_logger().info("Setting map")
        future = self.set_map_cli.call_async(set_map_req)
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info("Map setup done")

        self.get_path_cli = self.create_client(GetPath, 'get_path', callback_group=self.service_client_callback_group)
        while not self.get_path_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('get_path service not available, waiting again...')
        
        self.get_logger().info("Path follower node setup complete")


    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")
        try:
            self.target_position_publisher = self.create_lifecycle_publisher(GeoPoint, 'target_position', 10)
            self.current_path_publisher = self.create_lifecycle_publisher(Path, 'current_path', 10)
            self.airmar_heading_subscription = self.create_subscription(
                Float64,
                '/airmar_data/heading',
                self.airmar_heading_callback,
                10, callback_group=self.subscription_callback_group)
            self.airmar_position_subscription = self.create_subscription(
                GeoPoint,
                '/airmar_data/lat_long',
                self.airmar_heading_callback,
                10, callback_group=self.subscription_callback_group)
            self.airmar_speed_knots_subscription = self.create_subscription(
                Float64,
                '/airmar_data/speed_knots',
                self.airmar_speed_knots_callback,
                10, callback_group=self.subscription_callback_group)
            self.waypoints_subscriber = self.create_subscription(
                Path, 'waypoints', self.waypoints_callback, 10, callback_group=self.subscription_callback_group)
            #self.timer = self.create_timer(0.1, self.control_loop_callback)
            #super().on_configure(state)
        
        except Exception as e:
            self.get_logger().info("Error in configure")
            self.get_logger().info(str(e))
        
        self.get_logger().info("Path following node configured")
        
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")
        # Start publishers or timers
        return super().on_activate(state)
        # map_msg = Map()
        # map_msg.grid = self.grid_msg
        # map_msg.north = self.bbox['north']
        # map_msg.south = self.bbox['south']
        # map_msg.east = self.bbox['east']
        # map_msg.west = self.bbox['west']
        # self.current_map_publisher.publish(map_msg)

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
        self.current_grid_cell = self.latlong_to_grid_proj(msg.latitude, msg.longitude, self.bbox, self.image_width, self.image_height)
        self.find_look_ahead()


    def airmar_speed_knots_callback(self, msg: Float64):
        self.speed_knots = msg.data
        self.find_look_ahead()

    def get_path(self, start, goal):
        self.get_logger().info(f"get_path with {start}, {goal}")
        if self.wind_angle_deg is None:
            self.get_logger().info("No wind reported yet, cannot path")
            return
        req = GetPath.Request()
        start_point = Point()
        end_point = Point()
        start_point.x = float(start[0])
        start_point.y = float(start[1])
        end_point.x = float(goal[0])
        end_point.y = float(goal[1])
        req.start = start_point
        req.end = end_point
        req.wind_angle_deg = float(self.wind_angle_deg)
        self.get_logger().info("Getting path")
        #synchronous service call because ROS2 async doesn't work in callbacks
        result = self.get_path_cli.call(req)
        #rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Path returned!")
        return result

    def recalculate_path_from_waypoints(self):
        if self.wind_angle_deg is None:
            self.get_logger().info("No wind reported yet, cannot path")
            return
        
        points = []
        for point in self.waypoints.points:
            points.append(self.latlong_to_grid_proj(point.latitude, point.longitude, self.bbox, self.image_width, self.image_height))
        
        if len(points) == 0:
            self.get_logger().info("Empty waypoints, will clear path")
            self.current_path = Path()
            self.current_path_publisher.publish(self.current_path)
            return
        path_segments = []
        path_segments.append(self.get_path(self.current_grid_cell, points[0]))
        for i in range(len(points)-1):
            path_segments.append(self.get_path(points[i], points[i+1]))
        
        final_path = Path()
        for segment in path_segments:
            for poseStamped in segment.path.poses:
                point = poseStamped.pose.position
                self.get_logger().info(f"point: {point}")
                lat, lon = grid_to_latlong_proj(point.x, point.y, self.bbox, self.image_width, self.image_height)
                geopoint = GeoPoint()
                geopoint.latitude = lat
                geopoint.longitude = lon
                final_path.points.append(geopoint)

        self.get_logger().info(f"New path: {final_path.points}")
        self.current_path_publisher.publish(final_path)
        self.current_path = final_path

    def waypoints_callback(self, msg: Path):
        self.get_logger().info("Got waypoints!")
        self.waypoints = msg
        self.recalculate_path_from_waypoints()
        self.find_look_ahead()
        self.get_logger().info("Ending waypoints callback")
    
    def find_look_ahead(self):
        if len(self.current_path.points) == 0:
            self.get_logger().info("No lookAhead point for zero-length path")
            return
        look_ahead_point = find_look_ahead_point(self.current_path.points, (self.latitude, self.longitude), self.speed_knots)
        self.get_logger().info(f"Calulated lookAhead point: {look_ahead_point.latitude}, {look_ahead_point.longitude}")
        self.target_position_publisher.publish(look_ahead_point)

    def latlong_to_grid_proj(self, latitude, longitude, bbox, image_width, image_height, src_proj='EPSG:4326', dest_proj='EPSG:3857'):
        """
        Convert lat/long coordinates to grid cell using pyproj for projection handling.
        
        Parameters:
        - latitude, longitude: The lat/long coordinates to convert.
        - bbox: A dictionary with keys 'north', 'south', 'east', 'west' representing the bounding box.
        - image_width, image_height: The dimensions of the image in pixels.
        - src_proj: Source projection (latitude/longitude).
        - dest_proj: Destination projection for the image.

        Returns:
        - A tuple (x, y) representing the grid cell coordinates in the image.
        """
        transformer = Transformer.from_crs(src_proj, dest_proj, always_xy=True)
        north_east = transformer.transform(bbox['north'], bbox['east'])
        south_west = transformer.transform(bbox['south'], bbox['west'])
        
        point_x, point_y = transformer.transform(latitude, longitude)
        
        # Calculate the percentage within the transformed bounding box
        long_pct = 1.0-(north_east[1] - point_y) / (north_east[1] - south_west[1])
        lat_pct = (point_x - south_west[0]) / (north_east[0] - south_west[0])
        self.get_logger().info(f"lat_pct: {lat_pct}, long_pct: {long_pct}")
        
        # Convert percentages to pixel positions
        x = int(long_pct * image_width)
        y = int(lat_pct * image_height)
        
        return x, y



def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()

    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.MultiThreadedExecutor()
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