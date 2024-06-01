#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String, Float64, Int16, Header, Empty
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from geographic_msgs.msg import GeoPoint
from sailbot_msgs.msg import GeoPath, Waypoint, WaypointPath, Wind, GaussianThreat, PathSegment, GeoPathSegment, BuoyDetectionStamped
from sailbot_msgs.srv import SetMap, GetPath, SetThreat
from typing import Optional
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from rclpy.timer import Timer
from rclpy.subscription import Subscription
from rclpy.node import ParameterType
from ament_index_python.packages import get_package_share_directory
import os
import cv2
import re
import numpy as np
from pyproj import Proj, Transformer
from shapely.geometry import LineString
from shapely.geometry import Point as shapely_point 
from shapely.affinity import rotate
import math
import traceback
import time

from geopy.distance import great_circle
from geopy.distance import geodesic
from geopy.point import Point as geopy_point
from math import sqrt, radians, degrees, cos, sin
from typing import Tuple, List

def get_maps_dir():
    package_path = get_package_share_directory('sailbot')
    resource_path = os.path.join(package_path, 'maps')
    return resource_path

def distance(x1, y1, x2, y2):
    x2x1 = x2-x1
    y2y1 = y2-y1
    return math.sqrt(x2x1*x2x1 + y2y1*y2y1)

def interpolate_point(point1, point2, fraction):
    lat = point1.latitude + (point2.latitude - point1.latitude) * fraction
    lon = point1.longitude + (point2.longitude - point1.longitude) * fraction
    return GeoPoint(latitude=lat, longitude=lon)

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

def normalize(vector):
    norm = np.linalg.norm(vector)
    return tuple(v / norm for v in vector)

def perpendicular_vector(vector):
    x, y = vector
    return (-y, x)


class PathFollower(LifecycleNode):
    """
    A ROS2 lifecycle node that manages pathfinding and navigation for the boat. It processes waypoints, handles dynamic path updates,
    manages threats, and integrates sensor data to navigate efficiently.

    :ivar latitude: Current latitude of the boat.
    :ivar longitude: Current longitude of the boat.
    :ivar heading: Current boat heading in degrees.
    :ivar speed_knots: Current speed of the boat in knots.
    :ivar wind_angle_deg: Current wind direction in degrees relative to the boat.
    :ivar waypoints: A 'WaypointPath' object containing the waypoints the boat must follow.
    :ivar current_path: A 'GeoPath' object representing the currently planned path.
    :ivar current_grid_path: List of grid coordinates corresponding to the 'current_path'.
    :ivar segment_endpoint_indices: Indices in the 'current_path' where path segments end.
    :ivar buoy_rounding_distance_meters: Distance for rounding buoys during navigation.
    :ivar min_path_recalculation_interval_seconds: Minimum interval between path recalculations to avoid excessive updates.
    :ivar threat_ids: List of internal IDs returned from pathfinder node for identified buoy threats.
    :ivar previous_position_index: Last path point the boat was at, tracked to avoid issues with self-intersecting paths.
    :ivar grid_points: The intermediate step between waypoints and the full grid path, passed to the pathfinder node.
    :ivar exact_points: The intermediate step between waypoints and the full geographical path.
    :ivar current_buoy_positions: Dictionary containing a mapping between buoy IDs and their current positions.
    :ivar last_waypoint_was_rounding_type: Internal state to decide if the last few points might need to be trimmed, as we don't always want to go fully around a buoy.

    **Subscriptions**:

    - 'airmar_heading_subscription': Subscribes to heading updates from the boat's sensors.
    - 'airmar_position_subscription': Subscribes to position updates, updating boat's geographic location.
    - 'waypoints_subscription': Subscribes to lists of waypoints. Currently only used for clearing all waypoints.
    - 'single_waypoint_subscription': Subscribes to single waypoints. Used to add waypoints.
    - 'true_wind_subscription': Subscribes to smoothed true wind data.
    - 'buoy_position_subscription': Subscribes to buoy positions, used for snapping waypoints.
    - 'request_recalculation_subscription': Allows other nodes to request path recalculation.

    **Publishers**:

    - 'current_grid_segment_publisher': Publishes the current segment of the navigation grid being followed.
    - 'current_segment_debug_publisher': Publishes debug information about the current navigation segment.
    - 'target_position_publisher': Publishes the target position the boat is navigating towards.
    - 'current_path_publisher': Publishes updates to the navigation path as it is recalculated.
    - 'current_grid_cell_publisher': Publishes the current grid cell location of the boat within the navigation map.

    **Services**:
    
    - 'set_map_cli', 'get_path_cli', 'set_threat_cli': Service clients for setting the navigation map, retrieving paths, and managing navigation threats.

    **Methods**:

    - 'set_parameters', 'get_parameters': Methods for declaring and retrieving ROS parameters related to navigation and pathfinding.
    - 'calculate_exact_points_from_waypoint': Processes waypoints to calculate exact navigation points.
    - 'recalculate_path_from_exact_points': Recalculates the navigation path based on new or updated exact navigation points.
    - 'find_current_segment': Determines the boat's progress along the path.
    - 'remove_last_points_if_necessary': Trims unnecessary navigation points if the last waypoint was a rounding type.
    - 'get_square_corners': Calculates and orders the four corners of a square about a target position, facing a starting position.

    **Usage**:

    - The node must be managed by state_manager

    **Notes**:

    - This is the vector-field variant of the boat's pathfinding. See path_follower.py for the look-ahead variant.

    """
    heading = 0
    # latitude = 42.273822
    # longitude = -71.805967
    latitude, longitude = 42.0396766107111, -71.84585650616927
    speed_knots = 0
    waypoint = Waypoint()
    current_path = GeoPath()
    current_grid_path = []
    segment_endpoint_indices = []
    #current_grid_cell = (16, 51)
    current_grid_cell = (16, 16)

    wind_angle_deg = 270

    buoy_rounding_distance_meters = None
    min_path_recalculation_interval_seconds = None

    threat_ids = []

    previous_position_index = 0

    grid_points = []
    exact_points = []

    last_recalculation_time = time.time()

    current_buoy_positions = {}
    current_buoy_times = {}
    last_waypoint_was_rounding_type = False

    waypoint_threat_id_map = {}

    earth_radius = 3958.8

    def __init__(self):
        super().__init__('path_follower')
        # Using different callback groups for subscription and service client
        self.subscription_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.service_client_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        self.current_grid_segment_publisher: Optional[Publisher]
        self.current_segment_debug_publisher: Optional[Publisher]
        self.target_position_publisher: Optional[Publisher]
        self.current_path_publisher: Optional[Publisher]
        self.current_grid_cell_publisher: Optional[Publisher]

        self.error_publisher: Optional[Publisher]

        self.airmar_heading_subscription: Optional[Subscription]
        self.airmar_position_subscription: Optional[Subscription]
        self.timer: Optional[Timer]
        
        self.set_parameters()
        self.get_parameters()
    
        self.get_logger().info(f'Map name: {self.map_name}')
        self.get_logger().info("Getting map image")
        image, self.bbox = find_and_load_image(get_maps_dir(), self.map_name)
        #cv2.imwrite("/home/sailbot/after_load.jpg", image)


        occupancy_grid_values = np.clip(image, 0, 1)

        #occupancy_grid_values = ((255 - occupancy_grid_values) * 100 / 255).astype(np.int8)
        grid_msg = OccupancyGrid()
        grid_msg.header = Header(frame_id="map")
        grid_msg.info.resolution = 0.00001
        grid_msg.info.width = occupancy_grid_values.shape[1]
        self.image_width = occupancy_grid_values.shape[1]
        grid_msg.info.height = occupancy_grid_values.shape[0]
        self.image_height = occupancy_grid_values.shape[0]
        self.get_logger().info(f"map width: {self.image_width}, height: {self.image_height}")

        self.current_grid_cell = self.latlong_to_grid_proj(self.latitude, self.longitude, self.bbox, self.image_width, self.image_height)

        grid_msg.info.origin = Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        self.get_logger().info(f"{occupancy_grid_values}")
        grid_msg.data = occupancy_grid_values.flatten().tolist()

        self.get_logger().info("Getting SetMap service")
        self.set_map_cli = self.create_client(SetMap, 'set_map', callback_group=self.service_client_callback_group)
        while not self.set_map_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_map service not available, waiting again...')
        set_map_req = SetMap.Request()
        set_map_req.map = grid_msg
        set_map_req.num_prm_nodes = 3000
        set_map_req.prm_connection_distance_percent = 5
        self.grid_msg = grid_msg
        self.get_logger().info("Setting map")
        future = self.set_map_cli.call_async(set_map_req)
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info("Map setup done")

        self.get_path_cli = self.create_client(GetPath, 'get_path', callback_group=self.service_client_callback_group)
        while not self.get_path_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('get_path service not available, waiting again...')
        
        self.set_threat_cli = self.create_client(SetThreat, 'set_threat', callback_group=self.service_client_callback_group)
        while not self.set_threat_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_threat service not available, waiting again...')
        
        self.get_logger().info("Path follower node setup complete")


    def set_parameters(self) -> None:
        self.declare_parameter('sailbot.pathfinding.min_path_recalculation_interval_seconds', 10.0)
        self.declare_parameter('map_name', 'quinsigamond')

    def get_parameters(self) -> None:
        self.min_path_recalculation_interval_seconds = self.get_parameter('sailbot.pathfinding.min_path_recalculation_interval_seconds').get_parameter_value().double_value
        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value

    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")
        try:
            self.current_grid_segment_publisher = self.create_lifecycle_publisher(PathSegment, 'current_path_segment', 10)
            self.current_segment_debug_publisher = self.create_lifecycle_publisher(GeoPathSegment, 'current_segment_debug', 10)
            self.target_position_publisher = self.create_lifecycle_publisher(GeoPoint, 'target_position', 10)
            self.current_path_publisher = self.create_lifecycle_publisher(GeoPath, 'current_path', 10)
            self.current_grid_cell_publisher = self.create_lifecycle_publisher(Point, 'current_grid_cell', 10)

            self.error_publisher = self.create_lifecycle_publisher(String, f'{self.get_name()}/error', 10)

            self.airmar_heading_subscription = self.create_subscription(
                Float64,
                'heading',
                self.heading_callback,
                10,
                callback_group=self.subscription_callback_group)
            self.airmar_position_subscription = self.create_subscription(
                NavSatFix,
                '/airmar_data/lat_long',
                self.airmar_position_callback,
                10,
                callback_group=self.subscription_callback_group)
            self.waypoints_subscription = self.create_subscription(
                WaypointPath, 
                'waypoints',
                self.waypoints_callback,
                10, 
                callback_group=self.subscription_callback_group)
            self.single_waypoint_subscription = self.create_subscription(
                Waypoint, 
                'single_waypoint',
                self.single_waypoint_callback,
                10, 
                callback_group=self.subscription_callback_group)
            self.true_wind_subscription = self.create_subscription(
                Wind,
                'true_wind_smoothed',
                self.true_wind_callback,
                10,
                callback_group=self.subscription_callback_group)
            self.buoy_position_subscription = self.create_subscription(
                BuoyDetectionStamped,
                'buoy_position',
                self.buoy_position_callback,
                10,
                callback_group = self.subscription_callback_group)
            
            self.request_replan_subscription = self.create_subscription(
                Empty,
                'request_replan',
                self.request_replan_callback,
                10,
                callback_group = self.subscription_callback_group)
        
        except Exception as e:
            self.get_logger().info("Error in configure")
            self.get_logger().info(str(e))
            raise(e)
        
        self.get_logger().info("Path following node configured")
        
        return super().on_configure(state)

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")
        # Start publishers or timers
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")
        # Destroy subscribers, publishers, and timers
        self.destroy_timer(self.timer)
        self.destroy_lifecycle_publisher(self.current_grid_cell_publisher)
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

    def heading_callback(self, msg: Float64) -> None:
        #self.get_logger().info("Got heading")
        self.heading = msg.data
    
    def airmar_position_callback(self, msg: NavSatFix) -> None:
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        new_grid_cell = self.latlong_to_grid_proj(self.latitude, self.longitude, self.bbox, self.image_width, self.image_height)
        current_time = time.time()
        #self.get_logger().info("Got new position")

        if new_grid_cell != self.current_grid_cell and (current_time-self.last_recalculation_time > self.min_path_recalculation_interval_seconds):
            self.get_logger().info("Recalculating path")
            self.recalculate_path_from_exact_points()
            self.last_recalculation_time = time.time()
            
        self.find_current_segment()
        self.current_grid_cell = new_grid_cell
        grid_cell_msg = Point()
        grid_cell_msg.x = float(new_grid_cell[0])
        grid_cell_msg.y = float(new_grid_cell[1])
        self.current_grid_cell_publisher.publish(grid_cell_msg)

    def get_path(self, start, goal) -> GetPath.Response:
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
        req.pathfinding_strategy = GetPath.Request.PATHFINDING_STRATEGY_PRMSTAR
        
        #Pathfinder assumes 0 is along the +X axis. Airmar data is 0 along -y axis.
        wind_angle_adjusted = self.wind_angle_deg+90


        req.wind_angle_deg = float(wind_angle_adjusted)
        self.get_logger().info("Getting path")
        #synchronous service call because ROS2 async doesn't work in callbacks
        result = self.get_path_cli.call(req)

        self.get_logger().info("Path returned!")
        return result
    
    def calculate_initial_bearing(self, point_A, point_B) -> float:
        lat1, lon1 = map(radians, point_A)
        lat2, lon2 = map(radians, point_B)

        dLon = lon2 - lon1
        x = math.sin(dLon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dLon))
        initial_bearing = math.atan2(x, y)

        return degrees(initial_bearing)

    def generate_lawnmower_pattern(self, center_lat, center_lon, radius_meters, wind_direction_deg):
        """
        Generate a lawnmower pattern search path within a given radius around a center point.

        Parameters:
        - center_lat: Center latitude of the search circle.
        - center_lon: Center longitude of the search circle.
        - radius_meters: Radius of the search circle in meters.
        - wind_direction_deg: Current wind direction in degrees from north.

        Updates:
        - Updates the 'exact_points' and 'grid_points' with the waypoints for the lawnmower pattern.
        """

        track_spacing=10 # Meters

        # Calculate number of tracks needed
        num_tracks = int(2 * radius_meters / track_spacing)

        # Initialize the list of points
        self.exact_points = []

        for i in range(num_tracks):
            # Calculate the radial distance from the center
            d = i * track_spacing - radius_meters
            if abs(d) > radius_meters:
                continue  # Skip if the radial distance is outside the circle

            # Calculate the chord length at this radial distance
            chord_length = 2 * sqrt(radius_meters**2 - d**2) if abs(d) <= radius_meters else 0
            self.get_logger().info(f"Chord length: {chord_length}")
            half_chord = chord_length / 2

            # Calculate the bearing for the midpoint of the chord along the wind direction
            radial_bearing = (wind_direction_deg+180)%360
            perpendicular_bearing = (wind_direction_deg + 90) % 360
            self.get_logger().info(f"perp. bearing: {perpendicular_bearing}")


            # Find the midpoint along the wind vector
            midpoint = geodesic(meters=d).destination((center_lat, center_lon), radial_bearing)
            self.get_logger().info(f"midpoint: {midpoint}")


            # Calculate the start and end points of the chord perpendicular to the wind direction
            if i%2==0:
                start_point = geodesic(meters=half_chord).destination((midpoint.latitude, midpoint.longitude), perpendicular_bearing)
                end_point = geodesic(meters=half_chord).destination((midpoint.latitude, midpoint.longitude), (perpendicular_bearing + 180) % 360)
            else:
                end_point = geodesic(meters=half_chord).destination((midpoint.latitude, midpoint.longitude), perpendicular_bearing)
                start_point = geodesic(meters=half_chord).destination((midpoint.latitude, midpoint.longitude), (perpendicular_bearing + 180) % 360)
            self.get_logger().info(f"start point: {start_point}")
            self.get_logger().info(f"end point: {end_point}")


            self.exact_points.append(GeoPoint(latitude=start_point.latitude, longitude=start_point.longitude))
            self.exact_points.append(GeoPoint(latitude=end_point.latitude, longitude=end_point.longitude))
        
        # Remove first point, duplicate from artifact of circle calculations
        self.exact_points.pop(0)

        self.grid_points = [self.latlong_to_grid_proj(p.latitude, p.longitude, self.bbox, self.image_width, self.image_height) for p in self.exact_points]

        self.get_logger().info(f"grid points: {self.grid_points}")


    def recalculate_path_from_exact_points(self) -> None:
        """
        Recalculates the current path based on exact geographic points and grid coordinates determined by previous processing.
        This function constructs the path segment by segment, updating it with any changes in conditions or coordinates.

        The function checks for wind conditions, recalculates path segments between exact points, and publishes the updated path.

        :return: None. The function directly updates the 'current_path' and 'current_grid_path' attributes of the instance, 
                and publishes the new path to a designated topic.

        Function behavior includes:
        - Verifying if wind conditions are reported; if not, the function logs a message and exits without updating the path.
        - If there are no waypoints to process (i.e., if 'grid_points' is empty), the function clears the current path and logs this event.
        - Creating a new path by connecting all exact points through calculated segments, while taking into account wind angle.
        - Dynamically inserting intermediate points into path segments to enhance navigation precision.
        - Publishing the newly calculated path for use by the navigation system.
        - Logging various state changes and decisions for debugging and monitoring purposes.

        This function assumes that 'current_grid_cell' and 'grid_points' are available within the class instance 
        and are appropriately set before calling this function.
        """
         
        if self.wind_angle_deg is None:
            self.get_logger().info("No wind reported yet, cannot path")
            return
        
        # Reset look-ahead, since previous values are not relevant anymore
        self.previous_position_index = 0
        if len(self.grid_points) == 0:
            self.get_logger().info("Empty waypoints, will clear path")
            self.current_path = GeoPath()
            self.current_path_publisher.publish(self.current_path)
            return
        path_segments = []
        path_segments.append(self.get_path(self.current_grid_cell, self.grid_points[0]).path)
        for i in range(len(self.grid_points)-1):
            path_segments.append(self.get_path(self.grid_points[i], self.grid_points[i+1]).path)
        
        self.get_logger().info("Calculated all path segments")
        for segment in path_segments:
           segment.poses = self.insert_intermediate_points(segment.poses, 1.0)

        final_path = GeoPath()
        final_grid_path = []
        i=-1
        k=0
        final_grid_path.append(Point(x=float(self.current_grid_cell[0]), y=float(self.current_grid_cell[1])))
        final_path.points.append(GeoPoint(latitude=self.latitude, longitude=self.longitude))

        # Track the indices in the current path which correspond to endpoints of straight-line path segments
        segment_endpoint_indices = [0]
        for segment in path_segments:
            #skip failed waypoints
            if(len(segment.poses)==0):
                continue 

            for j in range(1, len(segment.poses)-1):
                poseStamped = segment.poses[j]
                point = poseStamped.pose.position
                #self.get_logger().info(f"point: {point}")
                lat, lon = self.grid_to_latlong_proj(point.x, point.y, self.bbox, self.image_width, self.image_height)
                geopoint = GeoPoint()
                geopoint.latitude = lat
                geopoint.longitude = lon
                #append latlon position to global path, and grid point to grid path
                final_path.points.append(geopoint)
                final_grid_path.append(point)
                k+=1
            #append exact final position
            #self.get_logger().info(f"num exact points: {len(self.exact_points)}, i: {i}")
            final_path.points.append(self.exact_points[i+1])
            final_grid_path.append(segment.poses[len(segment.poses)-1].pose.position)
            segment_endpoint_indices.append(len(segment.poses)-1)
            #self.waypoint_indices.append(k)
            i+=1


        #self.get_logger().info(f"New path: {final_path.points}")
        self.current_path_publisher.publish(final_path)
        self.current_path = final_path
        self.current_grid_path = final_grid_path
        self.segment_endpoint_indices = segment_endpoint_indices

    #currently only used to clear points.
    def waypoints_callback(self, msg: WaypointPath) -> None:
        self.get_logger().info("Got waypoints!")
        self.waypoints = msg
        self.clear_threats()
        self.exact_points = []
        self.grid_points = []
        self.recalculate_path_from_exact_points()
        self.find_current_segment()
        self.get_logger().info("Ending waypoints callback")


    def single_waypoint_callback(self, msg: Waypoint) -> None:
        """
        Callback function that processes a single waypoint message. This function updates the waypoint list, calculates exact points, 
        recalculates the path, and determines the look-ahead point based on the received waypoint message.

        :param msg: A sailbot_msgs/Waypoint object that contains details such as the type of waypoint and its geographic coordinates.

        :return: None. This function triggers a series of operations that update the internal state of the navigation system, 
                including appending the waypoint to a list, potentially adding a threat to the map based on the waypoint type, 
                recalculating the navigation path, and updating the look-ahead mechanism.

        Function behavior includes:
        - Logging receipt of a new waypoint.
        - Appending the new waypoint to the 'waypoints' list.
        - Adding the waypoint as a threat if it is a rounding waypoint (either to the right or left).
        - Calling the function to calculate exact points from the new waypoint.
        - Recalculating the entire path based on updated waypoint data.
        - Determining and updating the look-ahead point for navigation.
        - Logging the completion of processing for the waypoint.

        This function is intended to be used as a callback for a waypoint message subscriber in a ROS2 node environment.
        """
        self.get_logger().info("Got single waypoint")
        self.waypoint = msg
        
        self.generate_lawnmower_pattern(msg.point.latitude, msg.point.longitude, 50, self.wind_angle_deg)

        self.recalculate_path_from_exact_points()
        self.find_current_segment()
        self.get_logger().info("Ending single waypoint callback")


    def true_wind_callback(self, msg: Wind) -> None:
        self.wind_angle_deg = msg.direction

    def buoy_position_callback(self, msg: BuoyDetectionStamped) -> None:
        self.current_buoy_positions[msg.id] = msg
        self.current_buoy_times[msg.id] = time.time()

    def request_replan_callback(self, msg: Empty) -> None:
        current_time = time.time()
        # Allow this recalculation at most once per second
        if(current_time-self.last_recalculation_time<1.0):
            return
        
        self.last_recalculation_time = time.time()
        self.recalculate_path_from_exact_points()
        self.find_current_segment()
    
    def find_current_segment(self) -> None:
        """
        Determines and updates the vector field target segment based on the current navigation path. 
        This function also handles publishing various navigation-related messages to update
        the system's state and debug information.

        :return: None. This function modifies the navigation state by publishing the current grid cell, target position, 
                and path segments for display or further processing.

        Function behavior includes:
        - Publishing the current grid cell position.
        - Early exit if the path has zero length.
        - Iterating through path points to find the appropriate target path segment based on calculated distance.
        - Conditionally publishing target positions and path segments for system updates and debugging.
        - Checking if the next path point is closer than the current target to avoid backtracking.
        - Managing and removing passed exact points to maintain an updated path.

        This function assumes that instance attributes 'latitude', 'longitude', 'current_path', 
        'current_grid_path', and 'exact_points' are properly initialized and available.
        """
        grid_cell_msg = Point()
        grid_cell_msg.x = float(self.current_grid_cell[0])
        grid_cell_msg.y = float(self.current_grid_cell[1])
        self.current_grid_cell_publisher.publish(grid_cell_msg)

        if len(self.current_path.points) == 0:
            #self.get_logger().info("No lookAhead point for zero-length path")
            return
        
        #self.get_logger().info(f"Grid path length: {len(self.current_grid_path)}")
        num_points = len(self.current_path.points) 
        for i in range(self.previous_position_index, num_points-1):
            point = self.current_path.points[i]
            distance = great_circle((self.latitude, self.longitude), (point.latitude, point.longitude)).meters
            # Check if the next point is closer. If so, we probably skipped some points. Don't target them. 
            next_is_closer = False if i>=num_points else (True if great_circle((self.latitude, self.longitude), (self.current_path.points[i+1].latitude, self.current_path.points[i+1].longitude)).meters<distance else False)
            #self.get_logger().info(f"next_is_closer: {next_is_closer}")
            if(not next_is_closer):
                self.previous_position_index = i
                #self.get_logger().info(f"Calulated current point: {point.latitude}, {point.longitude}")
                self.target_position_publisher.publish(point) # In this version, this is just for display in the UI. This is NOT an input to heading_controller_vf 
                segment = PathSegment()
                segment.start = self.current_grid_path[i]
                segment.end = self.current_grid_path[i+1]
                self.current_grid_segment_publisher.publish(segment)
                geoSegment = GeoPathSegment()
                geoSegment.start = self.current_path.points[i]
                geoSegment.end = self.current_path.points[i+1]
                self.current_segment_debug_publisher.publish(geoSegment)

                return
            else:
                #remove exact points if we've passed them
                #self.get_logger().info(f"Point is: {point}")
                if(point.latitude == self.exact_points[0].latitude and point.longitude == self.exact_points[0].longitude):
                    self.get_logger().info("Removing passed exact point")
                    self.grid_points.pop(0)
                    self.exact_points.pop(0)

    def vector_to_heading(self, dx, dy):
        """
        Convert vector components (dx, dy) at a grid position to a navigation heading.

        Parameters:
        - dx, dy: Changes in x and y grid coordinates.

        Returns:
        - Navigation bearing in degrees from north.
        """
        theta = math.atan2(dy, dx)  # Angle in radians
        bearing = (90 - math.degrees(theta)) % 360
        return bearing

    def latlong_to_grid_proj(self, latitude: float, longitude: float, bbox: dict, image_width: int, image_height: int, src_proj='EPSG:4326', dest_proj='EPSG:3857') -> Tuple[float, float]:
        """
        Convert latitude and longitude coordinates to grid cell coordinates using pyproj for projection handling.

        :param latitude: The latitude coordinate to convert.
        :param longitude: The longitude coordinate to convert.
        :param bbox: A dictionary with keys 'north', 'south', 'east', 'west' representing the bounding box.
        :param image_width: The width of the image in pixels.
        :param image_height: The height of the image in pixels.
        :param src_proj: Source projection (usually latitude/longitude).
        :param dest_proj: Destination projection for converting geographic coordinates into grid coordinates.

        :return: A tuple (x, y) representing the grid cell coordinates in the image.
        """
        transformer = Transformer.from_crs(src_proj, dest_proj, always_xy=True)
        north_east = transformer.transform(bbox['north'], bbox['east'])
        south_west = transformer.transform(bbox['south'], bbox['west'])
        
        point_x, point_y = transformer.transform(latitude, longitude)
        
        # Calculate the percentage within the transformed bounding box
        long_pct = 1.0-(north_east[1] - point_y) / (north_east[1] - south_west[1])
        lat_pct = (point_x - south_west[0]) / (north_east[0] - south_west[0])
        #self.get_logger().info(f"lat_pct: {lat_pct}, long_pct: {long_pct}")
        
        # Convert percentages to pixel positions
        x = (long_pct * image_width)
        y = (lat_pct * image_height)
        
        return x, y

    def grid_to_latlong_proj(self, x: int, y: int, bbox: dict, image_width, image_height) -> Tuple[float, float]:
        """
        Convert grid cell coordinates in an image to latitude/longitude coordinates using specified projections.

        :param x: The x-coordinate (pixel position) in the image.
        :param y: The y-coordinate (pixel position) in the image.
        :param bbox: A dictionary with keys 'north', 'south', 'east', 'west' representing the bounding box.
        :param image_width: The width of the image in pixels.
        :param image_height: The height of the image in pixels.

        :return: A tuple (latitude, longitude) representing the geographic coordinates.
        """
        
        # Transform the bounding box to the destination projection
        north_east = (bbox['north'], bbox['east'])
        south_west = (bbox['south'], bbox['west'])

        lat_res =  abs(bbox['north']-bbox['south'])/image_height
        long_res = abs(bbox['east']-bbox['west'])/image_width
        #self.get_logger().info(f"Lat res: {lat_res}")
        #self.get_logger().info(f"Long res: {long_res}")

        # Calculate the geographical coordinates from the pixel positions
        long_pct = x / image_width
        lat_pct = 1.0-(y / image_height)
        
        # Interpolate the latitude and longitude within the bounding box
        latitude = (north_east[0] - lat_pct * (north_east[0] - south_west[0]))+(lat_res/2)
        longitude = (south_west[1] + long_pct * (north_east[1] - south_west[1]))+(long_res/2)
        
        return latitude, longitude
    
    def insert_intermediate_points(self, path, num_per_unit_distance) -> List[PoseStamped]:
        """
        Inserts intermediate points into a given path segment to simplify more granular path following.
        The number of points inserted between each pair of original points is determined by the distance between them
        multiplied by a specified factor ('num_per_unit_distance').

        :param path: A list of 'PoseStamped' objects representing the waypoints of a path segment.
        :param num_per_unit_distance: A floating point number that determines how many points to insert per unit of distance
                                    between each pair of consecutive waypoints in the path.

        :return: A list of 'PoseStamped' objects with the newly inserted intermediate points included.

        Function behavior includes:
        - Logging a warning and returning the original path if it has zero length.
        - Dynamically calculating the number of intermediate points to insert based on the distance between consecutive waypoints.
        - Using linear interpolation to determine the position of each intermediate point between each consecutive waypoint pair.
        - Returning a new path list that includes both the original and the newly interpolated points.
        """
        length = len(path)
        if(length  == 0):
            self.get_logger().warn("Called insert_intermediate_points on a zero length segment!")
            return path
        appended = []
        for i in range(length):
            if i<(length-1):
                num = round(distance(path[i].pose.position.x, path[i].pose.position.y, path[i+1].pose.position.x, path[i+1].pose.position.y)*num_per_unit_distance)
                #self.get_logger().info(f"Num to insert: {num}")
                appended.append(path[i])
                x_step = (path[i+1].pose.position.x-path[i].pose.position.x)/(num+1)
                #self.get_logger().info(f"X step: {x_step}")
                y_step = (path[i+1].pose.position.y-path[i].pose.position.y)/(num+1)
                #self.get_logger().info(f"y step: {y_step}")
                for j in range(1, num+1):
                    new_x = path[i].pose.position.x+x_step*j
                    new_y = path[i].pose.position.y+y_step*j
                    new_point = PoseStamped()
                    new_point.pose.position.x = new_x
                    new_point.pose.position.y = new_y
                    appended.append(new_point)
        appended.append(path[-1])
        return appended
    
    def publish_error(self, string: str):
        error_msg = String()
        error_msg.data = string
        self.error_publisher.publish(error_msg)

def trigger_error_transition(node: LifecycleNode):
    client = node.create_client(ChangeState, f'{node.get_name()}/change_state')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')

    request = ChangeState.Request()
    request.transition.id = Transition.TRANSITION_DEACTIVATE # Transition to inactive state

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('Transition to error state successful')
    else:
        node.get_logger().info('Failed to transition to error state')


def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()

    # Use the MultiThreadedExecutor to spin the node.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(path_follower)

    try:
        # Spin the node to execute callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        trace = traceback.format_exc()
        error_string = f'Unhandled exception: {e}\n{trace}'
        path_follower.get_logger().fatal(error_string)
        path_follower.publish_error(error_string)
    finally:
        # Shutdown and cleanup the node
        executor.shutdown()
        path_follower.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()