#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String, Float64, Int16, Header
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from geographic_msgs.msg import GeoPoint
from sailbot_msgs.msg import GeoPath, Waypoint, WaypointPath, Wind, GaussianThreat, PathSegment, GeoPathSegment
from sailbot_msgs.srv import SetMap, GetPath, SetThreat
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
import math
import traceback
import time

from geopy.distance import great_circle
from geopy.distance import geodesic
from geopy.point import Point as geopy_point
from math import sqrt, radians, degrees
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


class PathFollower(LifecycleNode):
    heading = 0
    # latitude = 42.273822
    # longitude = -71.805967
    latitude, longitude = 42.0396766107111, -71.84585650616927
    speed_knots = 0
    waypoints = WaypointPath()
    current_path = GeoPath()
    current_grid_path = []
    segment_endpoint_indices = []
    #current_grid_cell = (16, 51)
    current_grid_cell = (16, 16)

    wind_angle_deg = 270

    buoy_rounding_distance_meters = None
    min_path_recalculation_interval_seconds = None

    threat_ids = []

    previous_look_ahead_index = 0

    #waypoint_indices = []

    last_recalculation_time = time.time()

    current_buoy_position = None

    waypoint_threat_id_map = {}

    def __init__(self):
        super().__init__('path_follower')
        # Using different callback groups for subscription and service client
        self.subscription_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.service_client_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.current_grid_segment_publisher: Optional[Publisher]
        self.current_segment_debug_publisher: Optional[Publisher]
        self.target_position_publisher: Optional[Publisher]
        self.current_path_publisher: Optional[Publisher]
        self.current_grid_cell_publisher: Optional[Publisher]

        self.airmar_heading_subscription: Optional[Subscription]
        self.airmar_position_subscription: Optional[Subscription]
        self.airmar_speed_knots_subscription: Optional[Subscription]
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
        self.declare_parameter('sailbot.pathfinding.buoy_rounding_distance_meters', 5.0)
        self.declare_parameter('sailbot.pathfinding.buoy_threat_size_map_units', 1.0)
        self.declare_parameter('sailbot.pathfinding.buoy_threat_guassian_intensity', 1.0)
        self.declare_parameter('sailbot.pathfinding.min_path_recalculation_interval_seconds', 10.0)
        self.declare_parameter('sailbot.navigation.look_ahead_distance_meters', 5.0)
        self.declare_parameter('sailbot.navigation.look_ahead_increase_per_knot', 1.0)
        self.declare_parameter('sailbot.navigation.buoy_snap_distance_meters', 10.0)


        self.declare_parameter('map_name', 'quinsigamond')

    def get_parameters(self) -> None:
        self.buoy_rounding_distance_meters = self.get_parameter('sailbot.pathfinding.buoy_rounding_distance_meters').get_parameter_value().double_value
        self.buoy_threat_size_map_units = self.get_parameter('sailbot.pathfinding.buoy_threat_size_map_units').get_parameter_value().double_value
        self.buoy_threat_guassian_intensity = self.get_parameter('sailbot.pathfinding.buoy_threat_guassian_intensity').get_parameter_value().double_value
        self.min_path_recalculation_interval_seconds = self.get_parameter('sailbot.pathfinding.min_path_recalculation_interval_seconds').get_parameter_value().double_value
        self.look_ahead_distance_meters = self.get_parameter('sailbot.navigation.look_ahead_distance_meters').get_parameter_value().double_value
        self.look_ahead_increase_per_knot = self.get_parameter('sailbot.navigation.look_ahead_increase_per_knot').get_parameter_value().double_value
        self.buoy_snap_distance_meters = self.get_parameter('sailbot.navigation.buoy_snap_distance_meters').get_parameter_value().double_value
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

            self.airmar_heading_subscription = self.create_subscription(
                Float64,
                '/airmar_data/heading',
                self.airmar_heading_callback,
                10,
                callback_group=self.subscription_callback_group)
            self.airmar_position_subscription = self.create_subscription(
                NavSatFix,
                '/airmar_data/lat_long',
                self.airmar_position_callback,
                10,
                callback_group=self.subscription_callback_group)
            self.airmar_speed_knots_subscription = self.create_subscription(
                Float64,
                '/airmar_data/speed_knots',
                self.airmar_speed_knots_callback,
                10,
                callback_group=self.subscription_callback_group)
            self.waypoints_subscriber = self.create_subscription(
                WaypointPath, 
                'waypoints',
                self.waypoints_callback,
                10, 
                callback_group=self.subscription_callback_group)
            self.single_waypoint_subscriber = self.create_subscription(
                Waypoint, 
                'single_waypoint',
                self.single_waypoint_callback,
                10, 
                callback_group=self.subscription_callback_group)
            self.true_wind_subscriber = self.create_subscription(
                Wind,
                'true_wind_smoothed',
                self.true_wind_callback,
                10,
                callback_group=self.subscription_callback_group)
            self.buoy_position_subscriber = self.create_subscription(
                GeoPoint,
                'buoy_position',
                self.buoy_position_callback,
                10,
                callback_group = self.subscription_callback_group)
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

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")
        super().on_deactivate(state)

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

    def airmar_heading_callback(self, msg: Float64) -> None:
        pass#self.heading = msg.data
    
    def airmar_position_callback(self, msg: NavSatFix) -> None:
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        new_grid_cell = self.latlong_to_grid_proj(self.latitude, self.longitude, self.bbox, self.image_width, self.image_height)
        current_time = time.time()
        if new_grid_cell != self.current_grid_cell and (current_time-self.last_recalculation_time > self.min_path_recalculation_interval_seconds):
            self.current_grid_cell = new_grid_cell
            grid_cell_msg = Point()
            grid_cell_msg.x = float(new_grid_cell[0])
            grid_cell_msg.y = float(new_grid_cell[1])
            self.current_grid_cell_publisher.publish(grid_cell_msg)
            self.get_logger().info("Recalculating path")
            self.recalculate_path_from_waypoints()
            self.last_recalculation_time = time.time()
            
        self.find_look_ahead()


    def airmar_speed_knots_callback(self, msg: Float64) -> None:
        self.speed_knots = msg.data
        self.find_look_ahead()

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
        
        #Pathfinder assumes 0 is along the +X axis. Airmar data is 0 along +y axis.
        wind_angle_adjusted = self.wind_angle_deg-90


        req.wind_angle_deg = float(wind_angle_adjusted)
        self.get_logger().info("Getting path")
        #synchronous service call because ROS2 async doesn't work in callbacks
        result = self.get_path_cli.call(req)
        #rclpy.spin_until_future_complete(self, future)
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

    def get_square_corners(self, A, B, side_length, direction) -> List[Tuple[float, float]]:
        # Convert side length to diagonal distance
        diagonal_distance = (side_length * sqrt(2)) / 2  # Half diagonal for geodesic distance
        
        def destination_point(start_point, bearing, distance):
            return geodesic(meters=distance).destination(start_point, bearing)
        
        # Initial point and bearing
        bearing_AB = self.calculate_initial_bearing(A, B)
        bearings = [(bearing_AB + angle) % 360 for angle in [45, -45, 135, -135]]  # four bearings for a perpendicular square
        
        point_B = geopy_point(B)
        # 0, 1, 2, 3 = back right, back left, front right, front left
        corners = [destination_point(point_B, bearing, diagonal_distance) for bearing in bearings]
        if (direction == "right"):
            return [corners[3], corners[1], corners[0], corners[2]]
        elif (direction == "left"):
            return [corners[2], corners[0], corners[1], corners[3]]
        
        #This should not happen
        return None

    def get_relevant_square_corners(self, target_point: GeoPoint, previous_point: GeoPoint, next_point: GeoPoint, direction) -> List[Tuple[float, float]]:
        corners = self.get_square_corners((previous_point.latitude, previous_point.longitude), (target_point.latitude, target_point.longitude), self.buoy_rounding_distance_meters, direction)
        self.get_logger().info(f"Target: {target_point}, previous: {previous_point}, next: {next_point}, corners: {corners}")
        if next_point is None:
            return corners
        
        relevant = corners[:2]
        d0 = great_circle(corners[1], (next_point.latitude, next_point.longitude)).meters
        d1 = great_circle(corners[2], (next_point.latitude, next_point.longitude)).meters
        if(d1<d0):
            relevant.append(corners[2])
        d2 = great_circle(corners[3], (next_point.latitude, next_point.longitude)).meters
        if(d2<d1):
            relevant.append(corners[3])
        
        return relevant

    waypoint_segment_last_coords = [] 
    def recalculate_path_from_waypoints(self) -> None:
        if self.wind_angle_deg is None:
            self.get_logger().info("No wind reported yet, cannot path")
            return
        
        # Reset look-ahead, since previous values are not relevant anymore
        self.previous_look_ahead_index = 0

        #self.waypoint_indices = []
        
        grid_points = []
        exact_points = []

        index=0
        buoy_snap_index = None
        if(self.current_buoy_position is not None):
            closestDistance = float('inf')
            for waypoint in self.waypoints.waypoints:
                if (waypoint.type != Waypoint.WAYPOINT_TYPE_INTERSECT):
                    distance = geodesic((self.current_buoy_position.latitude, self.current_buoy_position.longitude), (waypoint.point.latitude, waypoint.point.longitude)).meters
                    if(distance<self.buoy_snap_distance_meters and distance<closestDistance):
                        closestDistance = distance
                        buoy_snap_index = index
                index+=1
        
        waypointIndex = 0
        for waypoint in self.waypoints.waypoints:
            #if we just want to intersect the point, we can add it directly
            if (waypoint.type == Waypoint.WAYPOINT_TYPE_INTERSECT):
                self.get_logger().info(f"Intersect")
                grid_points.append(self.latlong_to_grid_proj(waypoint.point.latitude, waypoint.point.longitude, self.bbox, self.image_width, self.image_height))
                exact_points.append(waypoint.point)
                self.waypoint_segment_last_coords.append(waypoint.point)
            #otherwise, we need to do some math
            else:
                nextPoint = None
                if(len(self.waypoints.waypoints)>waypointIndex+1):
                    nextPoint = self.waypoints.waypoints[waypointIndex+1].point
                previousPoint = None
                if(waypointIndex != 0):
                    previousPoint = self.waypoints.waypoints[waypointIndex-1].point
                else:
                    previousPoint = GeoPoint(latitude=self.latitude, longitude=self.longitude)

                corners = []
                adjustedPoint = waypoint.point
                if(buoy_snap_index is not None and buoy_snap_index == waypointIndex):
                    adjustedPoint = self.current_buoy_position
                    threat_id = self.waypoint_threat_id_map[(waypoint.point.latitude, waypoint.point.longitude)]
                    self.get_logger().info(f"Threat of id {threat_id} being modified")
                    self.add_threat(Waypoint(point=self.current_buoy_position, type=waypoint.type), id=threat_id)
                    self.get_logger().info(f"Snapping index {buoy_snap_index} to buoy: {self.current_buoy_position}")
                if waypoint.type == Waypoint.WAYPOINT_TYPE_CIRCLE_RIGHT:
                    corners = self.get_relevant_square_corners(adjustedPoint, previousPoint, nextPoint, "right")
                    self.get_logger().info(f"Circle right {corners}")
                elif waypoint.type == Waypoint.WAYPOINT_TYPE_CIRCLE_LEFT:
                    self.get_logger().info(f"Circle left {corners}")
                    corners = self.get_relevant_square_corners(adjustedPoint, previousPoint, nextPoint, "left")
                for corner in corners:
                    grid_points.append(self.latlong_to_grid_proj(corner[0], corner[1], self.bbox, self.image_width, self.image_height))
                    exact_points.append(GeoPoint(latitude = corner[0], longitude = corner[1]))
                if(len(corners)>0):
                    self.waypoint_segment_last_coords.append(GeoPoint(latitude = corners[-1][0], longitude=corners[-1][1]))


            waypointIndex+=1

        if len(grid_points) == 0:
            self.get_logger().info("Empty waypoints, will clear path")
            self.current_path = GeoPath()
            self.current_path_publisher.publish(self.current_path)
            return
        path_segments = []
        path_segments.append(self.get_path(self.current_grid_cell, grid_points[0]).path)
        for i in range(len(grid_points)-1):
            path_segments.append(self.get_path(grid_points[i], grid_points[i+1]).path)
        
        for segment in path_segments:
           segment.poses = self.insert_intermediate_points(segment.poses, 0.1)

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
                self.get_logger().info(f"point: {point}")
                lat, lon = self.grid_to_latlong_proj(point.x, point.y, self.bbox, self.image_width, self.image_height)
                geopoint = GeoPoint()
                geopoint.latitude = lat
                geopoint.longitude = lon
                #append latlon position to global path, and grid point to grid path
                final_path.points.append(geopoint)
                final_grid_path.append(point)
                k+=1
            #append exact final position
            self.get_logger().info(f"num waypoints: {len(exact_points)}, i: {i}")
            final_path.points.append(exact_points[i+1])
            final_grid_path.append(segment.poses[len(segment.poses)-1].pose.position)
            segment_endpoint_indices.append(len(segment.poses)-1)
            #self.waypoint_indices.append(k)
            i+=1


        self.get_logger().info(f"New path: {final_path.points}")
        self.current_path_publisher.publish(final_path)
        self.current_path = final_path
        self.current_grid_path = final_grid_path
        self.segment_endpoint_indices = segment_endpoint_indices

    def waypoints_callback(self, msg: WaypointPath) -> None:
        self.get_logger().info("Got waypoints!")
        self.waypoints = msg
        self.clear_threats()
        self.recalculate_path_from_waypoints()
        self.find_look_ahead()
        self.get_logger().info("Ending waypoints callback")
    
    def clear_threats(self) -> None:
        for id in self.threat_ids:
            req = SetThreat.Request()
            req.id = id
            req.remove = True
            self.get_logger().info("Removing threat")
            #synchronous service call because ROS2 async doesn't work in callbacks
            result = self.set_threat_cli.call(req)
            self.get_logger().info("Threat removed")


    def add_threat(self, waypoint, id=-1) -> None:
        threat = GaussianThreat()
        threat.size = self.buoy_threat_size_map_units
        threat.intensity = 1.0
        x, y = self.latlong_to_grid_proj(waypoint.point.latitude, waypoint.point.longitude, self.bbox, self.image_width, self.image_height)
        threat.center.x = float(x)
        threat.center.y = float(y)
        
        req = SetThreat.Request()
        req.id = id
        req.threat = threat
        
        self.get_logger().info("Setting threat")
        #synchronous service call because ROS2 async doesn't work in callbacks
        result = self.set_threat_cli.call(req)
        self.get_logger().info(f"Threat id returned: {result.assigned_id}")
        if(id == -1): # for position adjustment later
            self.waypoint_threat_id_map[(waypoint.point.latitude, waypoint.point.longitude)] = result.assigned_id
        self.threat_ids.append(result.assigned_id)

    def single_waypoint_callback(self, msg: Waypoint) -> None:
        self.get_logger().info("Got single waypoint")
        self.waypoints.waypoints.append(msg)
        if msg.type == Waypoint.WAYPOINT_TYPE_CIRCLE_RIGHT or msg.type == Waypoint.WAYPOINT_TYPE_CIRCLE_LEFT:
            self.add_threat(msg)
        self.recalculate_path_from_waypoints()
        self.find_look_ahead()
        self.get_logger().info("Ending single waypoint callback")


    def true_wind_callback(self, msg: Wind) -> None:
        self.wind_angle_deg = msg.direction

    def buoy_position_callback(self, msg: GeoPoint) -> None:
        self.current_buoy_position = msg
    
    def find_look_ahead(self) -> None:
        grid_cell_msg = Point()
        grid_cell_msg.x = float(self.current_grid_cell[0])
        grid_cell_msg.y = float(self.current_grid_cell[1])
        self.current_grid_cell_publisher.publish(grid_cell_msg)

        if len(self.current_path.points) == 0:
            #self.get_logger().info("No lookAhead point for zero-length path")
            return
        
        base_distance = self.look_ahead_distance_meters  # Minimum look-ahead distance in meters
        speed_factor = self.look_ahead_increase_per_knot   # How much the look-ahead distance increases per knot of speed
        
        look_ahead_distance = base_distance + speed_factor * self.speed_knots
        self.get_logger().info(f"Grid path length: {len(self.current_grid_path)}")
        num_points = len(self.current_path.points) 
        for i in range(self.previous_look_ahead_index, num_points-1):
            point = self.current_path.points[i]
            distance = great_circle((self.latitude, self.longitude), (point.latitude, point.longitude)).meters
            # Check if the next point is closer. If so, we probably skipped some points. Don't target them. 
            next_is_closer = False if i>=num_points else (True if great_circle((self.latitude, self.longitude), (self.current_path.points[i+1].latitude, self.current_path.points[i+1].longitude)).meters<distance else False)
            self.get_logger().info(f"next_is_closer: {next_is_closer}")
            if(not next_is_closer):
                self.previous_look_ahead_index = i
                self.get_logger().info(f"Calulated current point: {point.latitude}, {point.longitude}")
                self.target_position_publisher.publish(point) # In this version, this is just for display in the UI. This is NOT an input to heading_controller_vf 
                segment = PathSegment()
                segment.start = self.current_grid_path[i]
                segment.end = self.current_grid_path[i+1]
                self.current_grid_segment_publisher.publish(segment)
                geoSegment = GeoPathSegment()
                geoSegment.start = self.current_path.points[i]
                geoSegment.end = self.current_path.points[i+1]
                self.current_segment_debug_publisher.publish(geoSegment)

                # for j, segment_endpoint_index in enumerate(self.segment_endpoint_indices):
                #     self.get_logger().info(f"Endpoint index: {segment_endpoint_index}")
                #     # Whichever endpoint index is greater than the current i, we are between that and the previous index
                #     if(segment_endpoint_index>i):
                #         segment = PathSegment()
                #         segment.start = self.current_grid_path[self.segment_endpoint_indices[j-1]]
                #         segment.end = self.current_grid_path[segment_endpoint_index]
                #         self.current_grid_segment_publisher.publish(segment)
                #         geoSegment = GeoPathSegment()
                #         geoSegment.start = self.current_path.points[self.segment_endpoint_indices[j-1]]
                #         geoSegment.end = self.current_path.points[segment_endpoint_index]
                #         self.current_segment_debug_publisher.publish(geoSegment)
                #         break

                #self.target_position_publisher.publish(point)
                return
            else:
                #remove waypoints if we've passed the last point in their segment
                if(point.latitude == self.waypoint_segment_last_coords[0].latitude and point.longitude == self.waypoint_segment_last_coords.longitude):
                    self.waypoints.waypoints.pop(0)
                    self.waypoint_segment_last_coords.pop(0)
            #     if(i==self.waypoint_indices[0]):
            #         self.get_logger().info(f"Num waypoint indices: {len(self.waypoint_indices)}, num waypoints: {len(self.waypoints.waypoints)}")
            #         self.waypoint_indices.pop(0)
            #         self.waypoints.waypoints.pop(0)


    # def find_look_ahead(self):
    #     if len(self.current_path.points) == 0:
    #         #self.get_logger().info("No lookAhead point for zero-length path")
    #         return
    #     look_ahead_point = self.find_look_ahead_point(self.current_path.points, (self.latitude, self.longitude), self.speed_knots)
    #     self.get_logger().info(f"Calulated lookAhead point: {look_ahead_point.latitude}, {look_ahead_point.longitude}")
    #     self.target_position_publisher.publish(look_ahead_point)

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

    def latlong_to_grid_proj(self, latitude, longitude, bbox, image_width, image_height, src_proj='EPSG:4326', dest_proj='EPSG:3857') -> Tuple[int, int]:
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
        #self.get_logger().info(f"lat_pct: {lat_pct}, long_pct: {long_pct}")
        
        # Convert percentages to pixel positions
        x = int(long_pct * image_width)
        y = int(lat_pct * image_height)
        
        return x, y

    def grid_to_latlong_proj(self, x, y, bbox, image_width, image_height, src_proj='EPSG:4326', dest_proj='EPSG:3857') -> Tuple[float, float]:
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

        lat_res =  abs(bbox['north']-bbox['south'])/image_height
        long_res = abs(bbox['east']-bbox['west'])/image_width
        self.get_logger().info(f"Lat res: {lat_res}")
        self.get_logger().info(f"Long res: {long_res}")

        # Calculate the geographical coordinates from the pixel positions
        long_pct = x / image_width
        lat_pct = 1.0-(y / image_height)
        
        # Interpolate the latitude and longitude within the bounding box
        latitude = (north_east[0] - lat_pct * (north_east[0] - south_west[0]))+(lat_res/2)
        longitude = (south_west[1] + long_pct * (north_east[1] - south_west[1]))+(long_res/2)
        
        return latitude, longitude
    
    def insert_intermediate_points(self, path, num_per_unit_distance) -> List[PoseStamped]:
        length = len(path)
        if(length  == 0):
            self.get_logger().warn("Called insert_intermediate_points on a zero length segment!")
            return path
        appended = []
        for i in range(length):
            if i<(length-1):
                num = round(distance(path[i].pose.position.x, path[i].pose.position.y, path[i+1].pose.position.x, path[i+1].pose.position.y)*num_per_unit_distance)
                self.get_logger().info(f"Num to insert: {num}")
                appended.append(path[i])
                x_step = (path[i+1].pose.position.x-path[i].pose.position.x)/(num+1)
                self.get_logger().info(f"X step: {x_step}")
                y_step = (path[i+1].pose.position.y-path[i].pose.position.y)/(num+1)
                self.get_logger().info(f"y step: {y_step}")
                for j in range(1, num+1):
                    new_x = path[i].pose.position.x+x_step*j
                    new_y = path[i].pose.position.y+y_step*j
                    new_point = PoseStamped()
                    new_point.pose.position.x = new_x
                    new_point.pose.position.y = new_y
                    appended.append(new_point)
        appended.append(path[-1])
        return appended



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
        trace = traceback.format_exc()
        path_follower.get_logger().fatal(f'Unhandled exception: {e}\n{trace}')
    finally:
        # Shutdown and cleanup the node
        executor.shutdown()
        path_follower.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()