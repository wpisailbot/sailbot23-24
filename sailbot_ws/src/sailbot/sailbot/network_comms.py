#!/usr/bin/env python3
import sys
import rclpy
from typing import Optional
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.subscription import Subscription
from std_msgs.msg import (
    String,
    Int8,
    Int16,
    Empty,
    Float64,
)
from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.msg import State as StateMsg
from sensor_msgs.msg import NavSatFix, Image
from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import OccupancyGrid
from ament_index_python.packages import get_package_share_directory
from sailbot_msgs.msg import (
    Wind,
    GeoPath,
    AutonomousMode,
    TrimState,
    Waypoint,
    WaypointPath,
    GeoPathSegment,
    BuoyDetectionStamped,
    CVParameters,
    HSVBounds,
    BuoyTypeInfo,
    AnnotatedImage
)
import grpc
from concurrent import futures
import json
import math
import time
import os
import numpy as np
import cv2
from cv_bridge import CvBridge
import re
import numpy as np
import traceback
import types
from typing import Callable, Any
import signal

from sailbot_msgs.srv import RestartNode

import telemetry_messages.python.boat_state_pb2 as boat_state_pb2
import telemetry_messages.python.boat_state_pb2_grpc as boat_state_pb2_rpc
import telemetry_messages.python.control_pb2 as control_pb2
import telemetry_messages.python.control_pb2_grpc as control_pb2_grpc
import telemetry_messages.python.node_restart_pb2 as node_restart_pb2
import telemetry_messages.python.node_restart_pb2_grpc as node_restart_pb2_grpc
import telemetry_messages.python.video_pb2 as video_pb2
import telemetry_messages.python.video_pb2_grpc as video_pb2_grpc

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
            image = cv2.flip(image, 0)
            img_rgba = cv2.cvtColor(image, cv2.COLOR_GRAY2BGRA)
            alpha_channel = np.ones(image.shape, dtype=np.uint8) * 127
            alpha_channel[image == 255] = 0 
            img_rgba[:, :, 3] = alpha_channel
            if image is None:
                raise ValueError(f"Unable to load image at {image_path}")

            return img_rgba, {"south": south, "west": west, "north": north, "east": east}

    # Return None if no matching file is found
    return None, None

def get_resource_dir():
    package_path = get_package_share_directory('sailbot')
    resource_path = os.path.join(package_path, 'maps')
    return resource_path

def encode_frame(frame):
    # Convert the frame to JPEG
    result, encoded_image = cv2.imencode('.jpg', frame)
    if result:
        return encoded_image.tobytes()
    else:
        return None

def make_json_string(json_msg):
    json_str = json.dumps(json_msg)
    message = String()
    message.data = json_str
    return message

def get_state(state_id: int):
    if state_id == StateMsg.PRIMARY_STATE_ACTIVE:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_ACTIVE
    if state_id == StateMsg.PRIMARY_STATE_INACTIVE:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_INACTIVE
    if state_id == StateMsg.PRIMARY_STATE_FINALIZED:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_FINALIZED
    if state_id == StateMsg.PRIMARY_STATE_UNCONFIGURED:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_UNCONFIGURED
    if state_id == StateMsg.PRIMARY_STATE_UNKNOWN:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_UNKNOWN
    if state_id == StateMsg.TRANSITION_STATE_ACTIVATING:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_ACTIVATING
    if state_id == StateMsg.TRANSITION_STATE_CLEANINGUP:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_CLEANING_UP
    if state_id == StateMsg.TRANSITION_STATE_CONFIGURING:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_CONFIGURING
    if state_id == StateMsg.TRANSITION_STATE_DEACTIVATING:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_DEACTIVATING
    if state_id == StateMsg.TRANSITION_STATE_ERRORPROCESSING:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_ERROR_PROCESSING
    if state_id == StateMsg.TRANSITION_STATE_SHUTTINGDOWN:
        return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_SHUTTINGDOWN
    return boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_UNKNOWN

class NetworkComms(LifecycleNode):
    """
    A ROS2 lifecycle node that handles network communications for various telemetry and control tasks
    for the boat. It handles command executions and telemetry data streaming over gRPC.

    :ivar current_map: An 'OccupancyGrid' object representing the current map used for navigation.
    :ivar current_boat_state: A protobuf message holding the current state of the boat.
    :ivar current_video_source: A string representing the current requested video source

    **Subscriptions**:

    - Multiple subscriptions for telemetry data such as position, heading, wind conditions, and video streams.

    **Publishers**:

    - Publishers for various control commands like rudder angle, ballast position, and trim tab angle.

    **Services**:

    - Service clients and servers for managing commands related to boat state, video streaming, and node restarts.

    **Methods**:

    - Various callback methods for handling telemetry data.
    - Methods for sending commands to control systems based on remote or autonomous inputs.

    **Usage**:

    - The node must be managed by state_manager

    **Notes**:
    
    - This node runs before all other lifecycle nodes, so that it can monitor their state transitions.
    - It utilizes gRPC for robust, efficient, and flexible network communication.

    """

    current_map: OccupancyGrid = None
    current_boat_state = boat_state_pb2.BoatState()
    current_cv_parameters = None
    last_camera_frame = None
    last_camera_frame_shape = None
    last_camera_frame_time = time.time()
    do_video_encode = False
    current_video_source = ""
    current_buoy_positions = {}
    current_buoy_times = {}
    available_video_sources = set()

    def __init__(self):
        super().__init__('network_comms')
        self.rudder_control_publisher: Optional[Publisher]
        self.ballast_position_publisher: Optional[Publisher]
        self.trim_tab_control_publisher: Optional[Publisher]
        self.trim_tab_angle_publisher: Optional[Publisher]
        self.autonomous_mode_publisher: Optional[Publisher]
        self.waypoints_publisher: Optional[Publisher]
        self.single_waypoint_publisher: Optional[Publisher]
        self.cv_parameters_publisher: Optional[Publisher]

        self.rot_subscription: Optional[Subscription]
        self.navsat_subscription: Optional[Subscription]
        self.track_degrees_true_subscription: Optional[Subscription]
        self.track_degrees_magnetic_subscription: Optional[Subscription]
        self.speed_knots_subscription: Optional[Subscription]
        self.speed_kmh_subscription: Optional[Subscription]
        self.heading_subscription: Optional[Subscription]
        self.true_wind_subscription: Optional[Subscription]
        self.apparent_wind_subscription: Optional[Subscription]
        self.roll_subscription: Optional[Subscription]
        self.pitch_subscription: Optional[Subscription]
        self.pwm_heartbeat_subscription: Optional[Subscription]
        self.control_system_subscription: Optional[Subscription]
        self.current_path_subscription: Optional[Subscription]
        self.target_position_subscriber: Optional[Subscription]
        self.trim_state_subscriber: Optional[Subscription]
        self.camera_image_subscriber: Optional[Subscription]

        #receives state updates from other nodes
        self.airmar_reader_lifecycle_state_subscriber: Optional[Subscription]

        self.callback_group_state = MutuallyExclusiveCallbackGroup()
        
        self.set_parameters()
        self.get_parameters()

        self.declare_parameter('map_name', 'quinsigamond')
        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value
        self.get_logger().info(f'Map name: {self.map_name}')
        self.get_logger().info("Getting map image")
        self.map_image, self.bbox = find_and_load_image(get_resource_dir(), self.map_name)
        self.bridge = CvBridge()

    def set_parameters(self) -> None:
        self.declare_parameter('sailbot.cv.buoy_circularity_threshold', 0.6)

    def get_parameters(self) -> None:
        self.circularity_threshold = self.get_parameter('sailbot.cv.buoy_circularity_threshold').get_parameter_value().double_value
        
    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")
        self.pwm_control_publisher = self.create_lifecycle_publisher(String, 'pwm_control', 10)
        self.rudder_control_publisher = self.create_lifecycle_publisher(Int16, 'rudder_angle', 10)

        self.ballast_position_publisher = self.create_lifecycle_publisher(Float64, 'ballast_position', 10)
        self.trim_tab_control_publisher = self.create_lifecycle_publisher(Int8, 'tt_control', 10)
        self.trim_tab_angle_publisher = self.create_lifecycle_publisher(Int16, 'tt_angle', 10)

        self.waypoints_publisher = self.create_lifecycle_publisher(WaypointPath, 'waypoints', 10)
        self.single_waypoint_publisher = self.create_lifecycle_publisher(Waypoint, 'single_waypoint', 10)


        self.autonomous_mode_publisher = self.create_lifecycle_publisher(AutonomousMode, 'autonomous_mode', 10)

        self.vf_forward_magnitude_publisher = self.create_lifecycle_publisher(Float64, 'vf_forward_magnitude', 10)
        self.rudder_adjustment_scale_publisher = self.create_lifecycle_publisher(Float64, 'rudder_adjustment_scale', 10)
        self.rudder_overshoot_bias_publisher = self.create_lifecycle_publisher(Float64, 'rudder_overshoot_bias', 10)

        self.request_tack_publisher = self.create_lifecycle_publisher(Empty, 'request_tack', 10)

        self.cv_parameters_publisher = self.create_lifecycle_publisher(CVParameters, 'cv_parameters', 10)

        self.rot_subscription = self.create_subscription(
            Float64,
            'airmar_data/rate_of_turn',
            self.rate_of_turn_callback,
            10)
        
        self.navsat_subscription = self.create_subscription(
            NavSatFix,
            'airmar_data/lat_long',
            self.lat_long_callback,
            10)
        
        self.track_degrees_true_subscription = self.create_subscription(
            Float64,
            'airmar_data/track_degrees_true',
            self.track_degrees_true_callback,
            10)
        
        self.track_degrees_magnetic_subscription = self.create_subscription(
            Float64,
            'airmar_data/track_degrees_magnetic',
            self.track_degrees_magnetic_callback,
            10)
        
        self.speed_knots_subscription = self.create_subscription(
            Float64,
            'airmar_data/speed_knots',
            self.speed_knots_callback,
            10)
        
        self.speed_kmh_subscription = self.create_subscription(
            Float64,
            'airmar_data/speed_kmh',
            self.speed_kmh_callback,
            10)
        
        self.heading_subscription = self.create_subscription(
            Float64,
            'heading',
            self.heading_callback,
            10)
        
        self.true_wind_subscription = self.create_subscription(
            Wind,
            'true_wind_smoothed',
            self.true_wind_callback,
            10)
        
        self.apparent_wind_subscription = self.create_subscription(
            Wind,
            'apparent_wind_smoothed',
            self.apparent_wind_callback,
            10)
        
        self.roll_subscription = self.create_subscription(
            Float64,
            'airmar_data/roll',
            self.roll_callback,
            10)
        
        self.pitch_subscription = self.create_subscription(
            Float64,
            'airmar_data/pitch',
            self.pitch_callback,
            10)

        self.current_path_subscription = self.create_subscription(
            GeoPath,
            'current_path',
            self.current_path_callback,
            10)
        self.target_position_subscriber = self.create_subscription(
            GeoPoint,
            'target_position',
            self.target_position_callback,
            10)
        self.trim_state_subscriber = self.create_subscription(
            TrimState,
            'trim_state',
            self.trim_state_callback,
            10)
        self.camera_color_image_subscriber = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.camera_color_image_callback,
            10)
        self.camera_depth_image_subscriber = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.camera_depth_image_callback,
            10)
        self.camera_mask_image_subscriber = self.create_subscription(
            AnnotatedImage,
            'cv_mask',
            self.camera_mask_image_callback,
            10)
        self.buoy_position_subscriber = self.create_subscription(
            BuoyDetectionStamped,
            'buoy_position',
            self.buoy_position_callback,
            10)
        self.rudder_angle_subscriber = self.create_subscription(
            Int16,
            'rudder_angle',
            self.rudder_angle_callback,
            10
        )
        self.path_segment_debug_subscriber = self.create_subscription(
            GeoPathSegment,
            'current_segment_debug',
            self.path_segment_debug_callback,
            10
        )
        self.target_heading_debug_subscriber = self.create_subscription(
            Float64,
            'target_heading',
            self.target_heading_debug_callback,
            10
        )
        self.target_track_debug_subscriber = self.create_subscription(
            Float64,
            'target_track',
            self.target_track_debug_callback,
            10
        )
        self.initial_cv_parameters_subscriber = self.create_subscription(
            CVParameters,
            'initial_cv_parameters',
            self.initial_cv_parameters_callback,
            10
        )
        self.restart_node_client = self.create_client(RestartNode, 'state_manager/restart_node', callback_group=self.callback_group_state)
        #initial dummy values, for testing
        # self.current_boat_state.latitude = 42.273822
        # self.current_boat_state.longitude = -71.805967
        # self.current_boat_state.latitude = 42.276842
        # self.current_boat_state.longitude = -71.756035
        self.current_boat_state.latitude = 42.0396766107111
        self.current_boat_state.longitude = -71.84585650616927
        self.current_boat_state.current_heading = 0
        self.current_boat_state.track_degrees_true = 0
        self.current_boat_state.track_degrees_magnetic = 0
        self.current_boat_state.speed_knots = 0
        self.current_boat_state.speed_kmh = 0
        self.current_boat_state.rate_of_turn = 0
        self.current_boat_state.true_wind.speed = 0
        self.current_boat_state.true_wind.direction = 270.0
        self.current_boat_state.apparent_wind.speed = 0
        self.current_boat_state.apparent_wind.direction = 0
        self.current_boat_state.pitch = 0
        self.current_boat_state.roll = 0
        self.current_boat_state.has_current_path_segment = False
        self.node_indices = {}
        self.declare_parameter('managed_nodes')
        node_names = self.get_parameter('managed_nodes').get_parameter_value().string_array_value
        self.get_logger().info(f'Active nodes: {node_names}')
        i=0
        for name in node_names:

            #init state
            node_info = boat_state_pb2.NodeInfo()
            node_info.name = name
            node_info.status = boat_state_pb2.NodeStatus.NODE_STATUS_OK
            node_info.info = ""
            self.current_boat_state.node_states.append(node_info)
            self.node_indices[name]=i
            i+=1

            #create lifecycle callback using function generation
            try:
                self.setup_node_subscriptions(node_name=name)
            except Exception as e:
                trace = traceback.format_exc()
                self.get_logger().fatal(f'Unhandled exception: {e}\n{trace}')

        self.current_boat_state.current_autonomous_mode = boat_state_pb2.AutonomousMode.AUTONOMOUS_MODE_NONE
        # a = boat_state_pb2.Point()
        # a.latitude = 5.1
        # a.longitude = 4.1
        # b=boat_state_pb2.Point()
        # b.latitude = 5.2
        # b.longitude = 4.1
        # self.current_boat_state.current_path.points.append(a)
        # self.current_boat_state.current_path.points.append(b)
        # c = boat_state_pb2.Point()
        # c.latitude = 4.9
        # c.longitude = 3.9
        # d=boat_state_pb2.Point()
        # d.latitude = 4.8
        # d.longitude = 3.9
        # e=boat_state_pb2.Point()
        # e.latitude = 4.7
        # e.longitude = 3.8
        # self.current_boat_state.previous_positions.points.append(c)
        # self.current_boat_state.previous_positions.points.append(d)
        # self.current_boat_state.previous_positions.points.append(e)
        
        self.last_pwm_heartbeat = -1
        self.last_ctrl_heartbeat = -1
        self.last_tt_heartbeat = -1
        try:
            self.create_grpc_server()
        except Exception as e:
            trace = traceback.format_exc()
            self.get_logger().fatal(f'Unhandled exception: {e}\n{trace}')

        self.pwm_heartbeat_subscription = self.create_subscription(
            Empty,
            'heartbeat/pwm_controler',
            self.pwm_controller_heartbeat,
            1)
        
        self.control_system_heartbeat_subscription = self.create_subscription(
            Empty,
            'heartbeat/control_system',
            self.control_system_heartbeat,
            1)
        
        self.trim_tab_heartbeat_subscription = self.create_subscription(
            Empty,
            'heartbeat/trim_tab_comms',
            self.trim_tab_comms_heartbeat,
            1)
        
        self.buoy_cleanup_timer = self.create_timer(1.0, self.remove_old_buoys)

        return super().on_configure(state)

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")
        # Start publishers or timers
        #self.ballast_position_publisher.on_activate()
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")
        # Destroy subscribers, publishers, and timers
        self.destroy_lifecycle_publisher(self.pwm_control_publisher)
        self.destroy_lifecycle_publisher(self.ballast_position_publisher)
        self.destroy_lifecycle_publisher(self.trim_tab_control_publisher)
        self.destroy_lifecycle_publisher(self.trim_tab_angle_publisher)
        self.destroy_lifecycle_publisher(self.autonomous_mode_publisher)

        self.destroy_subscription(self.rot_subscription)
        self.destroy_subscription(self.navsat_subscription)
        self.destroy_subscription(self.track_degrees_true_subscription)
        self.destroy_subscription(self.track_degrees_magnetic_subscription)
        self.destroy_subscription(self.speed_knots_subscription)
        self.destroy_subscription(self.speed_kmh_subscription)
        self.destroy_subscription(self.heading_subscription)
        self.destroy_subscription(self.true_wind_subscription)
        self.destroy_subscription(self.apparent_wind_subscription)
        self.destroy_subscription(self.roll_subscription)
        self.destroy_subscription(self.pitch_subscription)
        self.destroy_subscription(self.pwm_heartbeat_subscription)
        self.destroy_subscription(self.control_system_heartbeat_subscription)
        self.destroy_subscription(self.trim_tab_heartbeat_subscription)
        self.destroy_subscription(self.trim_state_subscriber)

        #return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        # Perform final cleanup if necessary
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().error("Error caught!")
        return super().on_error(state)
     
    #end lifecycle callbacks

    def target_position_callback(self, msg: GeoPoint) -> None:
        #self.get_logger().info(f"Sending target point: {msg}")
        self.current_boat_state.current_target_point.latitude = msg.latitude
        self.current_boat_state.current_target_point.longitude = msg.longitude

    def trim_state_callback(self, msg: TrimState) -> None:
        if(msg.state == TrimState.TRIM_STATE_MIN_LIFT):
            self.current_boat_state.current_trim_state = boat_state_pb2.TrimState.TRIM_STATE_MIN_LIFT
        elif(msg.state == TrimState.TRIM_STATE_MAX_LIFT_PORT):
            self.current_boat_state.current_trim_state = boat_state_pb2.TrimState.TRIM_STATE_MAX_LIFT_PORT
        elif(msg.state == TrimState.TRIM_STATE_MAX_LIFT_STARBOARD):
            self.current_boat_state.current_trim_state = boat_state_pb2.TrimState.TRIM_STATE_MAX_LIFT_STARBOARD
        elif(msg.state == TrimState.TRIM_STATE_MAX_DRAG_PORT):
            self.current_boat_state.current_trim_state = boat_state_pb2.TrimState.TRIM_STATE_MAX_DRAG_PORT
        elif(msg.state == TrimState.TRIM_STATE_MAX_DRAG_STARBOARD):
            self.current_boat_state.current_trim_state = boat_state_pb2.TrimState.TRIM_STATE_MAX_DRAG_STARBOARD
        elif(msg.state == TrimState.TRIM_STATE_MANUAL):
            self.current_boat_state.current_trim_state = boat_state_pb2.TrimState.TRIM_STATE_MANUAL
        
    def current_path_callback(self, msg: GeoPath) -> None:
        #self.get_logger().info(f"Updating boat state with new path of length: {len(msg.points)}")
        self.current_boat_state.current_path.ClearField("points")# = command.new_path
        #self.get_logger().info("Cleared old path")
        for geo_point in msg.points:
            point_msg = boat_state_pb2.Point(latitude=geo_point.latitude, longitude = geo_point.longitude)
            self.current_boat_state.current_path.points.append(point_msg)
        #self.get_logger().info("Added new points")

        #self.get_logger().info(f"length of boatState path is: {len(self.current_boat_state.current_path.points)}")

    def create_lifecycle_callback(self, node_name: str) -> Callable[[Any, TransitionEvent], None]:
        def lifecycle_callback(self, msg: TransitionEvent):
            msg_details = f"Received {node_name} update! State is: {msg.goal_state.id}"
            self.get_logger().info(msg_details)
            state = get_state(msg.goal_state.id)
            if(state == boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_ERROR_PROCESSING):
                self.current_boat_state.node_states[self.node_indices[node_name]].status = boat_state_pb2.NodeStatus.NODE_STATUS_ERROR
            elif (state==boat_state_pb2.NodeLifecycleState.NODE_LIFECYCLE_STATE_UNKNOWN):
                self.current_boat_state.node_states[self.node_indices[node_name]].status = boat_state_pb2.NodeStatus.NODE_STATUS_WARN
            else:
                self.current_boat_state.node_states[self.node_indices[node_name]].status = boat_state_pb2.NodeStatus.NODE_STATUS_OK
            self.current_boat_state.node_states[self.node_indices[node_name]].lifecycle_state = state

        return lifecycle_callback
    
    def create_error_callback(self, node_name: str) -> Callable[[Any, String], None]:
        def error_callback(self, msg: String):
            msg_details = f"Received {node_name} error! String is: {msg.data}"
            self.get_logger().info(msg_details)
            self.current_boat_state.node_states[self.node_indices[node_name]].status = boat_state_pb2.NodeStatus.NODE_STATUS_ERROR
        return error_callback

    def create_and_bind_lifecycle_callback(self, node_name: str) -> None:
        callback_method_name = f"{node_name}_lifecycle_callback"
        # Dynamically create the callback method
        method = self.create_lifecycle_callback(node_name)
        # Bind the method to the instance, ensuring it receives 'self' properly
        bound_method = types.MethodType(method, self)
        # Attach the bound method to the instance
        setattr(self, callback_method_name, bound_method)
        
        # Set up the subscription using the dynamically created and bound callback
        subscription_name = f"{node_name}_lifecycle_subscription"
        topic_name = f"/{node_name}/transition_event"
        subscription = self.create_subscription(
            TransitionEvent,
            topic_name,
            getattr(self, callback_method_name),
            10)
        # Attach the subscription to this class instance
        setattr(self, subscription_name, subscription)
    
    def create_and_bind_error_callback(self, node_name: str) -> None:
        callback_method_name = f"{node_name}_error_callback"
        # Dynamically create the callback method
        method = self.create_error_callback(node_name)
        # Bind the method to the instance, ensuring it receives 'self' properly
        bound_method = types.MethodType(method, self)
        # Attach the bound method to the instance
        setattr(self, callback_method_name, bound_method)
        
        # Set up the subscription using the dynamically created and bound callback
        subscription_name = f"{node_name}_error_subscription"
        topic_name = f"/{node_name}/error"
        subscription = self.create_subscription(
            String,
            topic_name,
            getattr(self, callback_method_name),
            10)
        # Attach the subscription to this class instance
        setattr(self, subscription_name, subscription)

    def setup_node_subscriptions(self, node_name: str) -> None:
        self.create_and_bind_lifecycle_callback(node_name)
        self.create_and_bind_error_callback(node_name)

    def rate_of_turn_callback(self, msg: Float64):
        self.current_boat_state.rate_of_turn = msg.data

    def lat_long_callback(self, msg: NavSatFix):
        #self.get_logger().info(f"Got latlong: {msg.latitude}, {msg.longitude}")
        self.current_boat_state.latitude = msg.latitude
        self.current_boat_state.longitude = msg.longitude

    def track_degrees_true_callback(self, msg: Float64):
        self.current_boat_state.track_degrees_true = msg.data

    def track_degrees_magnetic_callback(self, msg: Float64):
        self.current_boat_state.track_degrees_magnetic = msg.data

    def speed_knots_callback(self, msg: Float64):
        self.current_boat_state.speed_knots = msg.data

    def speed_kmh_callback(self, msg: Float64):
        self.current_boat_state.speed_kmh = msg.data
    
    def heading_callback(self, msg: Float64):
        self.current_boat_state.current_heading = msg.data

    def true_wind_callback(self, msg: Wind):
        self.current_boat_state.true_wind.speed = msg.speed
        self.current_boat_state.true_wind.direction = msg.direction

    def apparent_wind_callback(self, msg: Wind):
        self.current_boat_state.apparent_wind.speed = msg.speed
        self.current_boat_state.apparent_wind.direction = msg.direction

    def roll_callback(self, msg: Float64):
        self.current_boat_state.roll = msg.data
    
    def pitch_callback(self, msg: Float64):
        self.current_boat_state.pitch = msg.data
    
    def decode_image(self, msg: Image):
        if msg.encoding == 'mono8':
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        elif msg.encoding == 'mono16':
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
        elif msg.encoding == 'rgb8':
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        elif msg.encoding == 'bgr8':
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        elif msg.encoding == 'bgra8':
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
        elif msg.encoding == 'rgba8':
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgba8')
        elif msg.encoding == '32FC1':
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            # Depth image has Inf and NaN in it, which breaks normalization
            valid_mask = np.isfinite(cv_image)
            if np.any(valid_mask):
                max_val = np.max(cv_image[valid_mask])
            else:
                max_val = 0.0
                self.get_logger().warn('No valid values found in the image.')

            cv_image = np.nan_to_num(cv_image, nan=max_val, posinf=max_val)

            frame = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')

        else:
            raise ValueError(f"Unsupported encoding: {msg.encoding}")
        return frame

    def encode_frame(self, frame):
        encoded_frame = cv2.imencode('.jpg', frame)[1].tobytes()
        return encoded_frame

    def set_current_image(self, msg: Image):
        current_time = time.time()
        if current_time > self.last_camera_frame_time + 0.1:
            frame = self.decode_image(msg)
            self.last_camera_frame_shape = frame.shape
            self.last_camera_frame = self.encode_frame(frame)
            self.last_camera_frame_time = current_time

    def update_video_sources(self):
        self.current_boat_state.ClearField('available_video_sources')
        for source in self.available_video_sources:
            self.current_boat_state.available_video_sources.append(source)
        
    def camera_color_image_callback(self, msg: Image):
        # if(self.do_video_encode == False):
        #     return
        if('COLOR' not in self.available_video_sources):
            self.available_video_sources.add('COLOR')
            self.update_video_sources()
        if(self.current_video_source != 'COLOR'):
            return
        self.set_current_image(msg)

    def camera_mask_image_callback(self, msg: AnnotatedImage):
        # if(self.do_video_encode == False):
        #     return
        if(msg.source not in self.available_video_sources):
            self.available_video_sources.add(msg.source)
            self.update_video_sources()
        if(self.current_video_source != msg.source):
            return
        self.set_current_image(msg.image)

    def camera_depth_image_callback(self, msg: Image):
        # if(self.do_video_encode == False):
        #     return
        if('DEPTH' not in self.available_video_sources):
            self.available_video_sources.add('DEPTH')
            self.update_video_sources()
        if(self.current_video_source != 'DEPTH'):
            return
        self.set_current_image(msg)

    def update_buoy_state(self):
        self.current_boat_state.ClearField("buoy_positions")# = command.new_path
        #self.get_logger().info("Cleared old path")
        for key in self.current_buoy_positions.keys():
            point = self.current_buoy_positions[key].position
            point_msg = boat_state_pb2.Point(latitude=point.latitude, longitude = point.longitude)
            self.current_boat_state.buoy_positions.append(point_msg)

    def buoy_position_callback(self, msg: BuoyDetectionStamped):
        self.current_buoy_positions[msg.id] = msg
        self.current_buoy_times[msg.id] = time.time()
        self.update_buoy_state()
    
    def remove_old_buoys(self):
        current_time = time.time()
        keys_to_delete = [key for key, (timestamp) in self.current_buoy_times.items() if current_time - timestamp > 3]
        for key in keys_to_delete:
            del self.current_buoy_positions[key]
            del self.current_buoy_times[key]

        self.update_buoy_state()

    def rudder_angle_callback(self, msg: Int16):
        self.current_boat_state.rudder_position = msg.data

    def path_segment_debug_callback(self, msg: GeoPathSegment):
        self.current_boat_state.has_current_path_segment = True
        self.current_boat_state.current_path_segment.start.latitude = msg.start.latitude
        self.current_boat_state.current_path_segment.start.longitude = msg.start.longitude
        self.current_boat_state.current_path_segment.end.latitude = msg.end.latitude
        self.current_boat_state.current_path_segment.end.longitude = msg.end.longitude
        
    def target_heading_debug_callback(self, msg: Float64):
        self.current_boat_state.has_target_heading = True
        self.current_boat_state.target_heading = msg.data
        #self.get_logger().info("Got target heading")

    def target_track_debug_callback(self, msg: Float64):
        self.current_boat_state.has_target_track = True
        self.current_boat_state.target_track = msg.data
        #self.get_logger().info("Got target track")
    
    def initial_cv_parameters_callback(self, msg: CVParameters):

        self.current_cv_parameters = boat_state_pb2.CVParameters()
        buoy_type: BuoyTypeInfo
        for buoy_type in msg.buoy_types:
            type_bounds: HSVBounds = buoy_type.hsv_bounds
            this_bounds = boat_state_pb2.HSVBounds(lower_h=type_bounds.lower_h/255, lower_s=type_bounds.lower_s/255, lower_v=type_bounds.lower_v/255, upper_h=type_bounds.upper_h/255, upper_s=type_bounds.upper_s/255, upper_v=type_bounds.upper_v/255)
            this_type = boat_state_pb2.BuoyTypeInfo(hsv_bounds=this_bounds, buoy_diameter=buoy_type.buoy_diameter, name=buoy_type.name)
            self.current_cv_parameters.buoy_types.append(this_type)
        
        self.current_cv_parameters.circularity_threshold = msg.circularity_threshold

        self.get_logger().info(f"Got initial CV values: {self.current_cv_parameters}")

    #new server code
    def create_grpc_server(self): 
        self.get_logger().info("Creating gRPC server")
        self.grpc_server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))

        control_pb2_grpc.add_ControlCommandServiceServicer_to_server(self, self.grpc_server)
        control_pb2_grpc.add_SetParameterServiceServicer_to_server(self, self.grpc_server)

        boat_state_pb2_rpc.add_SendBoatStateServiceServicer_to_server(self, self.grpc_server)
        boat_state_pb2_rpc.add_GetMapServiceServicer_to_server(self, self.grpc_server)
        boat_state_pb2_rpc.add_GetCVParametersServiceServicer_to_server(self, self.grpc_server)
        boat_state_pb2_rpc.add_StreamBoatStateServiceServicer_to_server(self, self.grpc_server)
        
        node_restart_pb2_grpc.add_RestartNodeServiceServicer_to_server(self, self.grpc_server)
        
        video_pb2_grpc.add_VideoStreamerServicer_to_server(self, self.grpc_server)

        #connect_pb2_grpc.add_ConnectToBoatServiceServicer_to_server(self, self.grpc_server)
        self.grpc_server.add_insecure_port('[::]:50051')
        self.grpc_server.start()
    
    def shutdown_handler(self, signum, frame):
        rclpy.logging.get_logger("network_comms").info("Received shutdown signal, shutting down...")
        self.grpc_server.stop(0)
        rclpy.shutdown()

    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def StreamVideo(self, command: video_pb2.VideoRequest, context):
        #self.do_video_encode = True
        rate = self.create_rate(10)
        self.current_video_source = command.videoSource

        try:
            while context.is_active():
                yield video_pb2.VideoFrame(data=self.last_camera_frame, width=self.last_camera_frame_shape[1], height=self.last_camera_frame_shape[0], timestamp=int(time.time()))
                rate.sleep()
        finally:
            if not context.is_active():
                #self.do_video_encode = False
                self.get_logger().info("Video stream was cancelled or client disconnected.")

    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteMarkBuoyCommand(self, command: control_pb2.MarkBuoyCommand, context):
        self.get_logger().info("Received mark buoy command")
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteRequestTackCommand(self, command: control_pb2.RequestTackCommand, context):
        self.get_logger().info("Received request tack command")
        self.request_tack_publisher.publish(Empty())


    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteRudderCommand(self, command: control_pb2.RudderCommand, context):
        #center = 75 degrees, full right=40, full left = 113
        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_SUCCESS
        #rudder commands are inverted radians, map to degrees and invert
        degrees = command.rudder_control_value*(180/math.pi)*-1
        self.get_logger().info(f"degrees: {degrees}")
        
        msg = Int16()
        msg.data = int(degrees)
        self.rudder_control_publisher.publish(msg)
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteTrimTabCommand(self, command: control_pb2.TrimTabCommand, context):
        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_SUCCESS
        state_msg = Int8()
        state_msg.data = 5
        angle_msg = Int16()
        angle_msg.data = int((command.trimtab_control_value+math.pi/2)*180/math.pi)
        self.trim_tab_control_publisher.publish(state_msg)
        self.trim_tab_angle_publisher.publish(angle_msg)
        #self.get_logger().info("Publishing trimtab command")
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteBallastCommand(self, command: control_pb2.BallastCommand, context):
        response = control_pb2.ControlResponse()
        self.get_logger().info("Publishing ballast command")
        position_msg = Float64()
        position_msg.data = command.ballast_control_value
        self.ballast_position_publisher.publish(position_msg)

        # current_target = 80 + ((110 - 80) / (1.0 - -1.0)) * (command.ballast_control_value - -1.0)
        # ballast_json = {"channel": "12", "angle": current_target}
        # self.pwm_control_publisher.publish(make_json_string(ballast_json))

        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_SUCCESS
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteAutonomousModeCommand(self, command: control_pb2.AutonomousModeCommand, context):
        self.get_logger().info(f"Received autonomous mode command: {command.autonomous_mode}")
        msg = AutonomousMode()
        if command.autonomous_mode == boat_state_pb2.AutonomousMode.AUTONOMOUS_MODE_NONE:
            msg.mode = AutonomousMode.AUTONOMOUS_MODE_NONE
        elif command.autonomous_mode == boat_state_pb2.AutonomousMode.AUTONOMOUS_MODE_BALLAST:
            msg.mode = AutonomousMode.AUTONOMOUS_MODE_BALLAST
        elif command.autonomous_mode == boat_state_pb2.AutonomousMode.AUTONOMOUS_MODE_TRIMTAB:
            msg.mode = AutonomousMode.AUTONOMOUS_MODE_TRIMTAB
        elif command.autonomous_mode == boat_state_pb2.AutonomousMode.AUTONOMOUS_MODE_FULL:
            msg.mode = AutonomousMode.AUTONOMOUS_MODE_FULL
        self.autonomous_mode_publisher.publish(msg)
        self.current_boat_state.autonomous_mode = command.autonomous_mode
        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_SUCCESS
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteSetWaypointsCommand(self, command: control_pb2.SetWaypointsCommand, context):
        self.get_logger().info("Received waypoints command")
        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_SUCCESS
        self.current_boat_state.current_waypoints.ClearField("waypoints")# = command.new_path
        self.current_boat_state.current_waypoints.waypoints.extend(command.new_waypoints.waypoints)
        self.get_logger().info(f"Received waypoints with {len(command.new_waypoints.waypoints)} points: ")
        waypoints = WaypointPath()
        for waypoint in command.new_waypoints.waypoints:
            self.get_logger().info(str(waypoint.point.latitude)+" : "+str(waypoint.point.longitude))
            fix = Waypoint()
            fix.point.latitude = waypoint.point.latitude
            fix.point.longitude = waypoint.point.longitude
            if(waypoint.type == boat_state_pb2.WaypointType.WAYPOINT_TYPE_INTERSECT):
                fix.type = Waypoint.WAYPOINT_TYPE_INTERSECT
            elif(waypoint.type == boat_state_pb2.WaypointType.WAYPOINT_TYPE_CIRCLE_RIGHT):
                fix.type = Waypoint.WAYPOINT_TYPE_CIRCLE_RIGHT
            elif(waypoint.type == boat_state_pb2.WaypointType.WAYPOINT_TYPE_CIRCLE_LEFT):
                fix.type = Waypoint.WAYPOINT_TYPE_CIRCLE_LEFT
            waypoints.waypoints.append(fix)

        self.get_logger().info("Publishing waypoints")
        self.waypoints_publisher.publish(waypoints)

        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteAddWaypointCommand(self, command: control_pb2.AddWaypointCommand, context):
        self.get_logger().info("Received add single waypoint command")
        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_SUCCESS
        self.current_boat_state.current_waypoints.waypoints.append(command.new_waypoint)
        waypoint = Waypoint()
        waypoint.point.latitude = command.new_waypoint.point.latitude
        waypoint.point.longitude = command.new_waypoint.point.longitude
        if(command.new_waypoint.type == boat_state_pb2.WaypointType.WAYPOINT_TYPE_INTERSECT):
            waypoint.type = Waypoint.WAYPOINT_TYPE_INTERSECT
        elif(command.new_waypoint.type == boat_state_pb2.WaypointType.WAYPOINT_TYPE_CIRCLE_RIGHT):
            waypoint.type = Waypoint.WAYPOINT_TYPE_CIRCLE_RIGHT
        elif(command.new_waypoint.type == boat_state_pb2.WaypointType.WAYPOINT_TYPE_CIRCLE_LEFT):
            waypoint.type = Waypoint.WAYPOINT_TYPE_CIRCLE_LEFT

        self.get_logger().info("Publishing single waypoint")
        self.single_waypoint_publisher.publish(waypoint)
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteSetVFForwardMagnitudeCommand(self, command: control_pb2.SetVFForwardMagnitudeCommand, context):
        self.get_logger().info("Got VF forward magnitude command")

        msg = Float64()
        msg.data = command.magnitude
        self.vf_forward_magnitude_publisher.publish(msg)

        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_SUCCESS
        return response

    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteSetRudderAdjustmentScaleCommand(self, command: control_pb2.SetRudderAdjustmentScaleCommand, context):
        self.get_logger().info("Got rudder adjustment scale command")

        msg = Float64()
        msg.data = command.scale
        self.rudder_adjustment_scale_publisher.publish(msg)

        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_SUCCESS
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteSetRudderOvershootBiasCommand(self, command: control_pb2.SetRudderOvershootBiasCommand, context):
        self.get_logger().info("Got rudder overshoot bias command")

        msg = Float64()
        msg.data = command.bias
        self.rudder_overshoot_bias_publisher.publish(msg)

        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_SUCCESS
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def ExecuteSetCVParametersCommand(self, command: control_pb2.SetCVParametersCommand, context):        
        try:
            self.current_cv_parameters = command.parameters

            newParams = CVParameters()
            for buoy_type in command.parameters.buoy_types:
                this_type = BuoyTypeInfo()
                this_type.hsv_bounds.lower_h = int(buoy_type.hsv_bounds.lower_h*255)
                this_type.hsv_bounds.lower_s = int(buoy_type.hsv_bounds.lower_s*255)
                this_type.hsv_bounds.lower_v = int(buoy_type.hsv_bounds.lower_v*255)
                this_type.hsv_bounds.upper_h = int(buoy_type.hsv_bounds.upper_h*255)
                this_type.hsv_bounds.upper_s = int(buoy_type.hsv_bounds.upper_s*255)
                this_type.hsv_bounds.upper_v = int(buoy_type.hsv_bounds.upper_v*255)

                this_type.name = buoy_type.name
                this_type.buoy_diameter = buoy_type.buoy_diameter

                newParams.buoy_types.append(this_type)

            newParams.circularity_threshold = command.parameters.circularity_threshold
            self.cv_parameters_publisher.publish(newParams)
        except Exception as e:
            trace = traceback.format_exc()
            self.get_logger().error(f'Caught exception: {e}\n{trace}')
        
        response = control_pb2.ControlResponse()
        response.execution_status = control_pb2.ControlExecutionStatus.CONTROL_EXECUTION_SUCCESS
        return response

    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def GetMap(self, command: boat_state_pb2.MapRequest, context):
        self.get_logger().info("Received GetMap request")
        _, buffer = cv2.imencode('.png', self.map_image)
        #self.get_logger().info(f"Image buffer: {buffer}")
        response = boat_state_pb2.MapResponse()
        response.image_data = buffer.tobytes()
        response.north = self.bbox['north']
        response.south = self.bbox['south']
        response.east = self.bbox['east']
        response.west = self.bbox['west']
        return response
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def GetCVParameters(self, command: boat_state_pb2.GetCVParametersRequest, context):
        try:
            self.get_logger().info("Received Get CV Params request")
            response = boat_state_pb2.GetCVParametersResponse()
            if(self.current_cv_parameters is not None):
                self.current_cv_parameters.circularity_threshold = self.circularity_threshold
                response = boat_state_pb2.GetCVParametersResponse(parameters=self.current_cv_parameters)
            return response
        except Exception as e:
            trace = traceback.format_exc()
            self.get_logger().error(f'Caught exception: {e}\n{trace}')

    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def SendBoatState(self, command: boat_state_pb2.BoatStateRequest, context):
        return self.current_boat_state
    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def StreamBoatState(self, command: boat_state_pb2.BoatStateRequest, context):
        rate = self.create_rate(10)
        try:
            while context.is_active():
                yield self.current_boat_state
                rate.sleep()
        finally:
            if not context.is_active():
                self.get_logger().info("Boat state stream was cancelled or client disconnected.")

    
    #gRPC function, do not rename unless you change proto defs and recompile gRPC files
    def RestartNode(self, command: node_restart_pb2.RestartNodeRequest, context):
        self.get_logger().info("Received restart command for: "+command.node_name)
        restart_node_request = RestartNode.Request()
        restart_node_request.node_name = command.node_name

        response = node_restart_pb2.RestartNodeResponse()
        if(self.restart_node_client.wait_for_service(3) is False):
            self.get_logger().error("Client service not available for state manager!")
            response.success = False
            return response
        
        result = self.restart_node_client.call(restart_node_request)
        response.success = result.success
        self.get_logger().info("Restart node: "+str(result))
        return response
    
    def pwm_controller_heartbeat(self, message):
        self.get_logger().info("Got pwm heartbeat")
        self.last_pwm_heartbeat = time.time()

    def control_system_heartbeat(self, message):
        self.get_logger().info("Got control heartbeat")
        self.last_ctrl_heartbeat = time.time()

    def trim_tab_comms_heartbeat(self, message):
        #self.get_logger().info("Got trimtab heartbeat")
        self.last_tt_heartbeat = time.time()
    
    def update_node_status_timer_callback(self):
        current_time = time.time()
        if(current_time-self.last_pwm_heartbeat>1):
            self.current_boat_state.node_states[self.node_indices["pwm_controller"]].node_status = boat_state_pb2.NodeStatus.NODE_STATUS_ERROR
        else:
            self.current_boat_state.node_states[self.node_indices["pwm_controller"]].node_status = boat_state_pb2.NodeStatus.NODE_STATUS_OK
        
        if(current_time-self.last_ctrl_heartbeat>1):
            self.current_boat_state.node_states[self.node_indices["control_system"]].node_status = boat_state_pb2.NodeStatus.NODE_STATUS_ERROR
        else:
            self.current_boat_state.node_states[self.node_indices["control_system"]].node_status = boat_state_pb2.NodeStatus.NODE_STATUS_OK


def main(args=None):
    rclpy.init(args=args)
    network_comms = NetworkComms()
    signal.signal(signal.SIGINT, network_comms.shutdown_handler)
    signal.signal(signal.SIGTERM, network_comms.shutdown_handler)
    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(network_comms)

    try:
        # Spin the node to execute callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        trace = traceback.format_exc()
        network_comms.get_logger().fatal(f'Unhandled exception: {e}\n{trace}')
    finally:
        # Shutdown and cleanup the node
        executor.shutdown()
        network_comms.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()