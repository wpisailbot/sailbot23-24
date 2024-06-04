#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String, Float64, Int16, Empty
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
# import json
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import math
import traceback
import numpy as np
import time

from typing import Optional
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.subscription import Subscription

from sailbot_msgs.msg import AutonomousMode, GeoPath, Wind, PathSegment

PI = math.pi
TWO_PI = PI*2
#normalizes an angle
def normalRelativeAngle(angle):
    #angle %= TWO_PI >= 0 ? (angle < PI) ? angle : angle - TWO_PI : (angle >= -PI) ? angle : angle + TWO_PI
    if angle>=0:
        if angle<PI:
            pass
        else:
            angle-=TWO_PI
    else:
        if angle>=-PI:
            pass
        else:
            angle+=TWO_PI
    return angle

# Computes if a given bearing is inside a no-sail zone
def is_in_nogo(bearing_rad, wind_angle_rad, nogo_angle_rad):

    #opposite_angle = math.fmod(bearing_rad + math.pi, 2 * math.pi)

    difference = abs(wind_angle_rad - bearing_rad)
    if difference > math.pi:
        difference = 2 * math.pi - difference
    
    return difference < nogo_angle_rad

def closest_edge_heading(target_track_rad, wind_angle_rad, nogo_angle_rad):
    """
    Calculate the closest edge heading to sail along the no-sail zone.

    :param target_track: Desired track angle in radians
    :param wind_direction_deg: Wind direction in degrees
    :param wind_restriction_replan_cutoff_degrees: Wind restriction cutoff angle in degrees
    :return: Heading along the closest edge of the no-sail zone in radians
    """
    
    # Calculate the boundaries of the no-sail zone
    no_sail_zone_left_bound = wind_angle_rad - nogo_angle_rad
    no_sail_zone_right_bound = wind_angle_rad + nogo_angle_rad
    
    # Normalize the target track to the range [0, 2*pi]
    target_track = target_track_rad % (2 * math.pi)
    
    # Calculate the difference between target track and the boundaries
    diff_left = min((target_track - no_sail_zone_left_bound) % (2 * math.pi),
                    (no_sail_zone_left_bound - target_track) % (2 * math.pi))
    diff_right = min((target_track - no_sail_zone_right_bound) % (2 * math.pi),
                     (no_sail_zone_right_bound - target_track) % (2 * math.pi))
    
    # Determine which edge is closer
    if diff_left < diff_right:
        return no_sail_zone_left_bound
    else:
        return no_sail_zone_right_bound

class HeadingController(LifecycleNode):

    """
    A ROS2 Lifecycle node for controlling the heading of a robotic sailboat based on navigational data and environmental conditions. 
    This node integrates fuzzy logic for rudder angle control and uses an adaptive vector field for path following, 
    dynamically adjusting the rudder based on computed heading errors and external factors like wind direction.
    It also triggers path replanning if cross-track error and VF settings make the boat unable to converge with the path
    without going upwind.

    The node subscribes to various topics to receive updates on the boat's heading, wind conditions, and navigation path, and publishes 
    commands for rudder angle adjustments. It is capable of handling different autonomous modes, specifically focusing on full autonomous navigation.

    :ivar heading: (float) Current heading of the boat.
    :iver leeway_angle: (float) Current difference between heading and track
    :ivar current_grid_cell: (Optional[Point]) Current grid cell position of the boat.
    :ivar path_segment: (Optional[PathSegment]) Current path segment for navigation.
    :ivar wind_direction_deg: (Optional[float]) Current wind direction in degrees.
    :ivar autonomous_mode: (int) Current autonomous mode of operation.
    :ivar rudder_adjustment_scale: (float) Proportional gain for heading control.
    :ivar rudder_angle: (float) Current angle of the rudder.


    **Functions**:

    - compute_rudder_angle(self): Computes the desired rudder angle based on the current and target headings, taking into account wind conditions and possible need for tacking. Utilizes fuzzy logic to determine the appropriate rudder adjustment.
    - needs_to_tack(self, boat_heading, target_heading, wind_direction) -> bool: Determines whether a change in direction is required that involves tacking, based on the relative angles of the boat's heading, target heading, and wind direction.
    - adaptive_vector_field(self, P1, P2, x, y, k_base=1.0, lambda_base=1.0) -> np.ndarray: Calculates a navigation vector based on the boat's current position relative to a defined path segment, adjusting the vector based on proximity to the desired path.
    - vector_to_heading(self, dx, dy) -> float: Converts vector components to a navigational heading, adjusting for the coordinate system used in navigation.
    - getRotationToPointLatLong(self, current_theta, current_lat, current_long, target_lat, target_long) -> float: Computes the necessary rotation to point towards a specific latitude and longitude, given the current heading and position.

    **Subscriptions**:

    - Subscribes to topics for boat heading, wind conditions, current path segment, and autonomous mode status to dynamically adjust the boat's rudder for optimal heading control.

    **Publishers**:

    - Publishes the computed rudder angle to a designated topic for execution by the boat's steering mechanism.

    **Usage**:

    - The node must be managed by state_manager

    """

    heading = 0
    leeway_angle = 0
    #latitude = 42.273822
    #longitude = -71.805967
    current_grid_cell = None
    path_segment = None
    wind_direction_deg = 270
    autonomous_mode = 0
    rudder_angle = 0

    last_heading_error = 0

    last_rudder_time = time.time()

    rudder_adjustment_scale = 0.01
    rudder_overshoot_bias = 50000.0
    vector_field_path_dir_weight = 2.0
    
    allow_tack = True

    request_tack_timer_duration = 3.0  # seconds
    request_tack_timer: Timer = None
    this_tack_start_time = time.time()
    request_tack_override = False

    def __init__(self):
        super().__init__('heading_controller')

        self.set_parameters()
        self.get_parameters()

        self.rudder_angle_publisher: Optional[Publisher]
        self.error_publisher: Optional[Publisher]


        self.path_segment_subscription: Optional[Subscription]
        self.airmar_heading_subscription: Optional[Subscription]
        self.airmar_track_degrees_true_subscription: Optional[Subscription]
        self.current_grid_cell_subscription: Optional[Subscription]
        self.autonomous_mode_subscription: Optional[Subscription]
        self.current_path_subscription: Optional[Subscription]
        self.request_tack_subscription: Optional[Subscription]



        self.timer: Optional[Timer]
        # self.target_position = GeoPoint()
        # self.target_position.latitude = 42.273051
        # self.target_position.longitude = -71.805049

    def set_parameters(self) -> None:
        self.declare_parameter('sailbot.heading_control.rudder_adjustment_scale', 0.01)
        self.declare_parameter('sailbot.heading_control.rudder_overshoot_bias', 50000.0)
        self.declare_parameter('sailbot.heading_control.vector_field_crosstrack_weight', 1.0)
        self.declare_parameter('sailbot.heading_control.vector_field_path_dir_weight', 2.0)
        self.declare_parameter('sailbot.heading_control.leeway_correction_limit_degrees', 10.0)
        self.declare_parameter('sailbot.heading_control.wind_restriction_replan_cutoff_degrees', 30.0)
        self.declare_parameter('sailbot.heading_control.allow_tack', True)


    def get_parameters(self) -> None:
        self.rudder_adjustment_scale = self.get_parameter('sailbot.heading_control.rudder_adjustment_scale').get_parameter_value().double_value
        self.rudder_overshoot_bias = self.get_parameter('sailbot.heading_control.rudder_overshoot_bias').get_parameter_value().double_value
        self.k_base = self.get_parameter('sailbot.heading_control.vector_field_crosstrack_weight').get_parameter_value().double_value
        self.lambda_base = self.get_parameter('sailbot.heading_control.vector_field_path_dir_weight').get_parameter_value().double_value
        self.leeway_correction_limit = self.get_parameter('sailbot.heading_control.leeway_correction_limit_degrees').get_parameter_value().double_value
        self.wind_restriction_replan_cutoff_degrees = self.get_parameter('sailbot.heading_control.wind_restriction_replan_cutoff_degrees').get_parameter_value().double_value
        self.allow_tack = self.get_parameter('sailbot.heading_control.allow_tack').get_parameter_value().bool_value
        self.original_allow_tack = self.allow_tack
        
    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")

        self.ballast_position_publisher = self.create_lifecycle_publisher(Float64, 'ballast_position', 10)

        self.rudder_angle_publisher = self.create_lifecycle_publisher(Int16, 'rudder_angle', 10)

        self.target_track_debug_publisher = self.create_lifecycle_publisher(Float64, 'target_track', 10)

        self.target_heading_debug_publisher = self.create_lifecycle_publisher(Float64, 'target_heading', 10)

        self.request_replan_publisher = self.create_lifecycle_publisher(Empty, 'request_replan', 10)

        self.request_tack_publisher = self.create_lifecycle_publisher(Empty, 'request_tack', 10)

        self.error_publisher = self.create_lifecycle_publisher(String, f'{self.get_name()}/error', 10)

        self.path_Segment_subscription = self.create_subscription(
            PathSegment,
            'current_path_segment',
            self.path_segment_callback,
            10)
        self.airmar_heading_subscription = self.create_subscription(
            Float64,
            'heading',
            self.airmar_heading_callback,
            10)
        self.airmar_track_degrees_true_subscription = self.create_subscription(
            Float64,
            '/airmar_data/track_degrees_true',
            self.airmar_track_degrees_true_callback,
            10)
        self.current_grid_cell_subscription = self.create_subscription(
            Point,
            'current_grid_cell',
            self.current_grid_cell_callback,
            10)
        self.current_path_subscription = self.create_subscription(
            GeoPath,
            'current_path',
            self.current_path_callback,
            10)
        self.true_wind_subscription = self.create_subscription(
            Wind,
            'true_wind_smoothed',
            self.true_wind_callback,
            10)
        self.vf_forward_magnitude_subscription = self.create_subscription(
            Float64,
            'vf_forward_magnitude',
            self.forward_magnitude_callback,
            10)
        self.rudder_adjustment_scale_subscription = self.create_subscription(
            Float64,
            'rudder_adjustment_scale',
            self.rudder_adjustment_scale_callback,
            10)
        self.rudder_overshoot_bias_subscription = self.create_subscription(
            Float64,
            'rudder_overshoot_bias',
            self.rudder_overshoot_bias_callback,
            10)
        self.autonomous_mode_subscription = self.create_subscription(AutonomousMode, 'autonomous_mode', self.autonomous_mode_callback, 10)

        self.request_tack_subscription = self.create_subscription(
            Empty,
            'request_tack',
            self.request_tack_callback,
            10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Heading controller node configured")

        heading_error = ctrl.Antecedent(np.arange(-180, 181, 1), 'heading_error')
        rate_of_change = ctrl.Antecedent(np.arange(-90, 91, 1), 'rate_of_change')
        rudder_adjustment = ctrl.Consequent(np.arange(-90, 91, 1), 'rudder_adjustment')

        #Negative Large (NL), Negative Medium (NM), Zero (ZE), Positive Medium (PM), and Positive Large (PL)
        # Membership functions for heading error
        heading_error['NL'] = fuzz.gaussmf(heading_error.universe, -180, 40)  # mean = -180, std = 30
        heading_error['NM'] = fuzz.gaussmf(heading_error.universe, -90, 40)  # mean = -100, std = 30
        heading_error['ZE'] = fuzz.gaussmf(heading_error.universe, 0, 40)     # mean = 0, std = 30
        heading_error['PM'] = fuzz.gaussmf(heading_error.universe, 90, 40)   # mean = 100, std = 30
        heading_error['PL'] = fuzz.gaussmf(heading_error.universe, 180, 40) 

        # Membership functions for rate of change
        rate_of_change['Large Negative'] = fuzz.zmf(rate_of_change.universe, -90, -30)
        rate_of_change['Small Negative'] = fuzz.gaussmf(rate_of_change.universe, -45, 30)
        rate_of_change['Zero'] = fuzz.pimf(rate_of_change.universe, -45, -10, 10, 45)
        rate_of_change['Small Positive'] = fuzz.gaussmf(rate_of_change.universe, 45, 30)
        rate_of_change['Large Positive'] = fuzz.smf(rate_of_change.universe, 30, 90)

        # Membership functions for rudder angle
        rudder_adjustment['Large Negative'] = fuzz.gaussmf(rudder_adjustment.universe, 90, 10)  # mean = -45, std = 10
        rudder_adjustment['Small Negative'] = fuzz.gaussmf(rudder_adjustment.universe, 60, 10)  # mean = -15, std = 10
        rudder_adjustment['Zero'] = fuzz.gaussmf(rudder_adjustment.universe, 0, 10)              # mean = 0, std = 10
        rudder_adjustment['Small Positive'] = fuzz.gaussmf(rudder_adjustment.universe, -60, 10)   # mean = 15, std = 10
        rudder_adjustment['Large Positive'] = fuzz.gaussmf(rudder_adjustment.universe, -90, 10)   # mean = 45, std = 10

        # Define the rules
        rule1 = ctrl.Rule(heading_error['ZE'] & rate_of_change['Large Negative'], rudder_adjustment['Large Positive'])
        rule2 = ctrl.Rule(heading_error['ZE'] & rate_of_change['Small Negative'], rudder_adjustment['Small Positive'])
        rule3 = ctrl.Rule(heading_error['ZE'] & rate_of_change['Zero'], rudder_adjustment['Zero'])
        rule4 = ctrl.Rule(heading_error['ZE'] & rate_of_change['Small Positive'], rudder_adjustment['Small Negative'])
        rule5 = ctrl.Rule(heading_error['ZE'] & rate_of_change['Large Positive'], rudder_adjustment['Large Negative'])

        rule6 = ctrl.Rule(heading_error['PM'] & rate_of_change['Large Negative'], rudder_adjustment['Small Positive'])
        rule7 = ctrl.Rule(heading_error['PM'] & rate_of_change['Small Negative'], rudder_adjustment['Zero'])
        rule8 = ctrl.Rule(heading_error['PM'] & rate_of_change['Zero'], rudder_adjustment['Small Negative'])
        rule9 = ctrl.Rule(heading_error['PM'] & rate_of_change['Small Positive'], rudder_adjustment['Large Negative'])
        rule10 = ctrl.Rule(heading_error['PM'] & rate_of_change['Large Positive'], rudder_adjustment['Large Negative'])

        rule11 = ctrl.Rule(heading_error['PL'], rudder_adjustment['Large Negative'])

        rule12 = ctrl.Rule(heading_error['NM'] & rate_of_change['Large Positive'], rudder_adjustment['Small Negative'])
        rule13 = ctrl.Rule(heading_error['NM'] & rate_of_change['Small Positive'], rudder_adjustment['Zero'])
        rule14 = ctrl.Rule(heading_error['NM'] & rate_of_change['Zero'], rudder_adjustment['Small Positive'])
        rule15 = ctrl.Rule(heading_error['NM'] & rate_of_change['Small Negative'], rudder_adjustment['Large Positive'])
        rule16 = ctrl.Rule(heading_error['NM'] & rate_of_change['Large Negative'], rudder_adjustment['Large Positive'])

        rule17 = ctrl.Rule(heading_error['NL'], rudder_adjustment['Large Positive'])

        self.rudder_control = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15, rule16, rule17])
        self.rudder_simulator = ctrl.ControlSystemSimulation(self.rudder_control)
        #super().on_configure(state)
        return TransitionCallbackReturn.SUCCESS

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
        self.destroy_lifecycle_publisher(self.rudder_angle_publisher)
        self.destroy_subscription(self.airmar_heading_subscription)
        self.destroy_subscription(self.path_segment_subscription)
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
    def request_tack_callback(self, msg: Empty) -> None:

        if(self.request_tack_override is False):
            self.this_tack_start_time = time.time()

        if(time.time()-self.this_tack_start_time > 20.0):
            self.allow_tack = False

        self.request_tack_override = True
        if self.request_tack_timer is not None:
            self.request_tack_timer.cancel()
        self.request_tack_timer = self.create_timer(self.request_tack_timer_duration, self.request_tack_timer_callback)
    
    def request_tack_timer_callback(self):
        self.allow_tack = self.original_allow_tack
        self.request_tack_override = False
        self.get_logger().info('Tack timer expired.')

        # Cancel the timer to clean up
        if self.request_tack_timer is not None:
            self.request_tack_timer.cancel()
            self.request_tack_timer = None
    
    def current_path_callback(self, msg: GeoPath) -> None:
        if len(msg.points) == 0:
            self.path_segment = None

    def autonomous_mode_callback(self, msg: AutonomousMode) -> None:
        self.get_logger().info(f"Got autonomous mode: {msg.mode}")
        self.autonomous_mode = msg.mode
        self.rudder_angle = 0 # Reset rudder angle when auto mode changes

        #if we're going into a manual rudder mode, reset it to 0 first
        if(msg.mode == AutonomousMode.AUTONOMOUS_MODE_NONE or msg.mode == AutonomousMode.AUTONOMOUS_MODE_TRIMTAB):
            self.get_logger().info("Resetting rudder angle")
            msg = Int16()
            msg.data = int(0)
            self.rudder_angle_publisher.publish(msg)

    def timer_callback(self) -> None:
        self.compute_rudder_angle()

    def airmar_heading_callback(self, msg: Float64) -> None:
        self.heading = msg.data
        #self.compute_rudder_angle()

    def magnetic_field_callback(self, msg):
        # Extract magnetic field components
        x = msg.magnetic_field.x
        y = msg.magnetic_field.y

        # Compute heading
        heading = math.atan2(y, x)
        
        # Convert heading to degrees
        heading_degrees = math.degrees(heading)
        
        # Normalize heading to [0, 360) degrees
        if heading_degrees < 0:
            heading_degrees += 360

        self.heading = heading_degrees

        self.get_logger().info("Heading: {:.2f} degrees".format(heading_degrees))

    def airmar_track_degrees_true_callback(self, msg: Float64) -> None:
        difference = msg.data - self.heading
        difference = (difference + 180) % 360 - 180
        if difference == -180 and msg.data > self.heading:
            difference = 180
        
        self.leeway_angle = difference
    
    def airmar_position_callback(self, msg: NavSatFix) -> None:
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        #self.compute_rudder_angle()

    def current_grid_cell_callback(self, msg: Point) -> None:
        self.current_grid_cell = msg
        #self.get_logger().info("Got new grid cell!")
        #self.compute_rudder_angle()
    
    def path_segment_callback(self, msg: PathSegment) -> None:
        self.path_segment = msg
        #self.compute_rudder_angle()

    def true_wind_callback(self, msg: Wind) -> None:
        self.wind_direction_deg = msg.direction

    def forward_magnitude_callback(self, msg: Float64) -> None:
        self.lambda_base = msg.data

    def rudder_adjustment_scale_callback(self, msg: Float64) -> None:
        self.rudder_adjustment_scale = msg.data

    def rudder_overshoot_bias_callback(self, msg: Float64) -> None:
        self.rudder_overshoot_bias = msg.data
    
    def needs_to_tack(self, boat_heading, target_heading, wind_direction) -> bool:
        """
        Determines whether a tacking maneuver is required based on the current boat heading, target heading, and the direction
        of the wind. Tacking is necessary when the desired heading would cause the boat to turn through the wind vector. 
        This is only relevant for the sail controller, which will direct the sail to switch sides as the boat turns.

        :param boat_heading: The current heading of the boat in degrees.
        :param target_heading: The desired heading of the boat in degrees.
        :param wind_direction: The current wind direction in degrees.

        :return: A boolean value. Returns True if tacking is necessary to reach the target heading; otherwise, False.

        Function behavior includes:
        - Normalizing all heading and direction values to ensure they fall within a 0-360 degree range.
        - Calculating whether the shortest path from the current to the target heading is clockwise or counterclockwise.
        - Checking if the wind direction lies within the arc between the current and target headings in the chosen direction.
        - Publishing a position adjustment to the ballast to aid in tacking, if necessary.
        - Returning True if a tacking maneuver is needed based on the relative positions of the boat heading, target heading, and wind.

        This function also manages the ballast position by publishing to 'ballast_position_publisher' to optimize the boat's
        balance and performance during potential tacking maneuvers.
        """
        # Normalize angles
        boat_heading %= 360
        target_heading %= 360
        wind_direction %= 360
        
        clockwise = ((target_heading - boat_heading + 360) % 360) < 180
        
        if clockwise:
            if boat_heading <= wind_direction < target_heading or \
            (target_heading < boat_heading and (wind_direction > boat_heading or wind_direction < target_heading)):
                position_msg = Float64()
                position_msg.data = -0.5
                self.ballast_position_publisher.publish(position_msg)
                return True
        else:
            if target_heading <= wind_direction < boat_heading or \
            (boat_heading < target_heading and (wind_direction < boat_heading or wind_direction > target_heading)):
                position_msg = Float64()
                position_msg.data = 0.5
                self.ballast_position_publisher.publish(position_msg)
                return True
        
        return False
    
    def adaptive_vector_field(self, P1, P2, x, y, k_base=1.0, lambda_base=1.0):
        """
        Computes an adaptive vector field between two points, providing a navigation vector that changes dynamically based on
        the current position relative to a line segment defined by two points (P1 and P2). This method is used for
        path following, where the vector field helps steer the boat towards and along the path.

        :param P1: Tuple (x1, y1) representing the start point of the line segment.
        :param P2: Tuple (x2, y2) representing the end point of the line segment.
        :param x: Current x-coordinate of the boat.
        :param y: Current y-coordinate of the boat.
        :param k_base: Base proportional gain for the vector field calculation. Default is 1.0.
        :param lambda_base: Base rate for the attraction to the path's direction. Default is 1.0.

        :return: A numpy array representing the vector (V) in the 2D plane, pointing in the direction of movement required
                to reduce the error relative to the path and maintain alignment with the path direction.

        Function behavior includes:
        - Calculating the directional vector (d) from P1 to P2.
        - Projecting the current point (x, y) onto the line defined by P1 and P2 to find the closest point on the line.
        - Calculating the error vector (e) from the current point to this projection.
        - Determining the distance from the current point to the projected point on the line to dynamically adjust the gains.
        - Computing the navigation vector (V) which combines a component to reduce the perpendicular distance to the line
        (corrective force) and a component to move along the line (propulsive force).

        The adaptive nature of the gains k and lambda ensures that the corrective force diminishes as the point approaches
        the line, and the propulsive force increases, aiding in smoother convergence and travel along the path.
        """
        x1, y1 = P1
        x2, y2 = P2
        d = np.array([x2 - x1, y2 - y1])
        point = np.array([x, y])
        projection = P1 + np.dot(point - P1, d) / np.dot(d, d) * d
        e = point - projection
        dist = np.linalg.norm(e)
        k = k_base / 1/(1 + dist)  # Adaptive gain that decreases as distance decreases
        lambda_param = lambda_base * (1 / (1 + dist))  # Increases as the boat approaches the line
        V = -k * e + lambda_param * d
        return V
    
    def direction_vector(self, p1, p2):
        """
        Calculate the vector from point p1 to point p2.

        :param P1: Tuple (x1, y1) representing the start point of the line segment.
        :param P2: Tuple (x2, y2) representing the end point of the line segment.

        :return: A tuple representing the vector (dx, dy) from p1 to p2.
        """
        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1
        return (dx, dy)

    def vector_to_heading(self, dx, dy):
        """
        Convert vector components (dx, dy) at a grid position to a navigation heading.

        :param dx: Change in x grid coordinate.
        :param dy: Change in y grid coordinate.

        :return: Navigation bearing in degrees from north.
        """
        theta = math.atan2(dy, dx)  # Angle in radians
        bearing = (90 - math.degrees(theta)) % 360
        return bearing

    def compute_rudder_angle(self) -> None:
        """
        Computes and publishes the rudder angle based on the current heading, target segment, and other navigational parameters.
        The function adjusts the rudder angle to align the boat's heading with the target heading derived from the current path segment.
        The rudder adjustment also considers the rate of change in heading error and conditions such as the need to tack against the wind.

        This function performs several checks and computations:
        - Ensures that the boat is in full autonomous mode before making any adjustments.
        - Publishes a zero rudder angle if there is no target path segment.
        - Logs and exits if the current grid cell is not available.
        - Computes a target track from the current path segment using a vector field.
        - Checks if track would bring the boat upwind. Reuqests path replanning if that happens, and sets the boat to sail 
        along the edge of the no-sail zone
        - Applies an offset to correct for calculated leeway angle (heading vs. track)
        - Calculates heading error and its rate of change.
        - Uses a fuzzy logic controller to compute the required rudder adjustment.
        - Caps the rudder angle within specified limits and adjusts for tacking if necessary based on wind direction.
        - Publishes the computed rudder angle.

        :return: None. This function directly affects the boat's steering by publishing rudder angle adjustments to a designated ROS topic.

        The function relies on the following attributes:
        - 'autonomous_mode': The current mode of operation, checked against predefined autonomous modes.
        - 'path_segment': The current navigation path segment from which the target heading is derived.
        - 'current_grid_cell': The boat's current position in grid coordinates, necessary for vector field calculations.
        - 'rudder_simulator': A fuzzy logic controller for computing the rudder angle based on heading error and rate of change.
        - 'rudder_adjustment_scale': A proportional gain used to scale the rudder adjustment.
        - 'wind_direction_deg': The current wind direction, used to determine if tacking maneuvers are necessary.

        This method logs significant states and decisions to assist with debugging and operational monitoring.
        """
        autonomous_modes = AutonomousMode()
        if (self.autonomous_mode != autonomous_modes.AUTONOMOUS_MODE_FULL):
            #self.get_logger().info("Not in auto")
            return
        
        if(self.path_segment is None):
            msg = Int16()
            msg.data = int(0)
            self.rudder_angle_publisher.publish(msg)
            self.get_logger().info("No target segment")

            return
        
        if(self.current_grid_cell is None):
            self.get_logger().info("Current grid cell is none, cannot operate")
            return
        
        #self.get_logger().info(f"Current segment: {self.path_segment.start}, {self.path_segment.end}")
        
        # Start with a vector field
        grid_direction_vector = self.adaptive_vector_field((self.path_segment.start.x, self.path_segment.start.y), (self.path_segment.end.x,self.path_segment.end.y), self.current_grid_cell.x, self.current_grid_cell.y, k_base=self.k_base, lambda_base=self.lambda_base)

        #self.get_logger().info(f"Direction vector: {grid_direction_vector}")
        target_track = self.vector_to_heading(grid_direction_vector[0], grid_direction_vector[1])
        
        # If that fails, go straight along the segment
        if not math.isfinite(target_track):
            self.get_logger().warn(f"Vector field failed! Falling back to straight line. p1: {(self.path_segment.start.x, self.path_segment.start.y)}, p2: {(self.path_segment.end.x,self.path_segment.end.y)}")
            grid_direction_vector = self.direction_vector((self.path_segment.start.x, self.path_segment.start.y), (self.path_segment.end.x,self.path_segment.end.y))
            target_track = self.vector_to_heading(grid_direction_vector[0], grid_direction_vector[1])
        # If that fails, just go to segment endpoint. Could bring us upwind, but that's a later problem.
        if not math.isfinite(target_track):
            self.get_logger().warn(f"Straight line also failed!")
            grid_direction_vector = self.direction_vector((self.current_grid_cell.x, self.current_grid_cell.y), (self.path_segment.end.x,self.path_segment.end.y))
            target_track = self.vector_to_heading(grid_direction_vector[0], grid_direction_vector[1])
        
        # If the necessary track would bring us too far upwind, request a replan
        # Don't trust this, disabling modifications
        # if(self.wind_direction_deg is not None):
        #     if(is_in_nogo(math.radians(target_track), math.radians(self.wind_direction_deg), math.radians(self.wind_restriction_replan_cutoff_degrees))):
        #         self.get_logger().warn(f"Target track is upwind, need to replan. Wind dir: {self.wind_direction_deg}, track: {target_track}")
        #         self.request_replan_publisher.publish(Empty())
                # Set track to bring us along edge of nogo zone
                #target_track = closest_edge_heading(math.radians(target_track), math.radians(self.wind_direction_deg), math.radians(self.wind_restriction_replan_cutoff_degrees))

        target_track_msg = Float64()
        target_track_msg.data = target_track
        self.target_track_debug_publisher.publish(target_track_msg)
        
        # Adjust for leeway angle, up to a set amount
        leeway_adjustment = -max(-self.leeway_correction_limit, min(self.leeway_angle, self.leeway_correction_limit))
        target_heading = target_track+leeway_adjustment # Whatever our leeway angle is, add the inverse of that to our target heading

        target_heading_msg = Float64()
        target_heading_msg.data = target_heading
        self.target_heading_debug_publisher.publish(target_heading_msg)
        
        #self.get_logger().info(f"Target heading: {target_heading}")
        heading_error = math.degrees(normalRelativeAngle(math.radians(self.heading-target_heading)))

        current_time = time.time()
        delta_time = current_time-self.last_rudder_time
        heading_rate_of_change = (heading_error - self.last_heading_error)/delta_time
        self.last_heading_error = heading_error
        #self.get_logger().info(f"Heading error: {heading_error} from heading: {self.heading} grid pos: {self.current_grid_cell} along segment: {self.path_segment}")
        self.rudder_simulator.input['heading_error'] = heading_error
        self.rudder_simulator.input['rate_of_change'] = heading_rate_of_change * self.rudder_overshoot_bias
        self.rudder_simulator.compute()
        rudder_value = self.rudder_simulator.output['rudder_adjustment']

        last_rudder_angle = self.rudder_angle

        is_tack = False
        if(self.wind_direction_deg is not None):
            if(self.needs_to_tack(self.heading, target_heading, self.wind_direction_deg)):
                is_tack = True
        
        #if we'd need to tack, 
        if is_tack:
            if self.allow_tack:
                self.rudder_angle += rudder_value*self.rudder_adjustment_scale
            else:
                self.rudder_angle -= rudder_value*self.rudder_adjustment_scale
        else:
            self.rudder_angle += rudder_value*self.rudder_adjustment_scale

        if(self.rudder_angle>30):
            self.rudder_angle = 30
        elif self.rudder_angle<-30:
            self.rudder_angle = -30

        # If we are tacking, turn as hard as possible.
        # Trim tab controller will see this and skip over min_lift
        if is_tack and self.allow_tack:
            if(self.rudder_angle>0):
                self.rudder_angle = 31
            else:
                self.rudder_angle = -31
            self.request_tack_publisher.publish(Empty())
                

        #self.get_logger().info(f"Computed rudder angle: {rudder_angle}")
        msg = Int16()
        try:
            msg.data = int(self.rudder_angle)
        except Exception as e:
            self.get_logger().error(f"Invalid rudder value: {self.rudder_angle} rudder change this iteration: {rudder_value}, grid vector: {grid_direction_vector}, path points: {(self.path_segment.start.x, self.path_segment.start.y), (self.path_segment.end.x,self.path_segment.end.y)}")
            self.rudder_angle = last_rudder_angle
            msg.data = int(self.rudder_angle)

        #self.get_logger().info(f"Rudder angle: {self.rudder_angle}")

        self.rudder_angle_publisher.publish(msg)

    #gets necessary rotation from an lat, long, theta pose to face a point
    def getRotationToPointLatLong(self, current_theta, current_lat, current_long, target_lat, target_long) -> float:
        # Convert latitude and longitude from degrees to radians
        lat1 = math.radians(current_lat)
        lon1 = math.radians(current_long)
        lat2 = math.radians(target_lat)
        lon2 = math.radians(target_long)
        
        delta_lon = lon2 - lon1
        #self.get_logger().info(f"delta_lon: {delta_lon}")
        
        # Calculate the bearing from current location to target location
        x = math.sin(delta_lon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon))
        #self.get_logger().info(f"x: {x}, y: {y}")
        initial_bearing = math.atan2(x, y)

        initial_bearing = math.degrees(initial_bearing)
        #self.get_logger().info(f"initial bearing: {initial_bearing}")
        
        #self.get_logger().info(f"Current theta: {current_theta}")
        # Calculate turn required by normalizing the difference between the bearing and current orientation
        turn = normalRelativeAngle(math.radians(current_theta-initial_bearing))
        
        return math.degrees(turn)

    def publish_error(self, string: str):
        error_msg = String()
        error_msg.data = string
        self.error_publisher.publish(error_msg)


def main(args=None):
    rclpy.init(args=args)
    heading_control = HeadingController()

    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(heading_control)

    try:
        # Spin the node to execute callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        trace = traceback.format_exc()
        error_string = f'Unhandled exception: {e}\n{trace}'
        heading_control.get_logger().fatal(error_string)
        heading_control.publish_error(error_string)
    finally:
        # Shutdown and cleanup the node
        executor.shutdown()
        heading_control.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()