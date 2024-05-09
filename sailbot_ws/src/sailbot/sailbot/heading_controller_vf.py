#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String, Float64, Int16
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint
# import json
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import math
import traceback
import numpy as np

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

class HeadingController(LifecycleNode):

    heading = 0
    #latitude = 42.273822
    #longitude = -71.805967
    current_grid_cell = None
    path_segment = None
    wind_direction_deg = None
    autonomous_mode = 0
    heading_kp = None
    rudder_angle = 0

    last_heading_error = 0

    def __init__(self):
        super().__init__('heading_controller')

        self.set_parameters()
        self.get_parameters()

        self.rudder_angle_publisher: Optional[Publisher]
        self.path_segment_subscription: Optional[Subscription]
        self.airmar_heading_subscription: Optional[Subscription]
        self.current_grid_cell_subscription: Optional[Subscription]
        self.autonomous_mode_subscription: Optional[Subscription]
        self.current_path_subscription: Optional[Subscription]


        self.timer: Optional[Timer]
        # self.target_position = GeoPoint()
        # self.target_position.latitude = 42.273051
        # self.target_position.longitude = -71.805049

    def set_parameters(self) -> None:
        self.declare_parameter('sailbot.heading_control.heading_kp', 0.1)
        self.declare_parameter('sailbot.heading_control.vector_field_crosstrack_weight', 1.0)
        self.declare_parameter('sailbot.heading_control.vector_field_path_dir_weight', 1.0)

    def get_parameters(self) -> None:
        self.heading_kp = self.get_parameter('sailbot.heading_control.heading_kp').get_parameter_value().double_value
        self.k_base = self.get_parameter('sailbot.heading_control.vector_field_crosstrack_weight').get_parameter_value().double_value
        self.lambda_base = self.get_parameter('sailbot.heading_control.vector_field_path_dir_weight').get_parameter_value().double_value
        
    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")

        self.ballast_position_publisher = self.create_lifecycle_publisher(Float64, 'ballast_position', 10)

        self.rudder_angle_publisher = self.create_lifecycle_publisher(Int16, 'rudder_angle', 10)

        self.target_heading_debug_publisher = self.create_lifecycle_publisher(Float64, 'target_heading', 10)


        self.path_Segment_subscription = self.create_subscription(
            PathSegment,
            'current_path_segment',
            self.path_segment_callback,
            10)
        self.airmar_heading_subscription = self.create_subscription(
            Float64,
            '/airmar_data/heading',
            self.airmar_heading_callback,
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
        self.rudder_kp_subscription = self.create_subscription(
            Float64,
            'rudder_kp',
            self.rudder_kp_callback,
            10)
        self.autonomous_mode_subscription = self.create_subscription(AutonomousMode, 'autonomous_mode', self.autonomous_mode_callback, 10)

        
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

    def current_path_callback(self, msg: GeoPath) -> None:
        if len(msg.points) == 0:
            self.path_segment = None

    def autonomous_mode_callback(self, msg: AutonomousMode) -> None:
        self.get_logger().info(f"Got autonomous mode: {msg.mode}")
        self.autonomous_mode = msg.mode

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
    
    def airmar_position_callback(self, msg: NavSatFix) -> None:
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        #self.compute_rudder_angle()

    def current_grid_cell_callback(self, msg: Point) -> None:
        self.current_grid_cell = msg
        self.get_logger().info("Got new grid cell!")
        #self.compute_rudder_angle()
    
    def path_segment_callback(self, msg: PathSegment) -> None:
        self.path_segment = msg
        #self.compute_rudder_angle()

    def true_wind_callback(self, msg: Wind) -> None:
        self.wind_direction_deg = msg.direction

    def forward_magnitude_callback(self, msg: Float64) -> None:
        self.lambda_base = msg.data

    def rudder_kp_callback(self, msg: Float64) -> None:
        self.heading_kp = msg.data
    
    def needs_to_tack(self, boat_heading, target_heading, wind_direction) -> bool:
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
    
    def adaptive_vector_field(self, P1, P2, x, y, k_base=1, lambda_base=0.1):
        x1, y1 = P1
        x2, y2 = P2
        d = np.array([x2 - x1, y2 - y1])
        point = np.array([x, y])
        projection = P1 + np.dot(point - P1, d) / np.dot(d, d) * d
        e = point - projection
        dist = np.linalg.norm(e)
        k = k_base / 1/(1 + dist)  # Adaptive gain that decreases as distance decreases
        lambda_param = lambda_base * (1 / (1 + dist))  # Increases as the robot approaches the line
        V = -k * e + lambda_param * d
        return V

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

    def compute_rudder_angle(self) -> None:
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
        
        grid_direction_vector = self.adaptive_vector_field((self.path_segment.start.x, self.path_segment.start.y), (self.path_segment.end.x,self.path_segment.end.y), self.current_grid_cell.x, self.current_grid_cell.y, k_base=self.k_base, lambda_base=self.lambda_base)
        self.get_logger().info(f"Direction vector: {grid_direction_vector}")
        target_heading = self.vector_to_heading(grid_direction_vector[0], grid_direction_vector[1])
        target_heading_msg = Float64()
        target_heading_msg.data = target_heading
        self.target_heading_debug_publisher.publish(target_heading_msg)
        
        self.get_logger().info(f"Target heading: {target_heading}")
        heading_error = math.degrees(normalRelativeAngle(math.radians(self.heading-target_heading)))
        delta_heading_error = heading_error - self.last_heading_error
        self.last_heading_error = heading_error
        self.get_logger().info(f"Heading error: {heading_error} from heading: {self.heading} grid pos: {self.current_grid_cell} along segment: {self.path_segment}")
        self.rudder_simulator.input['heading_error'] = heading_error
        self.rudder_simulator.input['rate_of_change'] = delta_heading_error
        self.rudder_simulator.compute()
        rudder_value = self.rudder_simulator.output['rudder_adjustment']
        self.rudder_angle += rudder_value*self.heading_kp # Scale

        if(self.rudder_angle>30):
            self.rudder_angle = 30
        elif self.rudder_angle<-30:
            self.rudder_angle = -30

        # If we are tacking, turn as hard as possible.
        # Trim tab controller will see this and skip over min_lift
        if(self.wind_direction_deg is not None):
            if(self.needs_to_tack(self.heading, target_heading, self.wind_direction_deg)):
                if(self.rudder_angle>0):
                    self.rudder_angle = 31
                else:
                    self.rudder_angle = -31

        #self.get_logger().info(f"Computed rudder angle: {rudder_angle}")
        msg = Int16()
        msg.data = int(self.rudder_angle)
        self.get_logger().info(f"Rudder angle: {self.rudder_angle}")

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
        heading_control.get_logger().fatal(f'Unhandled exception: {e}\n{trace}')
    finally:
        # Shutdown and cleanup the node
        executor.shutdown()
        heading_control.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()