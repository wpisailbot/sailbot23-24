#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String, Float64, Int16
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoint
import json
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import math

from typing import Optional
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.subscription import Subscription

from sailbot_msgs.msg import AutonomousMode, Path

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
    latitude = 42.273822
    longitude = -71.805967
    target_position = None
    autonomous_mode = 0

    def __init__(self):
        super().__init__('heading_controller')
        self.rudder_angle_publisher: Optional[Publisher]
        self.target_position_subscription: Optional[Subscription]
        self.airmar_heading_subscription: Optional[Subscription]
        self.airmar_position_subscription: Optional[Subscription]
        self.autonomous_mode_subscription: Optional[Subscription]
        self.current_path_subscription: Optional[Subscription]


        self.timer: Optional[Timer]
        # self.target_position = GeoPoint()
        # self.target_position.latitude = 42.273051
        # self.target_position.longitude = -71.805049

    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")

        self.rudder_angle_publisher = self.create_lifecycle_publisher(Int16, 'rudder_angle', 10)

        self.target_position_subscription = self.create_subscription(
            GeoPoint,
            'target_position',
            self.target_position_callback,
            10)
        self.airmar_heading_subscription = self.create_subscription(
            Float64,
            '/airmar_data/heading',
            self.airmar_heading_callback,
            10)
        self.airmar_position_subscription = self.create_subscription(
            NavSatFix,
            '/airmar_data/lat_long',
            self.airmar_position_callback,
            10)
        self.current_path_subscription = self.create_subscription(
            Path,
            'current_path',
            self.current_path_callback,
            10)
        self.autonomous_mode_subscription = self.create_subscription(AutonomousMode, 'autonomous_mode', self.autonomous_mode_callback, 10)

        
        #self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Heading controller node configured")

        heading_error = ctrl.Antecedent(np.arange(-180, 181, 1), 'heading_error')
        rate_of_change = ctrl.Antecedent(np.arange(-90, 91, 1), 'rate_of_change')
        rudder_angle = ctrl.Consequent(np.arange(-90, 91, 1), 'rudder_angle')

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
        rudder_angle['Large Negative'] = fuzz.gaussmf(rudder_angle.universe, -90, 10)  # mean = -45, std = 10
        rudder_angle['Small Negative'] = fuzz.gaussmf(rudder_angle.universe, -60, 10)  # mean = -15, std = 10
        rudder_angle['Zero'] = fuzz.gaussmf(rudder_angle.universe, 0, 10)              # mean = 0, std = 10
        rudder_angle['Small Positive'] = fuzz.gaussmf(rudder_angle.universe, 60, 10)   # mean = 15, std = 10
        rudder_angle['Large Positive'] = fuzz.gaussmf(rudder_angle.universe, 90, 10)   # mean = 45, std = 10

        # Define the rules
        rule1 = ctrl.Rule(heading_error['ZE'] & rate_of_change['Large Negative'], rudder_angle['Large Positive'])
        rule2 = ctrl.Rule(heading_error['ZE'] & rate_of_change['Small Negative'], rudder_angle['Small Positive'])
        rule3 = ctrl.Rule(heading_error['ZE'] & rate_of_change['Zero'], rudder_angle['Zero'])
        rule4 = ctrl.Rule(heading_error['ZE'] & rate_of_change['Small Positive'], rudder_angle['Small Negative'])
        rule5 = ctrl.Rule(heading_error['ZE'] & rate_of_change['Large Positive'], rudder_angle['Large Negative'])

        rule6 = ctrl.Rule(heading_error['PM'] & rate_of_change['Large Negative'], rudder_angle['Small Negative'])
        rule7 = ctrl.Rule(heading_error['PM'] & rate_of_change['Small Negative'], rudder_angle['Small Negative'])
        rule8 = ctrl.Rule(heading_error['PM'] & rate_of_change['Zero'], rudder_angle['Small Negative'])
        rule9 = ctrl.Rule(heading_error['PM'] & rate_of_change['Small Positive'], rudder_angle['Large Negative'])
        rule10 = ctrl.Rule(heading_error['PM'] & rate_of_change['Large Positive'], rudder_angle['Large Negative'])

        rule11 = ctrl.Rule(heading_error['PL'], rudder_angle['Large Negative'])

        rule12 = ctrl.Rule(heading_error['NM'] & rate_of_change['Large Positive'], rudder_angle['Small Positive'])
        rule13 = ctrl.Rule(heading_error['NM'] & rate_of_change['Small Positive'], rudder_angle['Small Positive'])
        rule14 = ctrl.Rule(heading_error['NM'] & rate_of_change['Zero'], rudder_angle['Small Positive'])
        rule15 = ctrl.Rule(heading_error['NM'] & rate_of_change['Small Negative'], rudder_angle['Large Positive'])
        rule16 = ctrl.Rule(heading_error['NM'] & rate_of_change['Large Negative'], rudder_angle['Large Positive'])

        rule17 = ctrl.Rule(heading_error['NL'], rudder_angle['Large Positive'])

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
        super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")
        # Destroy subscribers, publishers, and timers
        self.destroy_timer(self.timer)
        self.destroy_lifecycle_publisher(self.rudder_angle_publisher)
        self.destroy_subscription(self.airmar_heading_subscription)
        self.destroy_subscription(self.target_position_subscription)
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

    def current_path_callback(self, msg: Path):
        if len(msg.points) == 0:
            self.target_position = None

    def autonomous_mode_callback(self, msg: AutonomousMode):
        self.get_logger().info(f"Got autonomous mode: {msg.mode}")
        self.autonomous_mode = msg.mode

    def timer_callback(self):
        self.compute_rudder_angle()

    def airmar_heading_callback(self, msg: Float64):
        self.heading = msg.data
        self.compute_rudder_angle()
    
    def airmar_position_callback(self, msg: NavSatFix):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.compute_rudder_angle()
    
    def target_position_callback(self, msg: GeoPoint):
        self.target_position = msg
        self.compute_rudder_angle()
    
    def compute_rudder_angle(self):
        autonomous_modes = AutonomousMode()
        if (self.autonomous_mode != autonomous_modes.AUTONOMOUS_MODE_FULL):
            return
        
        if(self.target_position is None):
            msg = Int16()
            msg.data = int(0)
            self.rudder_angle_publisher.publish(msg)
            return
        
        heading_error = self.getRotationToPointLatLong(self.heading, self.latitude, self.longitude, self.target_position.latitude, self.target_position.longitude)
        #self.get_logger().info(f"Heading error: {heading_error} from heading: {self.heading} pos: {self.latitude}, {self.longitude} to pos: {self.target_position.latitude}, {self.target_position.longitude}")
        #self.rudder_simulator.input['heading_error'] = heading_error
        #self.rudder_simulator.input['rate_of_change'] = 0 # Heading rate-of-change, not sure if Airmar provides this directly. Zero for now.
        #self.rudder_simulator.compute()
        #rudder_angle = self.rudder_simulator.output['rudder_angle']
        rudder_angle = heading_error # P controller for now
        if(rudder_angle>90):
            rudder_angle = 90
        elif rudder_angle<-90:
            rudder_angle = -90
        #self.get_logger().info(f"Computed rudder angle: {rudder_angle}")
        msg = Int16()
        msg.data = int(rudder_angle)
        self.rudder_angle_publisher.publish(msg)

    #gets necessary rotation from an lat, long, theta pose to face a point
    def getRotationToPointLatLong(self, current_theta, current_lat, current_long, target_lat, target_long):
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
        heading_control.get_logger().fatal(f'Unhandled exception: {e}')
    finally:
        # Shutdown and cleanup the node
        executor.shutdown()
        heading_control.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()