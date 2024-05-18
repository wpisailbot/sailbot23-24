#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String, Float64, Int16, Empty
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
import time

from typing import Optional
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.subscription import Subscription

from sailbot_msgs.msg import AutonomousMode, GeoPath, Wind, PathSegment
from sensor_msgs.msg import MagneticField

class HeadingSelect(LifecycleNode):

    use_camera_heading = False

    def __init__(self):
        super().__init__('heading_select')

        self.set_parameters()
        self.get_parameters()

        self.rudder_angle_publisher: Optional[Publisher]
        self.path_segment_subscription: Optional[Subscription]
        self.airmar_heading_subscription: Optional[Subscription]
        self.airmar_track_degrees_true_subscription: Optional[Subscription]
        self.current_grid_cell_subscription: Optional[Subscription]
        self.autonomous_mode_subscription: Optional[Subscription]
        self.current_path_subscription: Optional[Subscription]


        self.timer: Optional[Timer]
        # self.target_position = GeoPoint()
        # self.target_position.latitude = 42.273051
        # self.target_position.longitude = -71.805049

    def set_parameters(self) -> None:
        self.declare_parameter('sailbot.heading_select.use_camera_heading', False)

    def get_parameters(self) -> None:
        self.use_camera_heading = self.get_parameter('sailbot.heading_select.use_camera_heading').get_parameter_value().bool_value
        
    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")

        self.heading_publisher = self.create_lifecycle_publisher(Float64, 'heading', 10)

        self.airmar_heading_subscription = self.create_subscription(
            Float64,
            '/airmar_data/heading',
            self.airmar_heading_callback,
            10)
        self.camera_heading_subscription = self.create_subscription(
            MagneticField,
            '/zed/zed_node/imu/mag',
            self.magnetic_field_callback,
            10)
 
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
        if(self.use_camera_heading == False):
            self.heading = msg.data
            heading_msg = Float64()
            heading_msg.data = msg.data
            self.heading_publisher.publish(heading_msg)

    def magnetic_field_callback(self, msg: MagneticField):
        if(self.use_camera_heading == True):
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

            heading_msg = Float64()
            heading_msg.data = heading_degrees
            self.heading_publisher.publish(heading_msg)

            self.get_logger().info("Heading: {:.2f} degrees".format(heading_degrees))



def main(args=None):
    rclpy.init(args=args)
    heading_select = HeadingSelect()

    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(heading_select)

    try:
        # Spin the node to execute callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        trace = traceback.format_exc()
        heading_select.get_logger().fatal(f'Unhandled exception: {e}\n{trace}')
    finally:
        # Shutdown and cleanup the node
        executor.shutdown()
        heading_select.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()