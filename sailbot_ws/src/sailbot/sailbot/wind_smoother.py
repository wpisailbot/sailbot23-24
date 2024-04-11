#!/usr/bin/env python3
import json
import rclpy
from typing import Optional
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.subscription import Subscription
from std_msgs.msg import String, Empty, Float64

from sailbot_msgs.msg import Wind

import math
import time
import traceback

class WindSmoother(LifecycleNode):

    last_apparent_winds = []
    last_true_winds = []
    num_wind_readings = 20

    def __init__(self):
        self.apparent_wind_subscriber: Optional[Subscription]

        self.smooth_apparent_wind_publisher: Optional[Publisher]
        super().__init__('wind_smoother')

    
    def set_parameters(self) -> None:
        self.declare_parameter('sailbot.pathfinding.num_wind_readings', 20)

    def get_parameters(self) -> None:
        self.num_wind_readings = self.get_parameter('sailbot.pathfinding.num_wind_readings').get_parameter_value().integer_value

    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")
        self.apparent_wind_subscriber = self.create_subscription(Wind, 'airmar_data/apparent_wind', self.apparent_wind_callback, 10)
        self.true_wind_subscriber = self.create_subscription(Wind, 'airmar_data/true_wind', self.true_wind_callback, 10)
        self.smooth_apparent_wind_publisher = self.create_lifecycle_publisher(Wind, 'apparent_wind_smoothed', 10)
        self.smooth_true_wind_publisher = self.create_lifecycle_publisher(Wind, 'true_wind_smoothed', 10)

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
        self.destroy_subscription(self.apparent_wind_subscriber)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        # Perform final cleanup if necessary
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Error caught!")
        return super().on_error(state)
    
    #end callbacks

    def median(self, lst):
        n = len(lst)
        s = sorted(lst)
        return (sum(s[n // 2 - 1:n // 2 + 1]) / 2.0, s[n // 2])[n % 2] if n else None
    
    def update_apparent_winds(self, relative_wind_direction):
        # Check we have new wind
        if len(self.last_apparent_winds) != 0 and relative_wind_direction == self.last_apparent_winds[len(self.last_apparent_winds) - 1]:
            return
            # First add wind to running list
        self.last_apparent_winds.append(float(relative_wind_direction))
        if len(self.last_apparent_winds) > self.num_wind_readings:
            self.last_apparent_winds.pop(0)
    
    def update_true_winds(self, true_wind_direction):
        # Check we have new wind
        if len(self.last_true_winds) != 0 and true_wind_direction == self.last_true_winds[len(self.last_true_winds) - 1]:
            return
            # First add wind to running list
        self.last_true_winds.append(float(true_wind_direction))
        if len(self.last_true_winds) > self.num_wind_readings:
            self.last_true_winds.pop(0)

    def apparent_wind_callback(self, msg: Wind):
        self.update_apparent_winds(msg.direction)
        smooth_angle = self.median(self.last_apparent_winds)
        smooth = Wind()
        smooth.direction = float(smooth_angle)
        smooth.speed = msg.speed
        self.smooth_apparent_wind_publisher.publish(smooth)

    def true_wind_callback(self, msg: Wind):
        self.update_true_winds(msg.direction)
        smooth_angle = self.median(self.last_true_winds)
        smooth = Wind()
        smooth.direction = float(smooth_angle)
        smooth.speed = msg.speed
        self.smooth_true_wind_publisher.publish(smooth)



def main(args=None):
    rclpy.init(args=args)
    wind_smoother = WindSmoother()

    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(wind_smoother)

    try:
        # Spin the node to execute callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        trace = traceback.format_exc()
        wind_smoother.get_logger().fatal(f'Unhandled exception: {e}\n{trace}')
    finally:
        # Shutdown and cleanup the node
        executor.shutdown()
        wind_smoother.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

