#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String, Float64, Int16
import json
import board
import busio
import time
from typing import Optional
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.subscription import Subscription

def make_json_string(json_msg):
    json_str = json.dumps(json_msg)
    message = String()
    message.data = json_str
    return message

def bound(low, high, value):
    return max(low, min(high, value))

class BallastControl(LifecycleNode):
    ADC_FULL_STARBOARD = 570
    ADC_FULL_PORT = 2140
    
    MOTOR_FAST_STARBOARD = 130
    MOTOR_FAST_PORT = 60

    CONTROL_FAST_PORT=-1.0
    CONTROL_FAST_STARBOARD=1.0

    Kp = 0.003

    current_target = (ADC_FULL_PORT-ADC_FULL_STARBOARD)/2+ADC_FULL_STARBOARD
    current_ballast_position = current_target

    def __init__(self):
        super().__init__('ballast_control')
        self.ballast_pwm_publisher: Optional[Publisher]
        self.position_subscription: Optional[Subscription]
        self.current_ballast_position_subscription: Optional[Subscription]
        self.airmar_roll_subscription: Optional[Subscription]
        self.timer: Optional[Timer]

    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")

        self.ballast_pwm_publisher = self.create_lifecycle_publisher(Int16, 'ballast_pwm', 10)

        self.position_subscription = self.create_subscription(
            Float64,
            'ballast_position',
            self.ballast_position_callback,
            10)
        self.current_ballast_position_subscription = self.create_subscription(
            Int16,
            'current_ballast_position',
            self.current_ballast_position_callback,
            10)
        self.airmar_roll_subscription = self.create_subscription(
            Float64,
            '/airmar_data/roll',
            self.airmar_roll_callback,
            10)
        self.timer = self.create_timer(0.1, self.control_loop_callback)
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
        self.destroy_lifecycle_publisher(self.pwm_control_publisher)
        self.destroy_subscription(self.position_subscription)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        # Perform final cleanup if necessary
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Error caught!")
        return super().on_error(state)
    
    #end callbacks
    
    def constrain_control(self, control):
        return bound(self.CONTROL_FAST_PORT, self.CONTROL_FAST_STARBOARD, control)
    
    def control_to_motor_value(self, control):
        return  self.MOTOR_FAST_PORT + ((self.MOTOR_FAST_STARBOARD - self.MOTOR_FAST_PORT) / (self.CONTROL_FAST_STARBOARD - self.CONTROL_FAST_PORT)) * (control - self.CONTROL_FAST_PORT)

    def ballast_position_callback(self, msg: Float64):
        self.get_logger().info("Received ballast setpoint: "+str(msg.data))
        self.current_target = self.ADC_FULL_PORT + ((self.ADC_FULL_STARBOARD - self.ADC_FULL_PORT) / (1.0 - -1.0)) * (msg.data - -1.0)

    def current_ballast_position_callback(self, msg: Int16):
        #self.get_logger().info("Got current ballast position")
        self.current_ballast_position = msg.data

    def airmar_roll_callback(self, msg: Float64):
        self.get_logger().info("Got roll data!")

    def control_loop_callback(self):
        motor_value = self.control_to_motor_value(self.constrain_control(self.Kp*(self.current_ballast_position-self.current_target)))
        #self.get_logger().info("Current target: "+str(self.current_target) + " Current position: "+str(self.current_ballast_position)+" Current motor value: "+str(motor_value))
        
        msg = Int16()
        msg.data = int(motor_value)
        self.ballast_pwm_publisher.publish(msg)
        pass


def main(args=None):
    rclpy.init(args=args)
    ballast_control = BallastControl()

    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(ballast_control)

    try:
        # Spin the node to execute callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        ballast_control.get_logger().fatal(f'Unhandled exception: {e}')
    finally:
        # Shutdown and cleanup the node
        executor.shutdown()
        ballast_control.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()