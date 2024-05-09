#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String, Float64, Int16
from sailbot_msgs.msg import Wind, AutonomousMode

import json
import board
import busio
from time import time as get_time
from typing import Optional
import traceback
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

    roll_errors = []
    num_error_readings = 20

    ADC_FULL_STARBOARD = 700
    ADC_FULL_PORT = 2300
    ADC_MAX = ADC_FULL_PORT
    ADC_MIN = ADC_FULL_STARBOARD
    
    MOTOR_FAST_STARBOARD = 130
    MOTOR_FAST_PORT = 60
    MOTOR_STOP = 95

    CONTROL_FAST_PORT=-1.0
    CONTROL_FAST_STARBOARD=1.0

    ERROR_MIN_MAGNITUDE = 20

    RAMP_UP_TIME = 2.0
    start_time = None
    in_ramp_up = False

    Kp = 0.005
    Kd = 0.1

    roll_kp = 20
    previous_error = 0
    previous_time = get_time()

    current_target = (ADC_FULL_PORT-ADC_FULL_STARBOARD)/2+ADC_FULL_STARBOARD
    current_ballast_position = current_target

    move = False
    current_wind_dir = None

    autonomous_mode = 0

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

        self.ballast_pwm_publisher = self.create_lifecycle_publisher(Int16, 'ballast_pwm', 1)

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
        self.apparent_wind_subscription = self.create_subscription(
            Wind,
            "apparent_wind_smoothed",
            self.apparent_wind_callback,
            10)
        self.airmar_heading_subscription = self.create_subscription(
            Float64,
            "/airmar_data/heading",
            self.airmar_heading_callback,
            10
        )
        self.autonomous_mode_subscriber = self.create_subscription(AutonomousMode, 'autonomous_mode', self.autonomous_mode_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop_callback)
        self.roll_correction_timer = self.create_timer(10, self.roll_correction_callback)
        self.get_logger().info("Ballast node configured")
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
    
    def autonomous_mode_callback(self, msg: AutonomousMode) -> None:
        self.get_logger().info(f"Got autonomous mode: {msg.mode}")
        self.autonomous_mode = msg.mode

    def median(self, lst):
        n = len(lst)
        s = sorted(lst)
        return (sum(s[n // 2 - 1:n // 2 + 1]) / 2.0, s[n // 2])[n % 2] if n else None

    def constrain_control(self, control):
        return bound(self.CONTROL_FAST_PORT, self.CONTROL_FAST_STARBOARD, control)
    
    def control_to_motor_value(self, control):
        return  self.MOTOR_FAST_PORT + ((self.MOTOR_FAST_STARBOARD - self.MOTOR_FAST_PORT) / (self.CONTROL_FAST_STARBOARD - self.CONTROL_FAST_PORT)) * (control - self.CONTROL_FAST_PORT)

    def ballast_position_callback(self, msg: Float64):
        self.get_logger().info("Received ballast setpoint: "+str(msg.data))
        self.current_target = self.ADC_FULL_PORT + ((self.ADC_FULL_STARBOARD - self.ADC_FULL_PORT) / (1.0 - -1.0)) * (msg.data - -1.0)
        self.move = True
        self.in_ramp_up = True
        self.start_time = get_time()

    def current_ballast_position_callback(self, msg: Int16):
        #self.get_logger().info("Got current ballast position")
        self.current_ballast_position = msg.data

    def airmar_roll_callback(self, msg: Float64):
        roll_target = 0

        if(self.current_wind_dir is None):
            self.get_logger().info("No apparent wind yet.")
            return

        relative_wind_angle = (self.current_wind_dir - self.current_heading + 360) % 360
        if relative_wind_angle == 180:
            pass # Directly from behind
        elif relative_wind_angle == 0:
            pass # Directly ahead
        elif relative_wind_angle < 180:
            roll_target = 20 # Starboard
        else:
            roll_target = -20 # Port
        
        roll_error = msg.data-roll_target
        self.roll_errors.append(float(roll_error))
        if len(self.roll_errors) > self.num_error_readings:
            self.roll_errors.pop(0)

        #self.get_logger().info("Got roll data!")

    def airmar_heading_callback(self, msg: Float64):
        self.current_heading = msg.data

    def apparent_wind_callback(self, msg: Wind):
        self.current_wind_dir = msg.direction

    def roll_correction_callback(self):
        if(self.autonomous_mode != AutonomousMode.AUTONOMOUS_MODE_FULL and self.autonomous_mode != AutonomousMode.AUTONOMOUS_MODE_BALLAST and self.autonomous_mode != AutonomousMode.AUTONOMOUS_MODE_TRIMTAB):
            return
        
        if(len(self.roll_errors) == 0):
            self.get_logger().info("No roll errors yet")
            return
        avg_roll_err = self.median(self.roll_errors)

        move = self.roll_kp*avg_roll_err
        new_target = self.current_target+move

        if(new_target < self.ADC_MIN):
            new_target = self.ADC_MIN
        elif(new_target > self.ADC_MAX):
            new_target = self.ADC_MAX

        self.current_target = new_target
        #self.get_logger().info(f"Set target from roll: {self.current_target}")
        self.move = True
        self.in_ramp_up = True
        self.start_time = get_time()


    def control_loop_callback(self):
        if(self.move == False):
            return
        # if(self.current_ballast_position < 500 or self.current_ballast_position > 3000):
        #     #self.get_logger().info("Ballast position is 0, assuming it's broken")
        #     self.get_logger().info("Ballast out of range!")
        #     return
        current_error = self.current_ballast_position - self.current_target
        #self.get_logger().info(f"Current target: {self.current_target}, current position: {self.current_ballast_position}")
        if abs(current_error)<self.ERROR_MIN_MAGNITUDE:
            self.get_logger().info("Error is small, stopping")
            msg = Int16()
            msg.data = int(95)
            self.ballast_pwm_publisher.publish(msg)
            self.move=False
            return
        
        current_time = get_time()
        delta_time = current_time-self.previous_time
        ramp_factor = 1
        if self.in_ramp_up:
            elapsed_time = current_time - self.start_time
            if elapsed_time >= self.RAMP_UP_TIME:
                self.in_ramp_up = False
            else:
                ramp_factor = elapsed_time / self.RAMP_UP_TIME

        error_derivative = (current_error - self.previous_error) / delta_time
        motor_value = self.control_to_motor_value(self.constrain_control(self.Kp*current_error*ramp_factor))
        #self.get_logger().info("Current target: "+str(self.current_target) + " Current position: "+str(self.current_ballast_position)+" Current motor value: "+str(motor_value))
        
        self.previous_error = current_error
        self.previous_time = current_time

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
        trace = traceback.format_exc()
        ballast_control.get_logger().fatal(f'Unhandled exception: {e}\n{trace}')
    finally:
        # Shutdown and cleanup the node
        executor.shutdown()
        ballast_control.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()