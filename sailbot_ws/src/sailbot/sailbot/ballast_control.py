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
    """
    A ROS 2 lifecycle node for controlling the ballast system of a sailboat, handling dynamic positioning based on sensor inputs
    to maintain boat stability and optimal sailing conditions.

    This node interfaces with hardware through PWM signals to control the ballast motor based on roll data from an Airmar sensor,
    adjusting the ballast's position to counteract roll induced by wind or other environmental factors.

    :ivar ballast_pwm_publisher: Publisher for ballast motor PWM values.
    :ivar roll_errors: (list) Stores recent roll error measurements to compute the average roll error.
    :ivar num_error_readings: (int) Number of roll readings to keep for averaging.
    :ivar ADC_FULL_STARBOARD: (int) ADC value corresponding to the full starboard position of the ballast.
    :ivar ADC_FULL_PORT: (int) ADC value corresponding to the full port position of the ballast.
    :ivar ADC_MAX: (int) Maximum ADC value for the ballast position.
    :ivar ADC_MIN: (int) Minimum ADC value for the ballast position.
    :ivar MOTOR_FAST_STARBOARD: (int) PWM value to move the ballast fast towards starboard.
    :ivar MOTOR_FAST_PORT: (int) PWM value to move the ballast fast towards port.
    :ivar MOTOR_STOP: (int) PWM value to stop the ballast motor.
    :ivar CONTROL_FAST_PORT: (float) Control signal for fast movement towards port.
    :ivar CONTROL_FAST_STARBOARD: (float) Control signal for fast movement towards starboard.
    :ivar ERROR_MIN_MAGNITUDE: (int) Minimum magnitude of error necessary to actuate the ballast.
    :ivar RAMP_UP_TIME: (float) Time period to ramp up the motor to full speed.
    :ivar Kp: (float) Proportional gain for the PID controller.
    :ivar Kd: (float) Derivative gain for the PID controller (currently not in use).
    :ivar roll_kp: (int) Proportional gain for roll correction.
    :ivar current_target: (int) Current target ADC value for the ballast position.
    :ivar current_ballast_position: (int) Last known ADC value of the ballast position.
    :ivar move: (bool) Flag to determine if the ballast needs to be moved.
    :ivar current_wind_dir: (float) Current wind direction in degrees.
    :ivar autonomous_mode: (int) Current autonomous mode of the vessel.

    **Lifecycle States**:
    - **Configuring**: Sets up subscriptions and publishers.
    - **Activating**: Starts the control loop timer.
    - **Deactivating**: Stops the control loop timer.
    - **Cleaning up**: Shuts down publishers and subscriptions.
    - **Shutting down**: Performs additional cleanup as required.

    The control strategy involves responding to roll data and desired position changes, using a P controller to compute
    the necessary adjustments to the ballast position. The node uses a ramp-up mechanism to smooth out the control signals,
    preventing sudden movements that could destabilize the boat.

    **Callback Functions**:
    - **ballast_position_callback**: Handles updates to the desired ballast position.
    - **current_ballast_position_callback**: Updates the internal state with the current ballast position.
    - **airmar_roll_callback**: Responds to new roll data by adjusting the ballast position to counteract the roll.
    - **apparent_wind_callback**: Optional handling of wind data for more sophisticated control strategies.
    - **airmar_heading_callback**: Tracks the boat's heading to adjust control strategies based on directional changes.
    - **control_loop_callback**: The main control loop that calculates and sends PWM values to the ballast motor based on the PID control strategy.
    - **roll_correction_callback**: Periodically adjusts the target position based on accumulated roll errors to maintain stability.

    **Usage**:
    - The node must be managed by state_manager

    **Notes**:
    - Currently, moves the ballast every 10 seconds. A shorter period will improve response, but ballast movement is extremely expensive in terms of battery use.

    """
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
        """
        Callback function that processes roll data received from the Airmar sensor, adjusts the target roll based on wind direction,
        and stores the roll error for future analysis.

        This method is called automatically whenever a new roll data message is received. It calculates the desired roll target
        based on the relative wind angle, computes the error between the actual and target roll, and logs this error for control purposes.

        :param msg: The roll data message containing the current roll angle of the boat.
        :type msg: Float64

        **Key Operations:**

        - **Wind Direction Check**: Determines if the wind direction is known; if not, logs a message and exits.
        - **Relative Wind Calculation**: Computes the angle of the wind relative to the boat's current heading.
        - **Target Roll Setting**: Sets the roll target based on the relative wind angle:
            - 20 degrees if wind is from the starboard side.
            - -20 degrees if wind is from the port side.
        - **Error Buffer Management**: Ensures that only the most recent 'self.num_error_readings' errors are kept.

        **Notes:**
        - The function handles cases where the wind is directly behind or ahead by not changing the current roll target.
        
        """
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
        """
        Periodically adjusts the ballast position based on the target setpoint, handling the motor control based on the error magnitude
        and applying a ramp-up strategy for smoother operation.

        This function forms the control loop for ballast position adjustment. It calculates the error between the current and target
        positions, decides on the motor PWM value based using a proportional controller, and sends the command to the esp comms node. 
        The loop also incorporates a ramp-up mechanism to smoothly reach the desired position over a specified time.

        **Conditions and Steps**:

        - **No Movement**: If the system is not in a 'move' state, the function returns immediately.
        - **Small Error Handling**: Stops the motor if the current error is below a minimal threshold ('ERROR_MIN_MAGNITUDE').
        - **Ramp-Up Calculation**: If in ramp-up phase, scales the control effort based on the elapsed time from the start of the movement.
        - **Control Computation**: Calculates the motor control signal using proportional control (Kp), scaled by the ramp-up factor.
        - **Command Publishing**: Sends the computed PWM value to the motor via a ROS publisher.

        **Motor Control Strategy**:

        - The motor command is derived from a simple proportional control law adjusted by a ramp-up factor that ensures smooth transitions when starting movements, to avoid straining the timing belt, which has torn without this system.
        - The actual motor value is constrained within operational PWM bounds.

        **Note**:

        - This callback is scheduled to run at regular intervals by a ROS timer, ensuring periodic updates to the ballast position.
        - Stops the motor and resets the movement flag when the target is achieved within a small error margin, preventing oscillation around the target.
        - Currently, "ramp-down" is provided by the proportional controller. A more robust system would decide if ramp-up or ramp-down was needed based on the target position and current movement direction/velocity, but since this runs only every 10 seconds, it's not necessary right now.
        
        """
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

        #error_derivative = (current_error - self.previous_error) / delta_time
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