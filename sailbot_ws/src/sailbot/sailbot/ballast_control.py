#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String, Float64
import json
import board
import busio
import time
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
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
    ADC_FULL_STARBOARD = 20480
    ADC_FULL_PORT = 10016
    
    MOTOR_FAST_STARBOARD = 130
    MOTOR_FAST_PORT = 60

    CONTROL_FAST_PORT=-1.0
    CONTROL_FAST_STARBOARD=1.0

    Kp = 0.0005

    current_target = (ADC_FULL_STARBOARD-ADC_FULL_PORT)/2+ADC_FULL_PORT

    def __init__(self):
        super().__init__('ballast_control')
        self.pwm_control_publisher: Optional[Publisher]
        self.position_subscription: Optional[Subscription]
        self.timer: Optional[Timer]

    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")
        try:
            # Create the I2C bus
            i2c = busio.I2C(board.SCL, board.SDA)
            # Create the ADC object using the I2C bus
            ads = ADS.ADS1015(i2c)
            # Create single-ended input on channel 0
            self.ballast_adc_channel = AnalogIn(ads, ADS.P0)
        except:
            return TransitionCallbackReturn.FAILURE

        self.pwm_control_publisher = self.create_lifecycle_publisher(String, 'pwm_control', 10)

        self.position_subscription = self.create_subscription(
            Float64,
            'ballast_position',
            self.ballast_position_callback,
            10)
        
        self.timer = self.create_timer(0.1, self.control_loop_callback)
        self.get_logger().info("ADC node configured")
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
        self.get_logger().info("Received ballast position: "+str(msg.data))
        self.current_target = self.ADC_FULL_PORT + ((self.ADC_FULL_STARBOARD - self.ADC_FULL_PORT) / (1.0 - -1.0)) * (msg.data - -1.0)

    def control_loop_callback(self):
        motor_value = self.control_to_motor_value(self.constrain_control(self.Kp*(self.ballast_adc_channel.value-self.current_target)))
        #self.get_logger().info("Current target: "+str(self.current_target) + " Current position: "+str(self.ballast_adc_channel.value)+" Current motor value: "+str(motor_value))
        
        ballast_json = {"channel": "12", "angle": motor_value}
        self.pwm_control_publisher.publish(make_json_string(ballast_json))
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