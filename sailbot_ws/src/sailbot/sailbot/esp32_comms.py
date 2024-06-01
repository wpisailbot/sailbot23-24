#!/usr/bin/env python3

import rclpy
from typing import Optional
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.subscription import Subscription
from time import time as get_time

from std_msgs.msg import Int8, Int16, Empty, Float64, String
from sailbot_msgs.msg import Wind, AutonomousMode, GeoPath, TrimState

import serial
import json
import traceback
import serial.tools.list_ports
import subprocess

serial_port = '/dev/ttyTHS0'
baud_rate = 115200 

# Local variables
angle = 0
wind_dir = 0.0
battery_level = 100

def find_esp32_serial_ports() -> list:
    # Common VID:PID pairs for USB-to-Serial adapters used with ESP32
    esp32_vid_pid = [
        ('10C4', 'EA60'),  # Silicon Labs CP210x
        ('1A86', '7523'),  # HL-340
        ('0403', '6001'),  # FTDI
    ]
    
    ports = serial.tools.list_ports.comports()
    esp32_ports = []
    
    for port in ports:
        vid_pid = (hex(port.vid)[2:].upper(), hex(port.pid)[2:].upper()) if port.vid and port.pid else ('', '')
        if vid_pid in esp32_vid_pid:
            esp32_ports.append(port.device)
    
    return esp32_ports


class ESPComms(LifecycleNode):
    """
    A ROS2 Lifecycle Node for handling communications with the onboard ESP32 for rudder, trim tab, and ballast control.

    :ivar last_winds: Stores recent wind measurements for processing.
    :ivar autonomous_mode: Current mode of operation, based on 'AutonomousMode' message.
    :ivar force_neutral_position: Flag to keep the trim tab in a neutral position, regardless of wind conditions.
    :ivar could_be_tacking: Indicates if the boat is be performing a tacking maneuver.
    :ivar last_lift_state: Last state of the trim tab concerning lift, stored as a 'TrimState'.
    :ivar rudder_angle_limit_deg: Configurable limit for rudder angle to avoid extreme positions and stalls.
    """

    last_winds = []
    autonomous_mode = 0
    force_neutral_position = True
    could_be_tacking = False
    last_lift_state = TrimState.TRIM_STATE_MIN_LIFT
    rudder_angle_limit_deg = None

    request_tack_timer_duration = 3.0  # seconds
    request_tack_timer: Timer = None
    request_tack_override = False
    sent_clear_winds_this_tack = False

    def __init__(self):
        super(ESPComms, self).__init__('esp32_comms')

        self.set_parameters()
        self.get_parameters()

        self.tt_battery_publisher: Optional[Publisher]
        self.ballast_pos_publisher: Optional[Publisher]
        self.trim_state_debug_publisher: Optional[Publisher]

        self.error_publisher: Optional[Publisher]

        self.tt_control_subscriber: Optional[Subscription]
        self.tt_angle_subscriber: Optional[Subscription]
        self.rudder_angle_subscriber: Optional[Subscription]

        self.autonomous_mode_subscriber: Optional[Subscription]
        self.current_path_subscription: Optional[Subscription]
        self.apparent_wind_subscriber: Optional[Subscription]
        self.roll_subscription: Optional[Subscription]

        self.request_tack_subscription: Optional[Subscription]

        self.timer_pub: Optional[Publisher]

        self.heartbeat_timer: Optional[Timer]
        self.timer: Optional[Timer]

    def set_parameters(self) -> None:
        self.declare_parameter('sailbot.rudder.angle_limit_deg', 30)

    def get_parameters(self) -> None:
        self.rudder_angle_limit_deg = self.get_parameter('sailbot.rudder.angle_limit_deg').get_parameter_value().integer_value

    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")

        #reset ESP32 in case it stopped working from brownout
        esp32_ports = find_esp32_serial_ports()
        if esp32_ports:
            self.get_logger().info("ESP32 may be connected to the following ports:")
            for port in esp32_ports:
                print(port)
                try:
                    subprocess.run(['python3', '-m', 'esptool', '--port', port, 'run'], check=True)
                except Exception as e:
                    self.get_logger().error(f"ESP is not responding!")
                    raise(e)
        else:
            self.get_logger().warn("No ESP32 ports found!")

        self.ballast_pos_publisher = self.create_lifecycle_publisher(Int16, 'current_ballast_position', 10)

        self.tt_battery_publisher = self.create_lifecycle_publisher(Int8, 'tt_battery', 10)  # Battery level
        self.trim_state_debug_publisher = self.create_lifecycle_publisher(TrimState, 'trim_state', 10)

        self.error_publisher = self.create_lifecycle_publisher(String, f'{self.get_name()}/error', 10)


        self.tt_angle_subscriber = self.create_subscription(Int16, 'tt_angle', self.tt_angle_callback, 10)

        self.rudder_angle_subscriber = self.create_subscription(Int16, 'rudder_angle', self.rudder_angle_callback, 10)
        self.ballast_pwm_subscriber = self.create_subscription(Int16, 'ballast_pwm', self.ballast_pwm_callback, 10)

        self.apparent_wind_subscriber = self.create_subscription(Wind, 'apparent_wind_smoothed', self.apparent_wind_callback, 10)

        self.autonomous_mode_subscriber = self.create_subscription(AutonomousMode, 'autonomous_mode', self.autonomous_mode_callback, 10)
        self.current_path_subscription = self.create_subscription(
            GeoPath,
            'current_path',
            self.current_path_callback,
            10)
        
        self.roll_subscription = self.create_subscription(
            Float64,
            'airmar_data/roll',
            self.roll_callback,
            10)
        
        self.request_tack_subscription = self.create_subscription(
            Empty,
            'request_tack',
            self.request_tack_callback,
            10)
        

        self.timer_pub = self.create_lifecycle_publisher(Empty, '/heartbeat/trim_tab_comms', 1)
        
        self.ballast_timer = self.create_timer(0.01, self.ballast_timer_callback)

        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.05)
        except Exception as e:
            self.get_logger().info(str(e))
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
        self.destroy_lifecycle_publisher(self.tt_battery_publisher)
        self.destroy_lifecycle_publisher(self.ballast_pos_publisher)
        self.destroy_lifecycle_publisher(self.timer_pub)
        self.destroy_subscription(self.tt_control_subscriber)
        self.destroy_subscription(self.tt_angle_subscriber)
        self.destroy_timer(self.heartbeat_timer)
        self.destroy_timer(self.ballast_timer)

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        # Perform final cleanup if necessary
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().error("Error caught!")
        return super().on_error(state)
    
    #end callbacks

    def current_path_callback(self, msg: GeoPath) -> None:
        if len(msg.points) == 0:
            self.force_neutral_position = True
        else:
            self.get_logger().info("Valid path received, allowing auto trimtab movement")
            self.force_neutral_position = False

    def autonomous_mode_callback(self, msg: AutonomousMode) -> None:
        self.get_logger().info(f"Got autonomous mode: {msg.mode}")
        if(msg.mode == AutonomousMode.AUTONOMOUS_MODE_NONE):
            message = {
                "state": "manual",
            }
            trim_state_msg = TrimState()
            trim_state_msg.state = TrimState.TRIM_STATE_MANUAL
            self.trim_state_debug_publisher.publish(trim_state_msg)

        self.autonomous_mode = msg.mode

    def apparent_wind_callback(self, msg: Wind) -> None:
        #self.get_logger().info(f"Got apparent wind: {msg.direction}")
        self.find_trim_tab_state(msg.direction)

    def find_trim_tab_state(self, relative_wind) -> None:  # five states of trim
        """
        Determines the trim tab state based on the relative wind angle and current autonomous mode.
        
        This function sets the trim tab state to optimize the sail's position by adjusting the trim tab
        to either maximize lift or drag on the port or starboard side, or to minimize lift when "in irons" (directly into the wind).
        The function also handles state changes during tacking by detecting tacking signals from the rudder controller.

        :param relative_wind: The angle of the wind relative to the boat, in degrees.

        **Key Steps**:
        - **Mode Check**: Exits if the system is not in full autonomous or trim tab autonomous mode.
        - **State Determination**: Sets the trim tab state based on the relative wind angle.
        - **Tacking Adjustment**: Adjusts trim tab states during tacking maneuvers.
        - **Serial Communication**: Sends the determined state to the ESP32 for execution.

        **Trim Tab States**:
        - **Max Lift Port**: Applied when the relative wind is between 25 and 100 degrees.
        - **Max Drag Port**: Applied when the relative wind is between 100 and 180 degrees.
        - **Max Drag Starboard**: Applied when the relative wind is between 180 and 260 degrees.
        - **Max Lift Starboard**: Applied when the relative wind is between 260 and 335 degrees.
        - **Min Lift**: Default state, used when the wind angle does not match any other conditions or when in irons.

        **Behavior**:
        - Updates the trim tab state based on wind angle, adjusting for tacking if necessary.
        - Publishes the new state to a ROS topic for other components to utilize.
        - Handles exceptional cases where the boat might be in an unexpected state due to sudden wind changes.

        """
        #self.get_logger().info(f"apparent wind: {relative_wind}")
        
        # Check autonomous mode TODO: This is a coupling that shouldn't be necessary. 
        # Can be fixed by separating nodes and using lifecycle state transitions, or by finishing behavior tree
        autonomous_modes = AutonomousMode()
        #self.get_logger().info(f"Auto mode: {self.autonomous_mode}")
        if ((self.autonomous_mode != autonomous_modes.AUTONOMOUS_MODE_FULL) and (self.autonomous_mode != autonomous_modes.AUTONOMOUS_MODE_TRIMTAB)):
            #self.get_logger().info(f"Skipping")
            return

        msg = None
        trim_state_msg = TrimState()

        force_tack = False
        if(self.request_tack_override):
            # Only tack if we're going upwind
            if (25.0 <= relative_wind < 100) or (260 <= relative_wind < 335):
                force_tack = True
            else:
                self.get_logger().warn("Tack requested, but we're going downwind...")

        if 25.0 <= relative_wind < 100 and force_tack is False:
            # Max lift port
            msg = {
                "state": "max_lift_port"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MAX_LIFT_PORT
            self.last_lift_state = TrimState.TRIM_STATE_MAX_LIFT_PORT
            #self.get_logger().info("Max lift port")
        elif 100 <= relative_wind < 180:
            # Max drag port
            msg = {
                "state": "max_drag_starboard" # switched for testing, need to swap in trim_tab_client
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MAX_DRAG_PORT
            #self.last_state = TrimState.TRIM_STATE_MAX_DRAG_PORT
            #self.get_logger().info("Max drag port")

        elif 180 <= relative_wind < 260:
            # Max drag starboard
            msg = {
                "state": "max_drag_port"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MAX_DRAG_STARBOARD
            #self.last_state = TrimState.TRIM_STATE_MAX_DRAG_STARBOARD
            #self.get_logger().info("Max drag starboard")
        elif 260 <= relative_wind < 335 and force_tack is False:
            # Max lift starboard
            msg = {
                "state": "max_lift_starboard"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MAX_LIFT_STARBOARD
            self.last_lift_state = TrimState.TRIM_STATE_MAX_LIFT_STARBOARD
            self.get_logger().info("Max lift starboard")
        else:
            # Adjust behavior to not stop during a tack
            if(self.could_be_tacking or force_tack):
                self.get_logger().info("Tacking detected!")
                if(self.switched_sides_this_tack is False):
                    self.switched_sides_this_tack = True
                    if(self.last_lift_state == TrimState.TRIM_STATE_MAX_LIFT_STARBOARD):
                        trim_state_msg.state = TrimState.TRIM_STATE_MAX_LIFT_PORT
                        self.get_logger().info("Switching from starboard to port")
                        msg = {
                            "clear_winds": True,
                            "state": "max_lift_port"
                        }
                    elif (self.last_lift_state == TrimState.TRIM_STATE_MAX_LIFT_PORT):
                        self.get_logger().info("Switching from port to starboard")
                        trim_state_msg.state = TrimState.TRIM_STATE_MAX_LIFT_STARBOARD
                        msg = {
                            "clear_winds": True,
                            "state": "max_lift_starboard"
                        }
                    else:
                        # How did we get here?
                        self.get_logger().warn("Went into min lift in tack mode, but previous state was not max lift. Did the wind change suddenly?")
                        msg = {
                            "clear_winds": True,
                            "state": "min_lift"
                        }
                        trim_state_msg.state = TrimState.TRIM_STATE_MIN_LIFT
            else:
                msg = {
                    "state": "min_lift"
                }
                trim_state_msg.state = TrimState.TRIM_STATE_MIN_LIFT
                # Don't log min lift as last state so we never get stuck in it

        #if we're in full auto and have no target, don't go anywhere
        if self.force_neutral_position and self.autonomous_mode == AutonomousMode.AUTONOMOUS_MODE_FULL:
            msg = {
                "state": "min_lift"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MIN_LIFT
            self.get_logger().info("Force neutral")
        
        if(msg is not None):
            self.trim_state_debug_publisher.publish(trim_state_msg)
            message_string = json.dumps(msg)+'\n'
            self.ser.write(message_string.encode())
        else:
            self.get_logger().info("Trim message is None, taking no action")

    def tt_angle_callback(self, msg: Int16) -> None:
        self.get_logger().info("Sending trimtab angle")
        angle = msg.data
        this_time = get_time()
        message = {
            "state": "manual",
            "angle": angle,
            "timestamp": this_time
        }
        message_string = json.dumps(message)+'\n'
        self.ser.write(message_string.encode())

    def rudder_angle_callback(self, msg: Int16) -> None:
        """
        Callback function that processes received rudder angle data, applies constraints, and sends a corrected value to the ESP32.

        This function is triggered by a ROS subscription whenever a new rudder angle message is published. It checks the received
        angle against preset limits, adjusts the angle if necessary to prevent extreme positions, and then sends the corrected
        rudder angle to the ESP32 via serial communication.

        :param msg: A message containing the rudder angle as an integer.

        **Process**:

        - **Angle Limiting**: Checks if the received rudder angle exceeds preset limits (''self.rudder_angle_limit_deg'').
        - **Tacking Detection**: Sets a flag (''self.could_be_tacking'') if the rudder angle is beyond its limit, which heading_controller will use to indicate tacking.
        - **Serial Communication**: Sends the adjusted rudder angle to the ESP32 in a JSON formatted string over a serial connection.

        **Example of Serial Message**:

        - Sent: '{"rudder_angle": 20}'

        **Usage**:

        - The node must be managed by state_manager

        **Note**:
        
        - The function modifies the state variable ''self.could_be_tacking'' based on the rudder angle's relation to its limits.

        """

        #self.get_logger().info(f"Got rudder position: {msg.data}")
        degrees = msg.data
        # If rudder angles are high, limit them, and note that we could be tacking
        # This lets find_trim_tab_state adjust its behavior accordingly, if it would enter min_lift.
        if(degrees>self.rudder_angle_limit_deg):
            self.could_be_tacking = True
            degrees = self.rudder_angle_limit_deg
        elif (degrees<-self.rudder_angle_limit_deg):
            self.could_be_tacking = True
            degrees = -self.rudder_angle_limit_deg
        else:
            self.could_be_tacking = False
        #degrees = degrees+13 #Servo degree offset
        message = {
            "rudder_angle": degrees
        }
        message_string = json.dumps(message)+'\n'
        self.ser.write(message_string.encode())

    def ballast_pwm_callback(self, msg: Int16) -> None:
        #self.get_logger().info("Got ballast position")
        pwm = msg.data
        message = {
            "ballast_pwm": pwm
        }
        message_string = json.dumps(message)+'\n'
        self.ser.write(message_string.encode())
    
    def roll_callback(self, msg: Float64) -> None:
        msg = {
                "roll": msg.data
        }
        message_string = json.dumps(msg)+'\n'
        self.ser.write(message_string.encode())
    
    def request_tack_timer_callback(self):
        self.request_tack_override = False
        self.get_logger().info('Tack timer expired.')

        # Cancel the timer to clean up
        if self.request_tack_timer is not None:
            self.request_tack_timer.cancel()
            self.switched_sides_this_tack = False
            self.request_tack_timer = None

    def request_tack_callback(self, msg: Empty) -> None:
        self.request_tack_override = True
        if self.request_tack_timer is not None:
            self.request_tack_timer.cancel()
        self.request_tack_timer = self.create_timer(self.request_tack_timer_duration, self.request_tack_timer_callback)

    def ballast_timer_callback(self) -> None:
        """
        Timer callback function that sends a request to the ESP32 for the current position of the ballast and handles the response.
        
        This function formats a JSON message to request the ballast position, sends it via serial communication, reads the response,
        decodes the JSON data received, and publishes the ballast position. It handles possible exceptions like serial communication
        errors or JSON decoding errors and logs them accordingly.
        
        **Process**:

        - **Request**: Sends a JSON message with a request for ballast position.
        - **Response Handling**: Attempts to read and decode the response. If successful, publishes the ballast position.
        - **Error Handling**: Captures and logs errors related to serial communication or JSON decoding.
        - **Logging**: Logs messages indicating the status of data reception and errors.
        
        **Details**:

        - The request is sent periodically, triggered by a ROS timer.
        - If no data is received, or if there is an error in the data, logs this as an warning message.
        - Even if the potentiometer is reading strangely (position is 0), the position is published to indicate that a response was received.
        
        **Example of Serial Message**:

        - Sent: '{"get_ballast_pos": True}'
        - Received: '{"ballast_pos": 102}'

        """

        message = {
            "get_ballast_pos": True
        }
        message_string = json.dumps(message)+'\n'
        self.ser.write(message_string.encode())
        line = None
        try:
            line = self.ser.readline().decode('utf-8').rstrip()
        except:
            #serial corruption
            pass
        
        if line:
            try:
                message = json.loads(line)

                pos = Int16()
                pos.data = message["ballast_pos"]
                #self.get_logger().info(f"Received position: {pos.data}")
                if(pos.data == 0):
                    #self.get_logger().info("Ballast potentiometer is not working!")
                    pass
                else:
                    #self.get_logger().info(f"Ballast position: {pos.data}")
                    pass
                #publish even if it's broken, ballast_control will detect it
                self.ballast_pos_publisher.publish(pos)
            except json.JSONDecodeError:
                self.get_logger().warn("Error decoding JSON")
        else:
            self.get_logger().warn("No data received within the timeout period.")
    
    def publish_error(self, string: str):
        error_msg = String()
        error_msg.data = string
        self.error_publisher.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)

    esp_comms = ESPComms()

    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        ser.close()
    except Exception as e:
        trace = traceback.format_exc()
        error_string = f'Unhandled exception: {e}\n{trace}'
        esp_comms.get_logger().fatal(error_string)
        esp_comms.publish_error(error_string)
    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(esp_comms)

    executor.spin()
    esp_comms.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
