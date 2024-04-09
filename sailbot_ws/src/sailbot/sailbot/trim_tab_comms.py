#!/usr/bin/env python3
import struct
import asyncio
import os

import rclpy
from typing import Optional
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.subscription import Subscription
from enum import Enum
from time import time as get_time

from std_msgs.msg import Int8, Int16, Float32, Empty
from sailbot_msgs.msg import Wind, AutonomousMode, Path, TrimState

import trim_tab_messages.python.messages_pb2 as message_pb2

import serial
import json
import time
import traceback
import serial.tools.list_ports
import subprocess

serial_port = '/dev/ttyTHS0'
baud_rate = 115200 

# Local variables
state = message_pb2.TRIM_STATE.TRIM_STATE_MIN_LIFT
angle = 0
wind_dir = 0.0
battery_level = 100

def find_esp32_serial_ports():
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


class TrimTabComms(LifecycleNode):
    last_websocket = None
    last_winds = []
    autonomous_mode = 0
    force_neutral_position = True

    def __init__(self):
        super(TrimTabComms, self).__init__('trim_tab_comms')

        self.tt_telemetry_publisher: Optional[Publisher]
        self.tt_battery_publisher: Optional[Publisher]
        self.ballast_pos_publisher: Optional[Publisher]
        self.trim_state_debug_publisher: Optional[Publisher]

        self.tt_control_subscriber: Optional[Subscription]
        self.tt_angle_subscriber: Optional[Subscription]
        self.rudder_angle_subscriber: Optional[Subscription]

        self.autonomous_mode_subscriber: Optional[Subscription]
        self.current_path_subscription: Optional[Subscription]
        self.apparent_wind_subscriber: Optional[Subscription]

        self.timer_pub: Optional[Publisher]

        self.heartbeat_timer: Optional[Timer]
        self.timer: Optional[Timer]

    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")

        #reset ESP32 in case it stopped working from brownout
        esp32_ports = find_esp32_serial_ports()
        if esp32_ports:
            self.get_logger().info("ESP32 may be connected to the following ports:")
            for port in esp32_ports:
                print(port)
                subprocess.run(['python3', '-m', 'esptool', '--port', port, 'run'], check=True)

        self.tt_telemetry_publisher = self.create_lifecycle_publisher(Float32, 'tt_telemetry', 10)
        self.ballast_pos_publisher = self.create_lifecycle_publisher(Int16, 'current_ballast_position', 10)

        self.tt_battery_publisher = self.create_lifecycle_publisher(Int8, 'tt_battery', 10)  # Battery level
        self.trim_state_debug_publisher = self.create_lifecycle_publisher(TrimState, 'trim_state', 10)

        self.tt_control_subscriber = self.create_subscription(Int8, 'tt_control', self.tt_state_callback, 10)  # Trim tab state
        self.tt_angle_subscriber = self.create_subscription(Int16, 'tt_angle', self.tt_angle_callback, 10)

        self.rudder_angle_subscriber = self.create_subscription(Int16, 'rudder_angle', self.rudder_angle_callback, 10)
        self.ballast_pwm_subscriber = self.create_subscription(Int16, 'ballast_pwm', self.ballast_pwm_callback, 10)

        self.apparent_wind_subscriber = self.create_subscription(Wind, 'apparent_wind_smoothed', self.apparent_wind_callback, 10)

        self.autonomous_mode_subscriber = self.create_subscription(AutonomousMode, 'autonomous_mode', self.autonomous_mode_callback, 10)
        self.current_path_subscription = self.create_subscription(
            Path,
            'current_path',
            self.current_path_callback,
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
        self.destroy_lifecycle_publisher(self.tt_telemetry_publisher)
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

    def current_path_callback(self, msg: Path):
        if len(msg.points) == 0:
            self.force_neutral_position = True
        else:
            self.get_logger().info("Valid path received, allowing auto trimtab movement")
            self.force_neutral_position = False

    def autonomous_mode_callback(self, msg: AutonomousMode):
        self.get_logger().info(f"Got autonomous mode: {msg.mode}")
        if(msg.mode == AutonomousMode.AUTONOMOUS_MODE_NONE):
            message = {
                "state": "manual",
            }
            trim_state_msg = TrimState()
            trim_state_msg.state = TrimState.TRIM_STATE_MANUAL
            self.trim_state_debug_publisher.publish(trim_state_msg)

        self.autonomous_mode = msg.mode

    def apparent_wind_callback(self, msg: Wind):
        #self.get_logger().info(f"Got apparent wind: {msg.direction}")
        self.find_trim_tab_state(msg.direction)

    def find_trim_tab_state(self, relative_wind):  # five states of trim
        #self.get_logger().info(f"apparent wind: {relative_wind}")
        #self.update_winds(relative_wind)
        #smooth_angle = self.median(self.last_winds)

        
        # Check autonomous mode TODO: This is a coupling that shouldn't be necessary. 
        # Can be fixed by separating nodes and using lifecycle state transitions, or by finishing behavior tree
        autonomous_modes = AutonomousMode()
        #self.get_logger().info(f"Auto mode: {self.autonomous_mode}")
        if ((self.autonomous_mode != autonomous_modes.AUTONOMOUS_MODE_FULL) and (self.autonomous_mode != autonomous_modes.AUTONOMOUS_MODE_TRIMTAB)):
            #self.get_logger().info(f"Skipping")
            return

        msg = None
        trim_state_msg = TrimState()
        if 45.0 <= relative_wind < 135:
            # Max lift port
            msg = {
                "state": "max_lift_port"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MAX_LIFT_PORT
            self.get_logger().info("Max lift port")
        elif 135 <= relative_wind < 180:
            # Max drag port
            msg = {
                "state": "max_drag_port"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MAX_DRAG_PORT
            self.get_logger().info("Max drag port")

        elif 180 <= relative_wind < 200:
            # Max drag starboard
            msg = {
                "state": "max_drag_starboard"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MAX_DRAG_STARBOARD
            self.get_logger().info("Max drag starboard")
        elif 200 <= relative_wind < 315:
            # Max lift starboard
            msg = {
                "state": "max_lift_starboard"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MAX_LIFT_STARBOARD
            self.get_logger().info("Max lift starboard")
        else:
            # In irons, min lift
            msg = {
                "state": "min_lift"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MIM_LIFT
            self.get_logger().info("Min lift")
        #if we're in full auto and have no target, don't go anywhere
        if self.force_neutral_position and self.autonomous_mode == AutonomousMode.AUTONOMOUS_MODE_FULL:
            msg = {
                "state": "min_lift"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MIN_LIFT
            self.get_logger().info("Force neutral")
        
        self.trim_state_debug_publisher.publish(trim_state_msg)
        message_string = json.dumps(msg)+'\n'
        self.ser.write(message_string.encode())

    def tt_state_callback(self, msg: Int8):
        protomsg = message_pb2.ControlMessage()
        protomsg.control_type = message_pb2.CONTROL_MESSAGE_CONTROL_TYPE.CONTROL_MESSAGE_CONTROL_TYPE_STATE
        protomsg.state = msg.data
        serialized_message = protomsg.SerializeToString()
        if(self.last_websocket is not None):
            self.schedule_async_function(self.last_websocket.send(serialized_message))

    def tt_angle_callback(self, msg: Int16):
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

    def rudder_angle_callback(self, msg: Int16):
        self.get_logger().info(f"Got rudder position: {msg.data}")
        degrees = msg.data+13 #Servo degree offset

        message = {
            "rudder_angle": degrees
        }
        message_string = json.dumps(message)+'\n'
        self.ser.write(message_string.encode())

    def ballast_pwm_callback(self, msg: Int16):
        #self.get_logger().info("Got ballast position")
        pwm = msg.data
        message = {
            "ballast_pwm": pwm
        }
        message_string = json.dumps(message)+'\n'
        self.ser.write(message_string.encode())

    def ballast_timer_callback(self):
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

                #self.get_logger().info("Received position:", message["ballast_pos"])
                pos = Int16()
                pos.data = message["ballast_pos"]
                if(pos.data == 0):
                    self.get_logger().info("Ballast potentiometer is not working!")
                    pass
                else:
                    #self.get_logger().info(f"Ballast position: {pos.data}")
                    pass
                #publish even if it's broken, ballast_control will detect it
                self.ballast_pos_publisher.publish(pos)
            except json.JSONDecodeError:
                self.get_logger().info("Error decoding JSON")
        else:
            self.get_logger().info("No data received within the timeout period.")

    async def echo(self, websocket, path):
        self.last_websocket = websocket
        async for message in websocket:
            try:
                # Deserialize the protobuf message
                protobuf_message = message_pb2.DataMessage()
                protobuf_message.ParseFromString(message)

                self.get_logger().info("Received message:"+ str(protobuf_message.windAngle)+": "+str(protobuf_message.batteryLevel))

                # Optionally, send a response back (as a string or protobuf)
                await websocket.send("Message received")
                
            except Exception as e:
                self.get_logger().info("Error processing message:", e)
                raise(e)

def main(args=None):
    rclpy.init(args=args)

    loop = asyncio.get_event_loop()
    tt_comms = TrimTabComms()

    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        ser.close()
    except Exception as e:
        trace = traceback.format_exc()
        tt_comms.get_logger().fatal(f'Unhandled exception: {e}\n{trace}')
    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(tt_comms)

    executor.spin()
    tt_comms.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
