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
import asyncio
import websocket
import netifaces as ni
import threading

from std_msgs.msg import Int8, Int16, Float32, Empty

import trim_tab_messages.python.messages_pb2 as message_pb2

# Local variables
state = message_pb2.TRIM_STATE.TRIM_STATE_MIN_LIFT
angle = 0
wind_dir = 0.0
battery_level = 100


class TrimTabComms(LifecycleNode):
    last_websocket = None
    def __init__(self):
        super(TrimTabComms, self).__init__('trim_tab_comms')

        self.tt_telemetry_publisher: Optional[Publisher]
        self.tt_battery_publisher: Optional[Publisher]
        self.tt_control_subscriber: Optional[Subscription]
        self.tt_angle_subscriber: Optional[Subscription]
        self.timer_pub: Optional[Publisher]

        self.heartbeat_timer: Optional[Timer]
        self.timer: Optional[Timer]

    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")
        self.tt_telemetry_publisher = self.create_lifecycle_publisher(Float32, 'tt_telemetry', 10)  # Wind direction
        self.tt_battery_publisher = self.create_lifecycle_publisher(Int8, 'tt_battery', 10)  # Battery level
        self.tt_control_subscriber = self.create_subscription(Int8, 'tt_control', self.tt_state_callback, 10)  # Trim tab state
        self.tt_angle_subscriber = self.create_subscription(Int16, 'tt_angle', self.tt_angle_callback, 10)

        self.timer_pub = self.create_lifecycle_publisher(
        Empty, '/heartbeat/trim_tab_comms', 1)
        #self.heartbeat_timer = self.create_timer(0.5, self.heartbeat_timer_callback)
        self.get_data_timer = self.create_timer(0.5, self.data_timer_callback)
        self.ws = websocket.WebSocket()
        try:
            self.ws.connect("ws://sailbot-trimtab.local:8080/")
        except Exception as e:
            self.get_logger().info(e)
            self.get_logger().info("Failed to connect to trimtab! Is it on, and connected to a Jetson hotspot?")
            raise(e)
        self.ws.settimeout(1)
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
        self.destroy_lifecycle_publisher(self.tt_telemetry_publisher)
        self.destroy_lifecycle_publisher(self.tt_battery_publisher)
        self.destroy_lifecycle_publisher(self.timer_pub)
        self.destroy_subscription(self.tt_control_subscriber)
        self.destroy_subscription(self.tt_angle_subscriber)
        self.destroy_timer(self.heartbeat_timer)

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        # Perform final cleanup if necessary
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Error caught!")
        return super().on_error(state)
    
    #end callbacks

    def data_timer_callback(self):
        self.ws.send("test")
        try:
            response = self.ws.recv()
            data = message_pb2.DataMessage()
            data.ParseFromString(response)
            self.get_logger().info("windAngle: "+str(data.windAngle))
            self.get_logger().info("batteryLevel: "+str(data.batteryLevel))
        except websocket.WebSocketTimeoutException:
            print("No message received within the timeout period.")

    def schedule_async_function(self, coroutine):
        asyncio.run_coroutine_threadsafe(coroutine, asyncio.get_event_loop())

    def tt_state_callback(self, msg: Int8):
        protomsg = message_pb2.ControlMessage()
        protomsg.control_type = message_pb2.CONTROL_MESSAGE_CONTROL_TYPE.CONTROL_MESSAGE_CONTROL_TYPE_STATE
        protomsg.state = msg.data
        serialized_message = protomsg.SerializeToString()
        if(self.ws is not None):
            try:
                self.ws.send_binary(serialized_message)
            except Exception as e:
                self.get_logger().info(e)
                self.get_logger().info("Faild to send message")
    def tt_angle_callback(self, msg: Int16):
        protomsg = message_pb2.ControlMessage()
        protomsg.control_type = message_pb2.CONTROL_MESSAGE_CONTROL_TYPE.CONTROL_MESSAGE_CONTROL_TYPE_ANGLE
        protomsg.control_angle = msg.data
        serialized_message = protomsg.SerializeToString()
        if(self.ws is not None):
            try:
                self.ws.send_binary(serialized_message)
            except Exception as e:
                self.get_logger().info(e)
                self.get_logger().info("Faild to send message")
def main(args=None):
    rclpy.init(args=args)
    tt_comms = TrimTabComms()

    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(tt_comms)

    try:
        # Spin the node to execute callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        tt_comms.get_logger().fatal(f'Unhandled exception: {e}')
        raise(e)
    finally:
        tt_comms.ws.close()
        # Shutdown and cleanup the node
        executor.shutdown()
        tt_comms.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
