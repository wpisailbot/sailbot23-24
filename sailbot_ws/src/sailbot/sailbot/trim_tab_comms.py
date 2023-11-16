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
import websockets
import netifaces as ni
import threading

from std_msgs.msg import Int8, Int16, Float32, Empty

import trim_tab_messages.python.messages_pb2 as message_pb2

# Local variables
state = message_pb2._TrimState_TRIM_STATE.TrimState_TRIM_STATE_MIN_LIFT
angle = 0
wind_dir = 0.0
battery_level = 100


class TrimTabComms(LifecycleNode):
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
        #self.tt_control_subscriber = self.create_subscription(Int8, 'tt_control', self.listener_callback, 10)  # Trim tab state
        #self.tt_angle_subscriber = self.create_subscription(Int16, 'tt_angle', self.angle_callback, 10)

        self.timer_pub = self.create_lifecycle_publisher(
        Empty, '/heartbeat/trim_tab_comms', 1)
        #self.heartbeat_timer = self.create_timer(0.5, self.heartbeat_timer_callback)
        try:
            ip = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']
            self.get_logger().info(ip)
            start_server = websockets.serve(self.echo, ip, 8080)
            asyncio.get_event_loop().run_until_complete(start_server)
            tt_comms_thread = threading.Thread(asyncio.get_event_loop().run_forever(), daemon=True)
            tt_comms_thread.start()
        except Exception as e:
            self.get_logger().error()
        
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

    async def echo(self, websocket, path):
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

    tt_comms = TrimTabComms()

    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(tt_comms)

    try:
        # Spin the node to execute callbacks
        while rclpy.ok():
            executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        tt_comms.get_logger().fatal(f'Unhandled exception: {e}')
        raise(e)
    finally:
        # Shutdown and cleanup the node
        #tt_comms.disconnect()
        tt_comms.destroy_node()
        executor.shutdown()
        tt_comms.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
