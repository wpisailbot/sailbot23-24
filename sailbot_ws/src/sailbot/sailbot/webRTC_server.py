#!/usr/bin/env python3
import asyncio
import rclpy
from typing import Optional
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.subscription import Subscription
import asyncio
from sensor_msgs.msg import Image



class WebRTCServer(LifecycleNode):
    last_websocket = None
    def __init__(self):
        super(WebRTCServer, self).__init__('trim_tab_comms')
        self.zed_image_subscriber: Optional[Subscription]
        self.async_timer: Optional[Timer]

    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")
        self.zed_image_subscriber = self.create_subscription(Image, '/zed/zed_node/stereo/image_rect_color', self.zed_image_callback, 10)  # Trim tab state
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
        self.destroy_subscription(self.zed_image_subscriber)
        self.destroy_timer(self.async_timer)

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        # Perform final cleanup if necessary
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Error caught!")
        return super().on_error(state)
    
    #end callbacks

    def zed_image_callback(self, image: Image):
        pass
        



#TODO: Horrible hack to allow perpetual async alongside ROS. This creates a bunch of unnecessary looping
#Find some way to replace this.
async def spin(executor: rclpy.executors.SingleThreadedExecutor, logger):
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)
        await asyncio.sleep(0)
        logger.info("looping")

def main(args=None):
    rclpy.init(args=args)

    webrtc_server = WebRTCServer()
    loop = asyncio.get_event_loop()
    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(webrtc_server)

    loop.run_until_complete(spin(executor, webrtc_server.get_logger()))
    webrtc_server.destroy_node()
    executor.shutdown()
    webrtc_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
