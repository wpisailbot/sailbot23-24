#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Float64, String

# import json

import math
import traceback

from typing import Optional
from rclpy.node import Node
from rclpy.timer import Timer

from sensor_msgs.msg import MagneticField

class HeadingSelect(Node):

    use_camera_heading = False

    def __init__(self):
        super().__init__('heading_select')

        self.set_parameters()
        self.get_parameters()

        self.heading_publisher = self.create_publisher(Float64, 'heading', 10)
        self.error_publisher = self.create_publisher(String, f'{self.get_name()}/error', 10)
        
        self.airmar_heading_subscription = self.create_subscription(
            Float64,
            '/airmar_data/heading',
            self.airmar_heading_callback,
            10)
        self.camera_heading_subscription = self.create_subscription(
            MagneticField,
            '/zed/zed_node/imu/mag',
            self.magnetic_field_callback,
            10)

        self.timer: Optional[Timer]

    def set_parameters(self) -> None:
        self.declare_parameter('sailbot.heading_select.use_camera_heading', False)

    def get_parameters(self) -> None:
        self.use_camera_heading = self.get_parameter('sailbot.heading_select.use_camera_heading').get_parameter_value().bool_value

    def airmar_heading_callback(self, msg: Float64) -> None:
        if(self.use_camera_heading == False):
            self.heading = msg.data
            heading_msg = Float64()
            heading_msg.data = msg.data
            self.heading_publisher.publish(heading_msg)

    def magnetic_field_callback(self, msg: MagneticField):
        if(self.use_camera_heading == True):
            # Extract magnetic field components
            x = msg.magnetic_field.x
            y = msg.magnetic_field.y

            # Compute heading
            heading = math.atan2(y, x)
            
            # Convert heading to degrees
            heading_degrees = math.degrees(heading)
            
            # Normalize heading to [0, 360) degrees
            if heading_degrees < 0:
                heading_degrees += 360

            self.heading = heading_degrees

            heading_msg = Float64()
            heading_msg.data = heading_degrees
            self.heading_publisher.publish(heading_msg)

            self.get_logger().info("Heading: {:.2f} degrees".format(heading_degrees))

    def publish_error(self, string: str):
        error_msg = String()
        error_msg.data = string
        self.error_publisher.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    heading_select = HeadingSelect()

    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(heading_select)

    try:
        # Spin the node to execute callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        trace = traceback.format_exc()
        error_string = f'Unhandled exception: {e}\n{trace}'
        heading_select.get_logger().fatal(error_string)
        heading_select.publish_error(error_string)
    finally:
        # Shutdown and cleanup the node
        executor.shutdown()
        heading_select.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()