# Sailbot

This ROS2 package contains the primary nodes for WPI's Sailbot. These nodes are written in Python3, and currently target [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html).

This package leverages ROS2's [managed lifecycle nodes](https://design.ros2.org/articles/node_lifecycle.html) (new to RCLPY with Ros2 Humble) to maintain deliberate control over the lifecycle of the various running nodes.

Currently, there are ten nodes in this package, of which six have been ported to lifecycle nodes. The nodes

-`airmar_reader`
-`ballast_control`
-`network_comms`
-`pwm_controller`
-`trim_tab_comms`
-`webRTC_server`

are lifecycle nodes, whose lifecycle state is managed by the `state_manager` node. Upon launch, the `network_comms` node is cycled into its active state. Once successful, the rest of the lifecycle nodes are transitioned in parallel, with their status monitored and communicated to any active telemetry clients by `network_comms`.

The remaining nodes

-`battery_monitor`
-`computer_vision`
-`control_system`

have not been fully reimplemented, and are not yet managed.

The function of each of the nodes is described below:

-`airmar_reader`: Reads NMEA 0183 messages from a [Maretron NMEA 2000 usb gateway](https://www.maretron.com/products/usb100-nmea-2000-usb-gateway/), which translates NMEA 2000 messages from the onboard [Airmar 220WX weatherstation](https://www.airmar.com/Product/220WX)
-`ballast_control`: Implements a P controller for the sliding-rail ballast
-`network_comms`: Runs a gRPC server to interface with active [telemetry clients](https://github.com/wpisailbot/sailbot_telemetry_flutter)
-`pwm_controller`: Controls ballast motor speed and rudder angle through a PWM hat attached to the Jetson Nano's GPIO interface
-`trim_tab_comms`: Runs a websocket server on a wifi AP to interface with the [trim-tab controller](https://github.com/wpisailbot/trim_tab_client) in the wingsail
-`webRTC_server`