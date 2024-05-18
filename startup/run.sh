#!/bin/bash
echo "Sourcing environment"
source /home/sailbot/ros2_humble/install/setup.bash
source /home/sailbot/ros2_ws/install/local_setup.bash
source /home/sailbot/sailbot23-24/sailbot_ws/install/local_setup.bash
echo "Launching server"
python3 /opt/sailbot/startup_server.py
