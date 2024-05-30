sudo mkdir -p /opt/sailbot
sudo cp -r startup/* /opt/sailbot/
chmod a+x /opt/sailbot/run.sh
sudo cp startup/systemd/ros2_control_server.service /etc/systemd/system/ros2_control_server.service
sudo systemctl daemon-reload
sudo systemctl enable ros2_control_server.service
sudo systemctl start ros2_control_server.service
sudo systemctl status ros2_control_server.service
