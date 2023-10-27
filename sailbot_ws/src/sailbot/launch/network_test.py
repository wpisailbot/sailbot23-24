from launch import LaunchDescription
from launch_ros.actions import Node

# Description:
# A launch file that simply launches all nodes


def generate_launch_description():
	return LaunchDescription([
		Node(
			package='sailbot',
			executable='pwm_controller.py',
			name='pwm'
		),
		Node(
			package='sailbot',
			executable='trim_tab_comms.py',
			name='trim_tab'
		),
		Node(
			package='sailbot',
			executable='network_comms.py',
			name='network_comms'
		)
	])
