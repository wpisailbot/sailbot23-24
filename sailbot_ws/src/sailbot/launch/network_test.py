from launch import LaunchDescription
from launch_ros.actions import Node

# Description:
# A launch file that simply launches all nodes


def generate_launch_description():
	return LaunchDescription([
		Node(
			package='sailbot',
			executable='pwm_controller',
			name='pwm'
		),
		Node(
			package='sailbot',
			executable='trim_tab_comms',
			name='trim_tab'
		),
		Node(
			package='sailbot',
			executable='network_comms',
			name='network_comms'
		),
		Node(
			package='sailbot',
			executable='airmar_reader',
			name='airmar'
		),
		Node(
			package='sailbot',
			executable='ballast_control',
			name='ballast_control'
		)
	])
