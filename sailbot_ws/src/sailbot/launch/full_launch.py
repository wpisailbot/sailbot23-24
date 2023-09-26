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
			executable='control_system.py',
			name='ctrl_sys'
		),
		Node(
			package='sailbot',
			executable='trim_tab_comms.py',
			name='trim_tab'
		),
		Node(
			package='sailbot',
			executable='debug_interface.py',
			name='debug'
		),
		Node(
			package='sailbot',
			executable='airmar_reader.py',
			name='airmar'
		),
		Node(
			package='sailbot',
			executable='serial_rc_receiver.py',
			name='rc_rcevr'
		)
	])
