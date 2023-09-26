from launch import LaunchDescription
from launch_ros.actions import Node

# Description:
# A launch file that launches all nodes and opens
# different terminal window to display each node's
# callback messages


def generate_launch_description():
	return LaunchDescription([
		# Prefix opens new terminal window
		# Output displays callback messages in terminal
		Node(
			package='sailbot',
			node_executable='pwm_controller',
			name='pwm',
			prefix='gnome-terminal --command',
			output='screen'
		),
		Node(
			package='sailbot',
			node_executable='control_system',
			name='ctrl_sys',
			prefix='gnome-terminal --command',
			output='screen'
		),
		Node(
			package='sailbot',
			node_executable='trim_tab_comms',
			name='trim_tab',
			prefix='gnome-terminal --command',
			output='screen'
		),
		Node(
			package='sailbot',
			node_executable='debug_interface',
			name='debug',
			prefix='gnome-terminal --command',
			output='screen'
		),
		Node(
			package='sailbot',
			node_executable='airmar_reader',
			name='airmar',
			prefix='gnome-terminal --command',
			output='screen'
		),
		Node(
			package='sailbot',
			node_executable='serial_rc_receiver',
			name='rc_rcevr',
			prefix='gnome-terminal --command',
			output='screen'
		)
	])
