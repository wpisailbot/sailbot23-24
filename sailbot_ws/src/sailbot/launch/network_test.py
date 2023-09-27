from launch import LaunchDescription
from launch_ros.actions import Node

# Description:
# A launch file that simply launches all nodes


def generate_launch_description():
	return LaunchDescription([
		Node(
			package='sailbot',
			executable='network_comms.py',
			name='network_comms'
		)
	])
