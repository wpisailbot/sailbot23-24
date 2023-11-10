import launch
from launch import LaunchDescription
import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
import lifecycle_msgs.msg

def generate_launch_description():
    ballast_node = LifecycleNode(
        package='sailbot', 
        executable='ballast_control', 
        name='ballast_control',
        namespace='',
        output='screen'
    )
    airmar_node = LifecycleNode(
        package='sailbot', 
        executable='airmar_reader', 
        name='airmar_reader',
        namespace='',
        output='screen'
    )
    state_manager_node = LifecycleNode(
        package='sailbot', 
        executable='state_manager', 
        name='state_manager',
        namespace='',
        output='screen'
    )
    
    # Launch Description
    ld = launch.LaunchDescription()
    ld.add_action(ballast_node) 
    ld.add_action(airmar_node)
    ld.add_action(state_manager_node)
    return ld
