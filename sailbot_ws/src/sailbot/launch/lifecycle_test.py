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
    lifecycle_node = LifecycleNode(
        package='sailbot', 
        executable='ballast_control', 
        name='ballast_control',
        namespace='',
        output='screen'
    )
    
    emit_event_to_request_that_node_does_configure_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.process.matches_name(lifecycle_node.name),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    
    # Launch Description
    ld = launch.LaunchDescription()
    ld.add_action(lifecycle_node) 
    ld.add_action(emit_event_to_request_that_node_does_configure_transition)
    return ld
