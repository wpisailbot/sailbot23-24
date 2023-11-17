import launch
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    network_comms_node = LifecycleNode(
        package='sailbot', 
        executable='network_comms', 
        name='network_comms',
        namespace='',
        output='screen'
    )
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
    pwm_node = LifecycleNode(
        package='sailbot', 
        executable='pwm_controller', 
        name='pwm_controller',
        namespace='',
        output='screen'
    )
    tt_node = LifecycleNode(
        package='sailbot', 
        executable='trim_tab_comms', 
        name='trim_tab_comms',
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
    ld.add_action(network_comms_node)
    ld.add_action(ballast_node) 
    ld.add_action(pwm_node)
    ld.add_action(airmar_node)
    ld.add_action(tt_node)
    ld.add_action(state_manager_node)
    return ld
