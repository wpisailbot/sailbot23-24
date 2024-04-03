import launch
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

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
    # pwm_node = LifecycleNode(
    #     package='sailbot', 
    #     executable='pwm_controller', 
    #     name='pwm_controller',
    #     namespace='',
    #     output='screen'
    # )
    tt_node = LifecycleNode(
        package='sailbot', 
        executable='trim_tab_comms', 
        name='trim_tab_comms',
        namespace='',
        output='screen'
    )
    managed_node_names = DeclareLaunchArgument(
        'managed_nodes',
        default_value=["trim_tab_comms", "ballast_control"],
        description='The nodes lifecycle manager will control'
    )
    state_manager_node = Node(
        package='sailbot', 
        executable='state_manager', 
        name='state_manager',
        namespace='',
        output='screen',
        parameters=[{'managed_nodes': LaunchConfiguration('managed_nodes')}]
    )
    pathfinder_node = Node(
        package='sailbot_pathfinding', 
        executable='pathfinder_node', 
        name='pathfinder_node',
        namespace='',
        output='screen'
    )
    
    # Launch Description
    ld = launch.LaunchDescription()
    ld.add_action(managed_node_names)
    ld.add_action(network_comms_node)
    ld.add_action(ballast_node) 
    #ld.add_action(pwm_node)
    ld.add_action(airmar_node)
    ld.add_action(tt_node)
    ld.add_action(state_manager_node)
    ld.add_action(pathfinder_node)
    return ld
