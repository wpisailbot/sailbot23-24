import launch
import os
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    map_name = DeclareLaunchArgument(
        'map_name',
        default_value="webster_medres",
        description="The text before the first ':' in the map file name"
    )
    config_file_path = os.path.join(
        get_package_share_directory('sailbot'),
        'config',
        'config.yaml'
    )
    network_comms_node = LifecycleNode(
        package='sailbot', 
        executable='network_comms', 
        name='network_comms',
        namespace='',
        output='screen',
        parameters=[{'map_name': LaunchConfiguration('map_name'), 'managed_nodes': ["ballast_control", "wind_smoother", "airmar_reader", "path_follower", "heading_controller", "esp32_comms"]}]
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
    cv_node = Node(
        package='sailbot',
        executable='buoy_detection',
        name='buoy_detection',
        namespace='',
        output='screen',
    )
    # pwm_node = LifecycleNode(
    #     package='sailbot', 
    #     executable='pwm_controller', 
    #     name='pwm_controller',
    #     namespace='',
    #     output='screen'
    # )
    esp_node = LifecycleNode(
        package='sailbot', 
        executable='esp32_comms', 
        name='esp32_comms',
        namespace='',
        output='screen',
        parameters=[config_file_path]
    )
    heading_node = LifecycleNode(
        package='sailbot', 
        executable='heading_controller', 
        name='heading_controller',
        namespace='',
        output='screen',
        parameters=[config_file_path]
    )
    path_follower_node = LifecycleNode(
        package='sailbot', 
        executable='path_follower', 
        name='path_follower',
        namespace='',
        output='screen',
        parameters=[config_file_path, {'map_name': LaunchConfiguration('map_name')}]
    )
    # managed_node_names = DeclareLaunchArgument(
    #     'managed_nodes',
    #     default_value=["path_follower", "heading_controller"],
    #     description='The nodes lifecycle manager will control'
    # )
    state_manager_node = Node(
        package='sailbot', 
        executable='state_manager', 
        name='state_manager',
        namespace='',
        output='screen',
        parameters=[{'managed_nodes': ["ballast_control", "wind_smoother", "airmar_reader", "path_follower", "heading_controller", "esp32_comms"]}]
    )
    pathfinder_node = Node(
        package='sailbot_pathfinding', 
        executable='pathfinder_node', 
        name='pathfinder_node',
        namespace='',
        output='screen'
    )

    wind_smoother_node = Node(
        package='sailbot',
        executable='wind_smoother',
        name='wind_smoother',
        namespace='',
        output='screen'
    )
    
    
    # Launch Description
    ld = launch.LaunchDescription()
    #ld.add_action(managed_node_names)
    ld.add_action(map_name)
    ld.add_action(network_comms_node)
    ld.add_action(ballast_node) 
    #ld.add_action(pwm_node)
    ld.add_action(airmar_node)
    ld.add_action(cv_node)
    ld.add_action(wind_smoother_node)

    ld.add_action(esp_node)
    ld.add_action(heading_node)
    ld.add_action(path_follower_node)

    ld.add_action(state_manager_node)
    ld.add_action(pathfinder_node)
    return ld
