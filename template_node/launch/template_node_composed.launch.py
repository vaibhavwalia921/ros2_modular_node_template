"""
Composed launch file for running multiple instances or with additional nodes
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the nodes'
    )
    
    use_composition = DeclareLaunchArgument(
        'use_composition',
        default_value='true',
        description='Use composed nodes for better performance'
    )
    
    config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('template_node'),
            'config',
            'default_params.yaml'
        ]),
        description='Path to the parameter file'
    )
    
    # Composable node container
    container = ComposableNodeContainer(
        name='template_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='template_node',
                plugin='template_node::middleware::LifecycleNodeWrapper',
                name='template_node',
                parameters=[LaunchConfiguration('config_file')],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
    )
    
    # Additional monitoring node
    monitor_node = Node(
        package='template_node',
        executable='template_node_monitor',
        name='template_monitor',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'monitored_node': 'template_node',
            'check_frequency_hz': 1.0
        }]
    )
    
    # Group actions
    grouped_actions = GroupAction([
        container,
        monitor_node
    ])
    
    return LaunchDescription([
        namespace,
        use_composition,
        config_file,
        grouped_actions
    ])