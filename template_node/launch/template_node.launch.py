from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
import launch

def generate_launch_description():
    # Declare arguments
    config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('template_node'),
            'config',
            'default_params.yaml'
        ]),
        description='Path to the parameter file'
    )
    
    # Create lifecycle node
    template_node = LifecycleNode(
        package='template_node',
        executable='template_node_main',
        name='template_node',
        namespace='',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )
    
    # Configure node after launch
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(template_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )
    
    # Activate node after configuration
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=template_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(template_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )
    
    return LaunchDescription([
        config_file,
        template_node,
        configure_event,
        activate_event
    ])