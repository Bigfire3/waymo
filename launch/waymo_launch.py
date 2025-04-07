from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node from the py_state_machine package
        Node(
            package='py_state_machine',
            executable='py_state_manager',
            name='py_state_manager',
            output='screen'
        ),
        
        # Node from the line_follower package
        Node(
            package='line_follower',
            executable='drive_with_line_follower',
            name='drive_with_line_follower',
            output='screen'
        ),
        
    ])