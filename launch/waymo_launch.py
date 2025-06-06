from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waymo',
            executable='gui_debug_node',
            name='gui_debug_node',
            output='screen'
        ),

        Node(
            package='waymo',
            executable='state_manager_node',
            name='state_manager_node',
            output='screen'
        ),

        Node(
            package='waymo',
            executable='obstacle_detection_node',
            name='obstacle_detection_node',
            output='screen'
        ),

        Node(
            package='waymo',
            executable='lane_detection_node',
            name='lane_detection_node',
            output='screen'
        ),

        Node(
            package='waymo',
            executable='passing_obstacle_node',
            name='passing_obstacle_node',
            output='screen'
        ),

        Node(
            package='waymo',
            executable='traffic_light_detection_node',
            name='traffic_light_detection_node',
            output='screen'
        ),

        Node(
            package='waymo',
            executable='sign_detection_node',
            name='sign_detection_node',
            output='screen'
        ),

        Node(
            package='waymo',
            executable='parking_node',
            name='parking_node',
            output='screen'
        ),
        Node(
            package='waymo',
            executable='speed_governor_node',
            name='speed_governor_node',
            output='screen'
        ),
    ])
