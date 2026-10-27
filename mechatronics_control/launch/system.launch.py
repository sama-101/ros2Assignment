from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # Start turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            output='screen'
        ),

        # Start GoToPose Action Server and speed parameters
        Node(
            package='mechatronics_control',
            executable='go_to_pose_server',
            parameters=[{
                'max_linear_speed': 0.8,
                'max_angular_speed': 1.2
            }],
            output='screen'
        ),

        # Start Reset Service
        Node(
            package='mechatronics_control',
            executable='reset_service',
            output='screen'
        ),

        # Start Square Action Server
        Node(
            package='mechatronics_control',
            executable='square_action_server',
            output='screen'
        ),

    ])