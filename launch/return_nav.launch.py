from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Path recorder node
        Node(
            package='go',
            executable='path_rec',
            name='path_recorder',
            output='screen',
            emulate_tty=True,
        ),
        
        # Reverse waypoint follower node
        Node(
            package='go',
            executable='rev_waypoint',
            name='waypoint_follower',
            output='screen',
            emulate_tty=True,
        )
    ])