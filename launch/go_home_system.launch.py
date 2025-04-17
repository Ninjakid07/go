from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Position tracker node
        Node(
            package='go',
            executable='position_tracker',
            name='position_tracker',
            output='screen',
            emulate_tty=True,
        ),
        
        # Return home node
        Node(
            package='go',
            executable='return_home',
            name='return_home',
            output='screen',
            emulate_tty=True,
        )
    ])