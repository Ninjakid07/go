from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='go',
            executable='position_tracker',
            name='position_tracker',
            output='screen',
            emulate_tty=True,
        )
    ])