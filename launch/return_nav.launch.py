from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Combined path recorder and navigator node
        Node(
            package='go',
            executable='path_recorder_navigator',
            name='path_recorder_navigator',
            output='screen',
            emulate_tty=True,
        )
    ])