from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='shelfbot',
            executable='orchestrator',
            name='orchestrator',
            output='screen',
        ),
    ])
