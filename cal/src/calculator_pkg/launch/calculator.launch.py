from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='calculator_pkg',
            executable='calculator_server',
            name='calculator_server',
            output='screen'
        ),
    ])
