from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yzz_detect_pest',
            executable='yzz_detect_pest',
            name='yzz_detect_pest',
            output='screen',
            respawn=True,
        ),
    ])
