from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yzz_detect',
            executable='yolo_detect',
            name='yolo_detect',
            output='screen',
            respawn=True,
        ),
    ])
