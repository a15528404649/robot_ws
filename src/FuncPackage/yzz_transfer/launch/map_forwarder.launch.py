from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yzz_transfer',
            executable='map_and_scan_forwarder',
            name='map_and_scan_forwarder',
            output='screen',
        ),
    ])
