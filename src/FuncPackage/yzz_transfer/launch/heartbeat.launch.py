from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="yzz_transfer", executable="heartbeat", name="heartbeat", output="screen",
            parameters=[PathJoinSubstitution([FindPackageShare("yzz_transfer"), "config", "yzz_transfer.yaml"])],
        ),
    ])
