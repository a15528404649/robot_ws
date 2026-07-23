from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(package="yzz_battery_adapter", executable="battery_adapter", name="battery_adapter", output="screen",
             parameters=[PathJoinSubstitution([FindPackageShare("yzz_battery_adapter"), "config", "battery_adapter.yaml")]),
    ])
