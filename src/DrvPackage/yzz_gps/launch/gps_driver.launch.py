from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="yzz_gps",
            executable="gps_driver",
            name="gps_driver",
            output="screen",
            parameters=[PathJoinSubstitution([
                FindPackageShare("yzz_gps"), "config", "gps.yaml"
            ])],
        )
    ])
