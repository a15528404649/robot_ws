from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("yzz_transfer"), "config", "yzz_transfer.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("start_gps", default_value="false", description="Start yzz_gps with transfer nodes; leave false when GPS is already running."),
        Node(package="yzz_gps", executable="gps_driver", name="gps_driver", output="screen", condition=IfCondition(LaunchConfiguration("start_gps"))),
        Node(package="yzz_transfer", executable="map_and_scan_forwarder", name="map_and_scan_forwarder", output="screen"),
        Node(package="yzz_transfer", executable="heartbeat", name="heartbeat", output="screen", parameters=[config]),
        Node(package="yzz_transfer", executable="result", name="result", output="screen", parameters=[config]),
        Node(package="yzz_transfer", executable="env_data", name="env_data", output="screen", parameters=[config]),
        Node(package="yzz_transfer", executable="http_transmitter", name="http_transmitter", output="screen", parameters=[config]),
    ])
