from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yzz_waypoint_nav',
            executable='waypoint_nav_node',
            name='waypoint_nav_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('yzz_waypoint_nav'), 'config', 'yzz_waypoint_nav.yaml'
            ])],
        ),
    ])
