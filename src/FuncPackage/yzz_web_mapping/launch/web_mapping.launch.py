from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('allow_teleop', default_value='true'),
        Node(
            package='yzz_waypoint_nav',
            executable='waypoint_nav_node',
            name='waypoint_nav_node',
            output='screen',
        ),
        Node(
            package='yzz_web_mapping',
            executable='web_mapping',
            name='web_mapping',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'allow_teleop': LaunchConfiguration('allow_teleop'),
            }],
        ),
    ])
