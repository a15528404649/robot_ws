from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('yzz_ptz_camera'), 'config', 'params.yaml')
    device = LaunchConfiguration('device')
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        Node(
            package='yzz_ptz_camera',
            executable='yzz_ptz_camera',
            name='yzz_ptz_camera',
            output='screen',
            parameters=[params_file, {'device': device}],
        ),
    ])
