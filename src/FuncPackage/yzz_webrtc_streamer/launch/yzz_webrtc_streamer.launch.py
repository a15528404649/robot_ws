from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('yzz_webrtc_streamer'),
        'config',
        'webrtc_streamer.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/usb_cam/image_raw'),
        Node(
            package='yzz_webrtc_streamer',
            executable='webrtc_streamer',
            name='webrtc_streamer',
            output='screen',
            parameters=[params_file, {'image_topic': LaunchConfiguration('image_topic')}],
        ),
    ])
