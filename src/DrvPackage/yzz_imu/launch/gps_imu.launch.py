"""Start the WIT IMU with its externally connected NMEA GPS input enabled."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yzz_imu',
            executable='yzz_imu',
            name='gps_imu_driver',
            output='screen',
            parameters=[{
                'port': '/dev/imu_usb',
                'baud': 9600,
                'frame_id': 'imu_link',
                'gps_enabled': True,
                'gps_frame_id': 'gps_link',
                'gps_min_satellites': 4,
                'gps_fix_topic': '/gps/fix',
                'gps_velocity_topic': '/gps/vel',
                'gps_compatibility_topic': '/gps',
            }],
        ),
    ])
