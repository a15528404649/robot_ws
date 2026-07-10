from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu_driver',
        output='screen',
        parameters=[
            {
                'port': '/dev/imu_usb',
                'baud': 9600,
                'frame_id': 'imu_link'
            }
        ]
    )

    return LaunchDescription([
        imu_node
    ])
