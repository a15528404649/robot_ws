from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('yzz_lidar')
    lidar_config = os.path.join(
        bringup_dir,
        'config',
        'ydlidar_4ros.yaml'
    )

    lidar_node = LifecycleNode(
        package='yzz_lidar',
        executable='yzz_lidar_node',
        name='yzz_lidar_node',
        namespace='/',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_config],
        remappings=[
            ('scan', 'scan_raw'),
        ],
    )

    scan_sanitizer_node = Node(
        package='yzz_lidar',
        executable='scan_sanitizer',
        name='scan_sanitizer',
        output='screen',
    )

    return LaunchDescription([
        lidar_node,
        scan_sanitizer_node,
    ])
