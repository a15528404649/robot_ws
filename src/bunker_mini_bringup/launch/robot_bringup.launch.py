from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('bunker_mini_description'),
            'urdf',
            'bunker_mini.urdf.xacro'
        ])
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
        }],
    )

    bunker_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bunker_base'),
                'launch',
                'bunker_base.launch.py'
            ])
        ),
        launch_arguments={
            'port_name': 'can_usb',
            'is_bunker_mini': 'true',
            'publish_tf': 'false',
        }.items()
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wit_ros2_imu'),
                'launch',
                'rviz_and_imu.launch.py'
            ])
        )
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bunker_mini_bringup'),
                'launch',
                'localization.launch.py'
            ])
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bunker_mini_bringup'),
                'launch',
                'lidar.launch.py'
            ])
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        bunker_base_launch,
        imu_launch,
        ekf_launch,
        lidar_launch,
    ])
