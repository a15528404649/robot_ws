from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # 1. 底盘、IMU、雷达、EKF
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('yzz_robotlaunch'),
                'launch',
                'robot_bringup.launch.py'
            ])
        )
    )

    # 2. slam_toolbox 建图节点
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('yzz_navigation2'),
                'launch',
                'mapping.launch.py'
            ])
        )
    )

    # 等待 slam_toolbox 启动后进行配置
    configure_slam = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'lifecycle', 'set',
                    '/slam_toolbox', 'configure'
                ],
                output='screen'
            )
        ]
    )

    # 配置完成后激活 slam_toolbox
    activate_slam = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'lifecycle', 'set',
                    '/slam_toolbox', 'activate'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        robot_bringup,
        mapping,
        configure_slam,
        activate_slam,
    ])
