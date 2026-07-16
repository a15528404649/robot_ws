from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file = LaunchConfiguration("map")
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution([FindPackageShare('yzz_navigation2'), 'maps', 'bunker_map.yaml']),
        description="Path to the map yaml file"
    )

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

    # 2. 地图加载与 AMCL
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("yzz_navigation2"),
                "launch",
                "nav2_localization.launch.py"
            ])
        ),
        launch_arguments={"map": map_file}.items()
    )

    # 3. Nav2 planner/controller/BT navigator
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("yzz_navigation2"),
                "launch",
                "nav2_navigation.launch.py"
            ])
        ),
        launch_arguments={'autostart': 'false'}.items(),
    )

    navigation_gate = Node(
        package='yzz_robotlaunch',
        executable='navigation_gate',
        name='navigation_gate',
        output='screen',
    )

    return LaunchDescription([
        map_arg,
        robot_bringup,
        localization,
        navigation,
        navigation_gate,
    ])
