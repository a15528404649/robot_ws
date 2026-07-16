from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    autostart = LaunchConfiguration('autostart')
    return LaunchDescription([
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate the Nav2 navigation lifecycle',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ),
            launch_arguments={
                'params_file': PathJoinSubstitution([FindPackageShare('yzz_navigation2'), 'config', 'nav2', 'nav2_params.yaml']),
                'use_sim_time': 'false',
                'autostart': autostart,
            }.items()
        )
    ])
