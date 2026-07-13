from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup_dir = FindPackageShare('nav2_bringup')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav2_bringup_dir,
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'map': '/home/orin/robot_ws/maps/bunker_map.yaml',
            'params_file': '/home/orin/robot_ws/src/bunker_mini_bringup/config/nav2/nav2_params.yaml',
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])
