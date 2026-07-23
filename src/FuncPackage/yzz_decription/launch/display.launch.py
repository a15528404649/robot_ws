from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_name = "yzz_decription"
    package_share = get_package_share_directory(package_name)

    xacro_file = os.path.join(
        package_share,
        "urdf",
        "bunker_mini.urdf.xacro",
    )

    rviz_config_file = os.path.join(
        package_share,
        "rviz",
        "bunker_mini.rviz",
    )

    robot_description = Command([
        "xacro ",
        xacro_file,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz on the local machine running this launch.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
            }],
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_rviz")),
            arguments=[
                "-d",
                rviz_config_file,
            ],
        ),
    ])