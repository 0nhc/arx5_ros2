#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    controller_launch_path = PathJoinSubstitution([FindPackageShare('arx5_controller'), 'launch', 'controller.launch.py'])
    display_launch_path = PathJoinSubstitution([FindPackageShare('arx5_description'), 'launch', 'display.launch.py'])

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_launch_path),
    )
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(display_launch_path),
    )

    return LaunchDescription([
        controller_launch,
        display_launch
    ])