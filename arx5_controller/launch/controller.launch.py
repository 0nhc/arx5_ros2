#!/usr/bin/env python3

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    arx5_display_node = Node(
        package="arx5_controller",
        executable="controller",
    )

    return LaunchDescription(
        [
            arx5_display_node,
        ]
    )
