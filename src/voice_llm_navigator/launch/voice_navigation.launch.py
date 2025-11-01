#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package='voice_llm_navigator',
        executable='voice_navigation',
        name='voice_navigation',
        output='screen',
        parameters=[
            # add params later
        ],
    )
    return LaunchDescription([node])
