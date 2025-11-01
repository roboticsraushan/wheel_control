#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    realsense_nav_dir = get_package_share_directory('realsense_nav')
    rviz_config = os.path.join(realsense_nav_dir, 'config', 'scene_graph.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_scene_graph',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        rviz_node
    ])
