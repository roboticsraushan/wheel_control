#!/usr/bin/env python3
"""voice_web_bridge.launch.py

Launch the voice web UI bridge that connects Flask web interface to ROS2 navigation.

Usage:
    ros2 launch voice_llm_navigator voice_web_bridge.launch.py
    
    # Custom host/port:
    ros2 launch voice_llm_navigator voice_web_bridge.launch.py flask_host:=127.0.0.1 flask_port:=5003
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'flask_host',
            default_value='0.0.0.0',
            description='Flask server host address'
        ),
        DeclareLaunchArgument(
            'flask_port',
            default_value='5003',
            description='Flask server port'
        ),
        Node(
            package='voice_llm_navigator',
            executable='voice_web_bridge',
            name='voice_web_bridge',
            output='screen',
            parameters=[{
                'flask_host': LaunchConfiguration('flask_host'),
                'flask_port': LaunchConfiguration('flask_port'),
            }]
        ),
    ])
