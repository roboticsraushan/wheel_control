#!/usr/bin/env python3
"""
Full navigation with USB RGB camera + motor control
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino'
    )
    
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='0',
        description='Camera device index (0 = /dev/video0)'
    )
    
    processing_scale_arg = DeclareLaunchArgument(
        'processing_scale',
        default_value='0.5',
        description='Processing scale factor (0.5 = 50% for speed)'
    )
    
    linear_gain_arg = DeclareLaunchArgument(
        'linear_gain',
        default_value='0.5',
        description='Linear velocity gain for navigation'
    )
    
    angular_gain_arg = DeclareLaunchArgument(
        'angular_gain',
        default_value='1.0',
        description='Angular velocity gain for navigation'
    )
    
    # Serial bridge for motor control
    serial_bridge_node = Node(
        package='wheel_control',
        executable='serial_motor_bridge',
        name='serial_motor_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': 115200,
            'use_cmd_vel': True,
            'wheel_separation': 0.40,
            'max_speed_mps': 1.0,
        }]
    )
    
    # RGB segmentation navigation node
    rgb_segmentation_node = Node(
        package='realsense_nav',
        executable='rgb_segmentation_node',
        name='rgb_segmentation_node',
        output='screen',
        parameters=[{
            'camera_device': LaunchConfiguration('camera_device'),
            'camera_width': 1920,
            'camera_height': 1080,
            'processing_scale': LaunchConfiguration('processing_scale'),
            'segmentation_method': 'color_based',
            'min_area': 10000,
            'linear_gain': LaunchConfiguration('linear_gain'),
            'angular_gain': LaunchConfiguration('angular_gain'),
            'max_linear_vel': 0.5,
            'max_angular_vel': 1.0,
        }]
    )
    
    return LaunchDescription([
        port_arg,
        camera_device_arg,
        processing_scale_arg,
        linear_gain_arg,
        angular_gain_arg,
        serial_bridge_node,
        rgb_segmentation_node,
    ])
