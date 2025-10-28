#!/usr/bin/env python3
"""
Launch file for USB RGB camera navigation with semantic segmentation
Works with any v4l2 compatible USB camera
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='0',
        description='Camera device index (0 = /dev/video0)'
    )
    
    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='1920',
        description='Camera resolution width'
    )
    
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='1080',
        description='Camera resolution height'
    )
    
    processing_scale_arg = DeclareLaunchArgument(
        'processing_scale',
        default_value='0.5',
        description='Scale factor for processing (0.5 = half resolution for speed)'
    )
    
    segmentation_method_arg = DeclareLaunchArgument(
        'segmentation_method',
        default_value='color_based',
        description='Segmentation method: color_based, edge_based, combined'
    )
    
    min_area_arg = DeclareLaunchArgument(
        'min_area',
        default_value='10000',
        description='Minimum navigable area in pixels'
    )
    
    linear_gain_arg = DeclareLaunchArgument(
        'linear_gain',
        default_value='0.5',
        description='Linear velocity gain'
    )
    
    angular_gain_arg = DeclareLaunchArgument(
        'angular_gain',
        default_value='1.0',
        description='Angular velocity gain'
    )
    
    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='0.5',
        description='Maximum linear velocity (m/s)'
    )
    
    max_angular_vel_arg = DeclareLaunchArgument(
        'max_angular_vel',
        default_value='1.0',
        description='Maximum angular velocity (rad/s)'
    )
    
    # RGB Segmentation node
    rgb_segmentation_node = Node(
        package='realsense_nav',
        executable='rgb_segmentation_node',
        name='rgb_segmentation_node',
        output='screen',
        parameters=[{
            'camera_device': LaunchConfiguration('camera_device'),
            'camera_width': LaunchConfiguration('camera_width'),
            'camera_height': LaunchConfiguration('camera_height'),
            'processing_scale': LaunchConfiguration('processing_scale'),
            'segmentation_method': LaunchConfiguration('segmentation_method'),
            'min_area': LaunchConfiguration('min_area'),
            'linear_gain': LaunchConfiguration('linear_gain'),
            'angular_gain': LaunchConfiguration('angular_gain'),
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'max_angular_vel': LaunchConfiguration('max_angular_vel'),
        }]
    )
    
    return LaunchDescription([
        camera_device_arg,
        camera_width_arg,
        camera_height_arg,
        processing_scale_arg,
        segmentation_method_arg,
        min_area_arg,
        linear_gain_arg,
        angular_gain_arg,
        max_linear_vel_arg,
        max_angular_vel_arg,
        rgb_segmentation_node,
    ])
