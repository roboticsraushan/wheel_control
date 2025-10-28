#!/usr/bin/env python3
"""
Launch file for RealSense D455 navigation with semantic segmentation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    segmentation_method_arg = DeclareLaunchArgument(
        'segmentation_method',
        default_value='color_based',
        description='Segmentation method: color_based, deeplab'
    )
    
    min_area_arg = DeclareLaunchArgument(
        'min_area',
        default_value='5000',
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
    
    # RealSense camera node
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'align_depth.enable': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
        }.items()
    )
    
    # Segmentation node
    segmentation_node = Node(
        package='realsense_nav',
        executable='segmentation_node',
        name='segmentation_node',
        output='screen',
        parameters=[{
            'segmentation_method': LaunchConfiguration('segmentation_method'),
            'min_area': LaunchConfiguration('min_area'),
            'linear_gain': LaunchConfiguration('linear_gain'),
            'angular_gain': LaunchConfiguration('angular_gain'),
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'max_angular_vel': LaunchConfiguration('max_angular_vel'),
        }]
    )
    
    return LaunchDescription([
        segmentation_method_arg,
        min_area_arg,
        linear_gain_arg,
        angular_gain_arg,
        max_linear_vel_arg,
        max_angular_vel_arg,
        realsense_launch,
        segmentation_node,
    ])
