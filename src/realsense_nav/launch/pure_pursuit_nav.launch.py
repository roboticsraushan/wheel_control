#!/usr/bin/env python3
"""
Launch file for complete navigation with goal seeking
Includes: RealSense camera, path segmentation, goal detection, pure pursuit controller
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    # Get package directories
    realsense_nav_dir = get_package_share_directory('realsense_nav')
    realsense2_camera_dir = get_package_share_directory('realsense2_camera')

    # Load hardware config for RealSense serial/defaults
    hw_cfg = {}
    try:
        cfg_path = os.path.join(realsense_nav_dir, 'config', 'hardware.yaml')
        with open(cfg_path, 'r') as f:
            hw_cfg = yaml.safe_load(f) or {}
    except Exception:
        hw_cfg = {}

    rs_cfg = hw_cfg.get('realsense', {})
    serial_no_default = str(rs_cfg.get('serial', ''))
    serial_no_arg = DeclareLaunchArgument('serial_no', default_value=serial_no_default,
                                         description='RealSense serial number')
    serial_no = LaunchConfiguration('serial_no')

    # RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense2_camera_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_color': 'true' if rs_cfg.get('enable_color', True) else 'false',
            'enable_depth': 'true' if rs_cfg.get('enable_depth', True) else 'false',
            'rgb_camera.profile': rs_cfg.get('rgb_profile', '1280x720x30'),
            'depth_module.profile': rs_cfg.get('depth_profile', '848x480x30'),
            'align_depth.enable': 'true' if rs_cfg.get('align_depth', True) else 'false',
            'serial_no': serial_no,
        }.items()
    )
    
    # Path segmentation node (keeps publishing centroid)
    segmentation_node = Node(
        package='realsense_nav',
        executable='segmentation_node',
        name='segmentation_node',
        parameters=[{
            'segmentation_method': 'color_based',
            'min_area': 5000,
        }],
        output='screen'
    )
    
    # Goal detection node (detects yellow cone)
    goal_detection_node = Node(
        package='realsense_nav',
        executable='goal_detection_node',
        name='goal_detection_node',
        parameters=[{
            'yellow_h_min': 20,
            'yellow_h_max': 35,
            'yellow_s_min': 100,
            'yellow_s_max': 255,
            'yellow_v_min': 100,
            'yellow_v_max': 255,
            'min_cone_area': 500,
            'max_cone_area': 50000,
        }],
        output='screen'
    )
    
    # Pure pursuit controller (combines path following and goal seeking)
    pure_pursuit_node = Node(
        package='realsense_nav',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        parameters=[{
            'lookahead_distance': 0.5,
            'max_linear_vel': 0.5,
            'max_angular_vel': 1.0,
            'goal_threshold': 0.8,
            'path_follow_weight': 0.3,
            'goal_seek_weight': 0.7,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        serial_no_arg,
        realsense_launch,
        segmentation_node,
        goal_detection_node,
        pure_pursuit_node,
    ])
