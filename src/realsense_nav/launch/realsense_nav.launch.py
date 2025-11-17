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
from ament_index_python.packages import get_package_share_directory
import os
import yaml


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
    
    # Load hardware config and forward RealSense args
    hw_cfg = {}
    try:
        cfg_path = os.path.join(get_package_share_directory('realsense_nav'), 'config', 'hardware.yaml')
        with open(cfg_path, 'r') as f:
            hw_cfg = yaml.safe_load(f) or {}
    except Exception:
        hw_cfg = {}

    rs_cfg = hw_cfg.get('realsense', {})
    # RealSense expects serial_no as an integer (not a quoted string in launch args)
    default_serial = str(rs_cfg.get('serial', ''))

    # Get package directory for realsense2_camera
    realsense2_camera_dir = get_package_share_directory('realsense2_camera')
    # RealSense camera launch
    # Note: Passing serial_no as a LaunchConfiguration causes a type error in RealSense
    # (it expects integer type, but launch passes strings).
    # Solution: Let RealSense enumerate and pick the first available device.
    # If you have multiple cameras, use usb_port_id instead.
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense2_camera_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_color': 'true' if rs_cfg.get('enable_color', True) else 'false',
            'enable_depth': 'true' if rs_cfg.get('enable_depth', True) else 'false',
            # Enable infrared streams for stereo visual odometry
            'enable_infra1': 'true',
            'enable_infra2': 'true',
            # Forward IMU / inertial options from hardware config (default to true)
            'enable_imu': 'true' if rs_cfg.get('enable_imu', True) else 'false',
            'enable_gyro': 'true' if rs_cfg.get('enable_gyro', True) else 'false',
            'enable_accel': 'true' if rs_cfg.get('enable_accel', True) else 'false',
            'rgb_camera.profile': rs_cfg.get('rgb_profile', '1280x720x30'),
            'depth_module.profile': rs_cfg.get('depth_profile', '848x480x30'),
            # Infrared resolution for stereo VO (match run_stereo.py: 640x360x30)
            'infra_rgb': 'false',  # Keep infrared as grayscale (Y8 format)
            'align_depth.enable': 'true' if rs_cfg.get('align_depth', True) else 'false',
            # Disable IR emitter for better stereo tracking (infrared pattern interferes with VO)
            'enable_ir_emitter': 'false',
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
