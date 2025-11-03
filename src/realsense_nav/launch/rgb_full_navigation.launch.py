#!/usr/bin/env python3
"""
Full navigation with segformer semantic segmentation + motor control
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    # Declare arguments
    # Load motor port default from hardware config
    hw_cfg = {}
    try:
        cfg_path = os.path.join(get_package_share_directory('realsense_nav'), 'config', 'hardware.yaml')
        with open(cfg_path, 'r') as f:
            hw_cfg = yaml.safe_load(f) or {}
    except Exception:
        hw_cfg = {}

    motors_cfg = hw_cfg.get('motors', {})
    rs_cfg = hw_cfg.get('realsense', {})

    port_arg = DeclareLaunchArgument(
        'port',
        default_value=str(motors_cfg.get('serial_port', '/dev/ttyACM0')),
        description='Serial port for Arduino'
    )
    
    # Get package directories
    realsense_nav_dir = get_package_share_directory('realsense_nav')
    realsense2_camera_dir = get_package_share_directory('realsense2_camera')
    segmentation_dir = get_package_share_directory('segmentation')
    
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
        }.items()
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
    
    # Segformer semantic segmentation node (using ExecuteProcess)
    segformer_node = ExecuteProcess(
        cmd=['segformer_node_clean',
             '--ros-args',
             '--param', 'allow_download:=False',
             '--param', 'local_model_dir:=models/segformer-b1-finetuned-ade-512-512',
             '--param', 'prob_threshold:=0.5',
             '--param', 'min_area:=5000'],
        output='screen'
    )
    
    return LaunchDescription([
        port_arg,
        realsense_launch,
        serial_bridge_node,
        segformer_node,
    ])
