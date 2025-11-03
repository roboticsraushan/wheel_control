#!/usr/bin/env python3
"""
Launch file for segformer semantic segmentation with RealSense D455
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    # Load hardware config
    hw_cfg = {}
    try:
        cfg_path = os.path.join(get_package_share_directory('realsense_nav'), 'config', 'hardware.yaml')
        with open(cfg_path, 'r') as f:
            hw_cfg = yaml.safe_load(f) or {}
    except Exception:
        hw_cfg = {}

    rs_cfg = hw_cfg.get('realsense', {})
    
    # Get package directories
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
        realsense_launch,
        segformer_node,
    ])
