#!/usr/bin/env python3
"""
Complete Junction-Based Navigation Launch File

Launches all components for autonomous navigation:
- RealSense camera
- Scene graph (YOLO)
- Voice command parser
- Path planning (graph-based)
- Junction recognition
- Junction navigation
- Localization
- Destination verification
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
    realsense_nav_dir = get_package_share_directory('realsense_nav')
    realsense2_camera_dir = get_package_share_directory('realsense2_camera')

    # Load hardware config and set defaults
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

    # RealSense camera
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
    
    # Scene graph node
    scene_graph_node = Node(
        package='realsense_nav',
        executable='scene_graph_node',
        name='scene_graph_node',
        parameters=[{
            'use_yolo': True,
            'yolo_model': 'yolov8n.pt',
            'yolo_conf': 0.3,
        }],
        output='screen'
    )
    
    # Topological map builder
    topo_map_builder = Node(
        package='realsense_nav',
        executable='topo_map_builder.py',
        name='topo_map_builder',
        output='screen'
    )
    
    # Voice navigation parser
    voice_nav_parser = Node(
        package='realsense_nav',
        executable='voice_nav_parser.py',
        name='voice_nav_parser',
        output='screen'
    )
    
    # Graph path planner
    graph_path_planner = Node(
        package='realsense_nav',
        executable='graph_path_planner.py',
        name='graph_path_planner',
        output='screen'
    )
    
    # Junction recognizer
    junction_recognizer = Node(
        package='realsense_nav',
        executable='junction_recognizer.py',
        name='junction_recognizer',
        parameters=[{
            'feature_type': 'ORB',
            'match_threshold': 0.7,
            'min_matches': 15,
        }],
        output='screen'
    )
    
    # Obstacle-aware junction navigator (replaces basic navigator)
    junction_navigator = Node(
        package='realsense_nav',
        executable='obstacle_aware_navigator.py',
        name='junction_navigator',
        parameters=[{
            'arrival_threshold': 0.8,
            'linear_vel': 0.3,
            'angular_vel': 0.5,
            'obstacle_threshold': 0.3,
            'min_clear_width': 0.4,
        }],
        output='screen'
    )
    
    # Junction visibility monitor
    visibility_monitor = Node(
        package='realsense_nav',
        executable='junction_visibility_monitor.py',
        name='visibility_monitor',
        parameters=[{
            'visibility_threshold': 0.6,
            'lost_timeout': 3.0,
        }],
        output='screen'
    )
    
    # Simple odometry
    odometry_node = Node(
        package='realsense_nav',
        executable='simple_odometry',
        name='simple_odometry',
        output='screen'
    )
    
    # Localization manager
    localization_manager = Node(
        package='realsense_nav',
        executable='localization_manager.py',
        name='localization_manager',
        output='screen'
    )
    
    # Destination verifier
    destination_verifier = Node(
        package='realsense_nav',
        executable='destination_verifier.py',
        name='destination_verifier',
        output='screen'
    )
    
    return LaunchDescription([
        serial_no_arg,
        realsense_launch,
        scene_graph_node,
        topo_map_builder,
        voice_nav_parser,
        graph_path_planner,
        junction_recognizer,
        junction_navigator,
        visibility_monitor,
        odometry_node,
        localization_manager,
        destination_verifier,
    ])
