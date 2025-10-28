#!/usr/bin/env python3
"""
Complete Navigation Launch File
Brings up:
- RealSense D455 camera
- Path segmentation (floor detection)
- Goal detection (yellow cone)
- Pure pursuit controller
- Motor control (serial bridge)
- Gamepad control (Logitech F310)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    realsense_nav_dir = get_package_share_directory('realsense_nav')
    wheel_control_dir = get_package_share_directory('wheel_control')
    realsense2_camera_dir = get_package_share_directory('realsense2_camera')
    
    # RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense2_camera_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'rgb_camera.profile': '1280x720x30',
            'depth_module.profile': '848x480x30',
            'align_depth.enable': 'true',
        }.items()
    )
    
    # Path segmentation node (floor detection)
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
    
    # Goal detection node (yellow cone detection)
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
            'goal_threshold': 0.3,
            'path_follow_weight': 0.3,
            'goal_seek_weight': 0.7,
        }],
        output='screen'
    )
    
    # Serial motor bridge (converts /cmd_vel to Arduino commands)
    serial_bridge_node = Node(
        package='wheel_control',
        executable='serial_motor_bridge',
        name='serial_motor_bridge',
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud': 115200,
            'wheel_separation': 0.40,
            'max_speed_mps': 1.0,
        }],
        output='screen'
    )
    
    # Joy node (reads gamepad input)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )
    
    # Teleop twist joy (converts gamepad to /cmd_vel)
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[{
            'axis_linear.x': 1,
            'axis_angular.yaw': 0,
            'scale_linear.x': 0.5,
            'scale_angular.yaw': 1.0,
            'enable_button': 4,  # LB button
            'enable_turbo_button': 5,  # RB button
        }]
    )
    
    return LaunchDescription([
        realsense_launch,
        segmentation_node,
        goal_detection_node,
        pure_pursuit_node,
        serial_bridge_node,
        joy_node,
        teleop_node,
    ])
