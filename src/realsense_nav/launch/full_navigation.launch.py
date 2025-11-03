#!/usr/bin/env python3
"""
Complete Navigation Launch File
Brings up:
- RealSense D455 camera
- Path segmentation (floor detection)
- Goal detection (yellow cone)
- Pure Pursuit controller (navigation)
- Behavior Tree manager (state management)
- Motor control (serial bridge)
- Gamepad control (Logitech F310)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    # Get package directories
    realsense_nav_dir = get_package_share_directory('realsense_nav')
    wheel_control_dir = get_package_share_directory('wheel_control')
    realsense2_camera_dir = get_package_share_directory('realsense2_camera')
    segmentation_dir = get_package_share_directory('segmentation')

    # Load hardware config
    hw_cfg = {}
    try:
        cfg_path = os.path.join(realsense_nav_dir, 'config', 'hardware.yaml')
        with open(cfg_path, 'r') as f:
            hw_cfg = yaml.safe_load(f) or {}
    except Exception:
        hw_cfg = {}

    rs_cfg = hw_cfg.get('realsense', {})
    motors_cfg = hw_cfg.get('motors', {})

    # Declare launch arguments with defaults from hardware config
    serial_no_arg = DeclareLaunchArgument('serial_no', default_value=str(rs_cfg.get('serial', '')),
                                         description='RealSense serial number')
    motor_port_arg = DeclareLaunchArgument('motor_port', default_value=str(motors_cfg.get('serial_port', '/dev/ttyACM0')),
                                          description='Serial port for motor Arduino')
    motor_baud_arg = DeclareLaunchArgument('motor_baud', default_value=str(motors_cfg.get('baud', 115200)),
                                          description='Baud rate for motor Arduino')

    serial_no = LaunchConfiguration('serial_no')
    motor_port = LaunchConfiguration('motor_port')
    motor_baud = LaunchConfiguration('motor_baud')
    
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
    
    # Path segmentation node (floor detection)
    # segmentation_node = Node(
    #     package='realsense_nav',
    #     executable='segmentation_node.py',
    #     name='segmentation_node',
    #     parameters=[{
    #         'segmentation_method': 'color_based',
    #         'min_area': 5000,
    #     }],
    #     output='log'  # Suppress output
    # )
    
    # Goal detection node (yellow cone detection)
    # goal_detection_node = Node(
    #     package='realsense_nav',
    #     executable='goal_detection_node.py',
    #     name='goal_detection_node',
    #     parameters=[{
    #         'yellow_h_min': 3,
    #         'yellow_h_max': 19,
    #         'yellow_s_min': 49,
    #         'yellow_s_max': 159,
    #         'yellow_v_min': 106,
    #         'yellow_v_max': 255,
    #         'min_cone_area': 1400,
    #         'max_cone_area': 20900,
    #     }],
    #     output='log'  # Suppress output
    # )
    
    # Pure pursuit controller (navigation - always runs)
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
        output='log'  # Suppress output
    )
    
    # Behavior Tree manager (state management - always runs)
    behavior_tree_node = Node(
        package='realsense_nav',
        executable='behavior_tree_node',
        name='behavior_tree_manager',
        output='screen'  # Show behavior tree status
    )
    
    # Serial motor bridge (converts /cmd_vel to Arduino commands)
    serial_bridge_node = Node(
        package='wheel_control',
        executable='serial_motor_bridge',
        name='serial_motor_bridge',
        parameters=[{
            'port': motor_port,
            'baud': LaunchConfiguration('motor_baud'),
            'wheel_separation': float(motors_cfg.get('wheel_separation', 0.40)),
            'max_speed_mps': float(motors_cfg.get('max_speed_mps', 1.0)),
        }],
        output='screen'  # Show Arduino commands
    )
    
    # Navigation monitor (prints distance and PWM values)
    # nav_monitor_node = Node(
    #     package='realsense_nav',
    #     executable='nav_monitor.py',
    #     name='nav_monitor',
    #     parameters=[{
    #         'wheel_separation': 0.40,
    #         'max_speed_mps': 1.0,
    #     }],
    #     output='screen'  # Only this node prints to screen
    # )
    
    # Joy node (reads gamepad input)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='log'  # Suppress output
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
        }],
        output='log'  # Suppress output
    )

    # Vision-only pipeline nodes (scene graph, safety, NL interpreter, topo planner)
    scene_graph_node = Node(
        package='realsense_nav',
        executable='scene_graph_node',
        name='scene_graph_node',
        parameters=[{
            'use_yolo': True,
            'yolo_model': 'yolov8n.pt',
            'yolo_conf': 0.25,
        }],
        output='log'
    )

    vision_safety_node = Node(
        package='realsense_nav',
        executable='vision_safety_node',
        name='vision_safety_node',
        output='log'
    )

    nl_interpreter_node = Node(
        package='realsense_nav',
        executable='nl_interpreter',
        name='nl_interpreter',
        output='log'
    )

    topo_planner_node = Node(
        package='realsense_nav',
        executable='topo_planner',
        name='topo_planner',
        output='log'
    )

    scene_graph_viz_node = Node(
        package='realsense_nav',
        executable='scene_graph_viz',
        name='scene_graph_viz',
        output='log'
    )

    llm_goal_detection_node = Node(
        package='realsense_nav',
        executable='llm_goal_detection',
        name='llm_goal_detection',
        output='log'
    )

    # Voice navigator (voice -> LLM -> /llm_goal)
    voice_navigation_node = Node(
        package='voice_llm_navigator',
        executable='voice_navigation',
        name='voice_navigation',
        output='screen'
    )
    # Segmentation (SegFormer) launch from segmentation package
    segformer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(segmentation_dir, 'launch', 'segformer.launch.py')
        )
    )
    
    # Start Foxglove bridge so web-based visualization (Foxglove Studio) can connect on port 8765
    foxglove_bridge_proc = ExecuteProcess(
        cmd=['ros2', 'launch', 'foxglove_bridge', 'foxglove_bridge_launch.xml', 'port:=8765'],
        output='screen'
    )

    return LaunchDescription([
        serial_no_arg,
        motor_port_arg,
        motor_baud_arg,
        realsense_launch,
        segformer_launch,
        # segmentation_node,
        # goal_detection_node,
        pure_pursuit_node,
        behavior_tree_node,
        serial_bridge_node,
        # nav_monitor_node,
        joy_node,
        teleop_node,
        # vision pipeline
        scene_graph_node,
        vision_safety_node,
        nl_interpreter_node,
        topo_planner_node,
        scene_graph_viz_node,
        llm_goal_detection_node,
        foxglove_bridge_proc,
    ])
