"""
Training Mode Launch File
#!/usr/bin/env python3

Launches nodes needed for training phase:
- RealSense camera
- Scene graph node (YOLO detection)
- Junction manager (records junctions)
- Voice command interface (optional)

Usage:
    ros2 launch realsense_nav training_mode.launch.py
    
Then:
    - Drive robot manually to junction
    - Say "record junction" or call service:
      ros2 service call /junction_manager/record_junction realsense_nav/srv/RecordJunction "{junction_name: 'kitchen'}"
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    # Load hardware config and allow specifying the RealSense serial number
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

    # Get package directories
    realsense_nav_dir = get_package_share_directory('realsense_nav')
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
            'rgb_camera.profile': rs_cfg.get('rgb_profile', '640x360x30'),
            'depth_module.profile': rs_cfg.get('depth_profile', '640x360x30'),
            # Enable common RealSense depth filters (can be overridden in hardware.yaml)
            # Typical filters: spatial, temporal, hole_filling, decimation, disparity
            # Provide a default set that improves single-pixel dropouts for object depth.
            'filters': rs_cfg.get('rs_filters', 'spatial,temporal,hole_filling'),
            # Infrared resolution for stereo VO (match run_stereo.py: 640x360x30)
            'infra_rgb': 'false',  # Keep infrared as grayscale (Y8 format)
            'align_depth.enable': 'true' if rs_cfg.get('align_depth', True) else 'false',
            # Disable IR emitter for better stereo tracking (infrared pattern interferes with VO).
            # Driver exposes emitter settings under `depth_module.*`; set them here so
            # training mode boots with the IR emitter off by default.
            'depth_module.emitter_enabled': '0',
            'depth_module.emitter_always_on': '0',
            # Some versions use an on/off toggle; include this for compatibility.
            'depth_module.emitter_on_off': '0',
            # Prefer device timestamps to keep frames synchronized (if driver supports it)
            'enable_device_time': 'true' if rs_cfg.get('enable_device_time', True) else 'false',
            # RealSense driver exposed params to force use of device timestamp
            # for each sensor submodule. Keep them on to improve stereo sync.
            'depth_module.global_time_enabled': 'true' if rs_cfg.get('enable_device_time', True) else 'false',
            'rgb_camera.global_time_enabled': 'true' if rs_cfg.get('enable_device_time', True) else 'false',
            'motion_module.global_time_enabled': 'true' if rs_cfg.get('enable_device_time', True) else 'false',
        }.items()
    )
    
    # Scene graph node (YOLO-based object detection)
    scene_graph_node = Node(
        package='realsense_nav',
        executable='scene_graph_node',
        name='scene_graph_node',
        parameters=[{
            # scene_graph_node will snapshot detections on trigger; continuous
            # detection is performed by a separate `yolo_detector` node.
            'use_yolo': False,
            'yolo_model': 'yolov8n.pt',
            'yolo_conf': 0.3,
        }],
        output='screen'
        )

    # Continuous YOLO detector (publishes /scene_graph/detections)
    yolo_detector_node = Node(
        package='realsense_nav',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[{
            'yolo_model': 'yolov8n.pt',
            'yolo_conf': 0.3,
            'min_interval_s': 0.15,
            'track_dist_px': 80.0,
            'max_lost_s': 1.0,
        }],
        # Switch to 'log' to avoid noisy stdout/stderr (model prints) flooding the terminal.
        output='log'
    )
    
    # Junction manager node
    junction_manager_node = Node(
        package='realsense_nav',
        executable='junction_manager_node',
        name='junction_manager',
        parameters=[{
            'data_dir': '/home/raushan/control_one/wheel_control/data/junctions',
            'image_format': 'jpg',
            'line_of_sight_distance': 5.0,
            'auto_save': True,
        }],
        output='screen'
    )
    
    # Enhanced scene graph recorder node (saves node images)
    scene_graph_recorder_node = Node(
        package='realsense_nav',
        executable='enhanced_scene_graph_recorder',
        name='scene_graph_recorder',
        parameters=[{
            'junction_dir': '/home/raushan/control_one/wheel_control/data/junctions',
            'node_dir': '/home/raushan/control_one/wheel_control/data/nodes',
        }],
        output='screen'
    )
    
    # Voice command node
    voice_command_node = Node(
        package='realsense_nav',
        executable='voice_command',
        name='voice_command',
        parameters=[{
            'language': 'en-US',
            'energy_threshold': 4000,
        }],
        # ALSA and audio library errors are written to stderr; keep them out of the console.
        output='log'
    )
    
    # Simple odometry (pose estimation)
    odometry_node = Node(
        package='realsense_nav',
        executable='simple_odometry',
        name='simple_odometry',
        output='screen'
    )
    
    # Visual odometry bridge (PyCuVSLAM ROS2 bridge)
    # Run the packaged Python script from workspace to provide visual odometry
    vo_bridge_script = os.path.join(os.getcwd(), 'src', 'PyCuVSLAM', 'bin', 'x86_64', 'ros2_run_stereo_bridge.py')
    vo_bridge_process = ExecuteProcess(
        cmd=['/usr/bin/env', 'python3', vo_bridge_script],
        output='screen'
    )
    
    # Segformer semantic segmentation node (using ExecuteProcess)
    # Note: allow_download=True to download model from Hugging Face if not present locally
    segformer_node = ExecuteProcess(
        cmd=['segformer_node_clean',
             '--ros-args',
             '--param', 'allow_download:=True',
             '--param', 'local_model_dir:=models/segformer-b1-finetuned-ade-512-512',
             '--param', 'prob_threshold:=0.5',
             '--param', 'min_area:=5000'],
        output='screen'
    )
    
    # Visualization node (world viewer)
    view_world_node = Node(
        package='realsense_nav',
        executable='view_world',
        name='view_world',
        parameters=[{
            'floorplan_path': '/home/raushan/control_one/wheel_control/data/maps/my_floorplan_11m.yaml'
        }],
        output='screen'
    )
    
    # Floorplan manager (publishes /memory/floorplan and metadata)
    # Use ExecuteProcess to run the installed python script directly so the
    # launch works even if the ament index isn't available in the current env.
    floorplan_script = os.path.join(
        os.path.abspath(os.path.join(realsense_nav_dir, '..', '..')), 'install', 'memory', 'lib', 'memory', 'floorplan_manager_node.py'
    )
    # Fallback path if running from workspace root
    if not os.path.exists(floorplan_script):
        floorplan_script = os.path.join(os.getcwd(), 'install', 'memory', 'lib', 'memory', 'floorplan_manager_node.py')

    floorplan_manager_process = ExecuteProcess(
        cmd=['/usr/bin/env', 'python3', floorplan_script,
             '--ros-args',
             '--param', "floorplan_path:='/home/raushan/control_one/wheel_control/data/maps/my_floorplan_11m_1.yaml'",
             '--param', "metadata_path:='/home/raushan/control_one/wheel_control/data/maps/my_floorplan_11m.yaml'",
             '--param', 'publish_rate:=1.0'
            ],
        output='screen'
    )
    
    # Map loader: publish OccupancyGrid built from YAML+PGM to /map
    map_loader_script = os.path.join(
        os.path.abspath(os.path.join(realsense_nav_dir, '..', '..')), 'install', 'memory', 'lib', 'memory', 'map_loader_node.py'
    )
    # Fallback path if running from workspace root
    if not os.path.exists(map_loader_script):
        map_loader_script = os.path.join(os.getcwd(), 'install', 'memory', 'lib', 'memory', 'map_loader_node.py')

    map_loader_process = ExecuteProcess(
        cmd=['/usr/bin/env', 'python3', map_loader_script,
             '--ros-args',
             '--param', "map_yaml:='/home/raushan/control_one/wheel_control/data/maps/my_floorplan_11m.yaml'",
             '--param', "map_topic:='/map'",
             '--param', 'publish_rate:=1.0'
            ],
        output='screen'
    )
    
    # Initial-pose handler: allow RViz 2D Pose Estimate to set map->odom
    initial_pose_script = os.path.join(
        os.path.abspath(os.path.join(realsense_nav_dir, '..', '..')), 'install', 'realsense_nav', 'lib', 'realsense_nav', 'initial_pose_to_map_odom.py'
    )
    if not os.path.exists(initial_pose_script):
        # fallback to workspace script
        initial_pose_script = os.path.join(os.getcwd(), 'src', 'realsense_nav', 'nodes', 'initial_pose_to_map_odom.py')

    initial_pose_process = ExecuteProcess(
        cmd=['/usr/bin/env', 'python3', initial_pose_script],
        output='screen'
    )
   
    # Semantic visualizer node
    semantic_visualizer_node = Node(
        package='realsense_nav',
        executable='semantic_visualizer_node',
        name='semantic_visualizer',
        parameters=[{
            'map_yaml_path': 'data/maps/my_floorplan_11m.yaml',
            'junction_db_path': 'data/junctions/junction_db.json',
            'topo_map_path': 'data/maps/topological_map.json',
            'web_port': 8080,
            'websocket_port': 8081,
        }],
        output='screen'
    )

    # Static transform: base_link -> camera_link
    # This defines where the camera is mounted on the robot.
    # Visual odometry now publishes odom->base_link, and this static transform
    # completes the chain: map -> odom -> base_link -> camera_link
    # 
    # Camera mounting position relative to base_link:
    #   Translation: (0.15, 0, 0.30) in base_link frame (camera is 15cm forward, 30cm up)
    #   Rotation: roll=+pi/2, pitch=0, yaw=+pi/2 (base_link -> camera optical frame)
    # 
    # This is the INVERSE of the camera_link->base_link transform we used before.
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_to_camera',
        # arguments: --x X --y Y --z Z --roll R --pitch P --yaw Y --frame-id FRAME --child-frame-id CHILD
        arguments=['--x', '0.15', '--y', '0.0', '--z', '0.30', '--roll', '1.57079632679', '--pitch', '0.0', '--yaw', '1.57079632679', '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
        output='screen'
    )

    # Scene graph mapper: convert /scene_graph detections into map-frame markers
    mapper_script = os.path.join(os.getcwd(), 'src', 'realsense_nav', 'realsense_nav', 'scene_graph_mapper.py')
    scene_graph_mapper_process = ExecuteProcess(
        cmd=['/usr/bin/env', 'python3', mapper_script,
             '--ros-args', '--param', "camera_info_topic:='/camera/camera/color/camera_info'"],
        output='screen'
    )

    return LaunchDescription([
        realsense_launch,
        yolo_detector_node,
        scene_graph_node,
        scene_graph_mapper_process,
        # junction_manager_node,
        # voice_command_node,
        # odometry_node,
        vo_bridge_process,
        segformer_node,
        # floorplan_manager_process,
        map_loader_process,
        initial_pose_process,
        view_world_node,
        # semantic_visualizer_node,
        static_tf_node,
        # Ensure the RealSense IR emitter is disabled after the camera node has started.
        # This matches the parameter we pass into the camera launch above but is a
        # second attempt run after everything else has launched (helps on slower
        # startup systems where the device node is not yet fully up when the
        # rs_launch receives arguments).
        TimerAction(
            period=6.0,
            actions=[ExecuteProcess(
                cmd=['/usr/bin/env', 'ros2', 'param', 'set', '/camera/camera', 'depth_module.emitter_enabled', '0'],
                output='screen'
            )]
        ),
    ])
