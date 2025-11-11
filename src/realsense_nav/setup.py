from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'realsense_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'visualization'), glob('realsense_nav/visualization/*.json')),
        (os.path.join('share', package_name, 'visualization', 'web'), glob('realsense_nav/visualization/web/*')),
    ],
    install_requires=['setuptools', 'py_trees'],
    zip_safe=True,
    maintainer='raushan',
    maintainer_email='roboticsraushan@gmail.com',
    description='RealSense D455 navigation with semantic segmentation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'segmentation_node = realsense_nav.segmentation_node:main',
            'color_tuner = realsense_nav.color_tuner:main',
            'rgb_color_tuner = realsense_nav.rgb_color_tuner:main',
            'goal_detection_node = realsense_nav.goal_detection_node:main',
            'pure_pursuit_controller = realsense_nav.pure_pursuit_controller:main',
            'nav_monitor = realsense_nav.nav_monitor:main',
            'behavior_tree_controller = realsense_nav.behavior_tree_controller:main',
            'behavior_tree_node = realsense_nav.behavior_tree_node:main',
            'scene_graph_node = realsense_nav.scene_graph_node:main',
            'yolo_detector = realsense_nav.yolo_detector_node:main',
            'view_world = realsense_nav.view_world:main',
            'llm_goal_detection = realsense_nav.llm_goal_detection:main',
            'vision_safety_node = realsense_nav.vision_safety_node:main',
            'nl_interpreter = realsense_nav.nl_interpreter:main',
            'topo_planner = realsense_nav.topo_planner:main',
            'scene_graph_viz = realsense_nav.scene_graph_viz:main',
            # Training / utility nodes
            'junction_manager_node = realsense_nav.training.junction_manager_node:main',
            'enhanced_scene_graph_recorder = realsense_nav.training.enhanced_scene_graph_recorder:main',
            'scene_graph_recorder = realsense_nav.training.scene_graph_recorder:main',
            'voice_command = realsense_nav.training.voice_command_node:main',
            # Navigation utilities
            'simple_odometry = realsense_nav.navigation.simple_odometry:main',
            # IMU Odometry
            'imu_odometry_node = realsense_nav.imu_odometry_node:main',
            'imu_odometry_madgwick = realsense_nav.imu_odometry_madgwick:main',
            # Path publisher from odometry
            'path_from_odom = realsense_nav.path_from_odom:main',
            # Visualization
            'semantic_visualizer_node = realsense_nav.visualization.semantic_visualizer_node:main',
        ],
    },
)
