"""
Launch floorplan manager together with the world viewer.

This launch starts:
 - memory:floorplan_manager (console script installed by the memory package)
 - realsense_nav:view_world (viewer showing camera streams + floorplan)

Usage:
  ros2 launch memory floorplan_with_view.launch.py params_file:=<path-to-params> \
    floorplan:=/absolute/path/to/data/maps/my_floorplan.pgm \
    metadata:=/absolute/path/to/data/maps/my_floorplan.yaml

This launch exposes two args:
 - params_file: memory params YAML (default: config/memory.yaml in memory package)
 - floorplan: absolute path to the floorplan image (PGM/PNG) to load
 - metadata: absolute path to YAML metadata (optional)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    memory_dir = get_package_share_directory('memory')
    default_params = os.path.join(memory_dir, 'config', 'memory.yaml')

    params_arg = DeclareLaunchArgument('params_file', default_value=default_params,
                                       description='Memory params file')

    floorplan_arg = DeclareLaunchArgument('floorplan', default_value='',
                                          description='Absolute path to floorplan image (PGM/PNG)')

    metadata_arg = DeclareLaunchArgument('metadata', default_value='',
                                         description='Absolute path to floorplan metadata YAML')

    params_file = LaunchConfiguration('params_file')
    floorplan = LaunchConfiguration('floorplan')
    metadata = LaunchConfiguration('metadata')

    # Floorplan Manager node (memory package console script / entry_point)
    floorplan_manager = Node(
        package='memory',
        executable='floorplan_manager',
        name='floorplan_manager',
        output='screen',
        parameters=[params_file],
        emulate_tty=True,
        # Pass floorplan parameters via node parameters if provided
        remappings=[],
    )

    # Viewer (console script installed in realsense_nav)
    viewer = Node(
        package='realsense_nav',
        executable='view_world',
        name='world_viewer',
        output='screen',
        emulate_tty=True,
    )

    # Informational log
    info = LogInfo(msg=['Launching floorplan manager + world viewer...'])

    ld = LaunchDescription([
        params_arg,
        floorplan_arg,
        metadata_arg,
        info,
        floorplan_manager,
        viewer,
    ])

    return ld
