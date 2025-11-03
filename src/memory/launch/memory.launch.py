"""
Launch all memory system nodes.

This launch file starts:
- Floorplan Manager: loads and serves floorplan data
- Semantic Memory: manages semantic understanding of the environment
- Episodic Memory: records robot experiences and events
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    memory_dir = get_package_share_directory('memory')
    config_file = os.path.join(memory_dir, 'config', 'memory.yaml')

    # Declare arguments
    declare_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=config_file,
        description='Path to parameters file'
    )

    # Floorplan Manager Node
    floorplan_manager = Node(
        package='memory',
        executable='floorplan_manager',
        name='floorplan_manager',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True,
    )

    # Semantic Memory Node
    semantic_memory = Node(
        package='memory',
        executable='semantic_memory',
        name='semantic_memory',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True,
    )

    # Episodic Memory Node
    episodic_memory = Node(
        package='memory',
        executable='episodic_memory',
        name='episodic_memory',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True,
    )

    ld = LaunchDescription([
        declare_params_arg,
        floorplan_manager,
        semantic_memory,
        episodic_memory,
    ])

    return ld
