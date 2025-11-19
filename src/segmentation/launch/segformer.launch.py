#!/usr/bin/env python3
"""Launch segformer_node_clean from the segmentation package."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution, JoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # By default prefer to use the packaged model (don't download at runtime)
    # Prefer workspace source model path if present (helps dev workflows)
    workspace_src_model = os.path.join(os.getcwd(), 'src', 'segmentation', 'models', 'segformer-b1-finetuned-ade-512-512')
    if os.path.isdir(workspace_src_model):
        default_model_dir = workspace_src_model
    else:
        segmentation_dir = get_package_share_directory('segmentation')
        default_model_dir = os.path.join(segmentation_dir, 'models', 'segformer-b1-finetuned-ade-512-512')

    allow_download = LaunchConfiguration('allow_download', default='False')
    local_model_dir = LaunchConfiguration('local_model_dir', default=default_model_dir)

    # Run the installed segformer_node_clean executable (console_scripts entry point)
    # Pass the launch configurations through to the node so the same
    # launch file can be used for development (allow_download=True) or
    # production (allow_download=False, use packaged model)
    proc = ExecuteProcess(
        cmd=[
            'segformer_node_clean',
            '--ros-args',
            '--param', JoinSubstitution([TextSubstitution(text='allow_download:='), allow_download]),
            '--param', JoinSubstitution([TextSubstitution(text='local_model_dir:='), local_model_dir])
        ],
        output='screen'
    )

    launch_args = [
        DeclareLaunchArgument('allow_download', default_value='False', description='Allow HF download at runtime'),
        DeclareLaunchArgument('local_model_dir', default_value=default_model_dir, description='Local model directory (absolute path computed from package)')
    ]

    return LaunchDescription(launch_args + [proc])
