#!/usr/bin/env python3
"""Launch segformer_node_clean from the segmentation package."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    allow_download = LaunchConfiguration('allow_download', default='True')
    local_model_dir = LaunchConfiguration('local_model_dir', default='models/segformer-b1-finetuned-ade-512-512')

    # Run the installed segformer_node_clean executable (console_scripts entry point)
    proc = ExecuteProcess(
        cmd=[
            'segformer_node_clean',
            '--ros-args',
            '--param', 'allow_download:=False',
            '--param', 'local_model_dir:=models/segformer-b1-finetuned-ade-512-512'
        ],
        output='screen'
    )

    launch_args = [
        DeclareLaunchArgument('allow_download', default_value='True', description='Allow HF download at runtime'),
        DeclareLaunchArgument('local_model_dir', default_value='models/segformer-b1-finetuned-ade-512-512', description='Local model directory (relative to package)')
    ]

    return LaunchDescription(launch_args + [proc])
