from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_share = os.path.join(os.getenv('COLCON_PREFIX_PATH', ''), 'share', 'rovio')
    params_file = os.path.join(os.path.dirname(__file__), '..', 'params', 'rovio_params.yaml')

    return LaunchDescription([
        Node(
            package='rovio',
            executable='rovio_node',
            name='rovio_node',
            output='screen',
            parameters=[params_file]
        )
    ])
