#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('wheel_control')

    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyACM0', description='Serial port for Arduino')
    baud_arg = DeclareLaunchArgument('baud', default_value='115200', description='Baud rate')
    use_cmd_vel_arg = DeclareLaunchArgument('use_cmd_vel', default_value='true', description='Bridge listens to /cmd_vel if true')
    wheel_sep_arg = DeclareLaunchArgument('wheel_separation', default_value='0.40', description='Wheel separation (m)')
    max_speed_arg = DeclareLaunchArgument('max_speed_mps', default_value='1.0', description='Max speed m/s corresponding to |PWM|=255')

    joy_dev_arg = DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0', description='Joystick device path')
    joy_deadzone_arg = DeclareLaunchArgument('joy_deadzone', default_value='0.05', description='Joystick deadzone')
    joy_autorepeat_arg = DeclareLaunchArgument('joy_autorepeat_rate', default_value='20.0', description='Joystick autorepeat rate (Hz)')

    teleop_cfg = os.path.join(pkg_share, 'config', 'teleop_f310.yaml')

    serial_bridge = Node(
        package='wheel_control',
        executable='serial_motor_bridge',
        name='serial_motor_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'use_cmd_vel': LaunchConfiguration('use_cmd_vel'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'max_speed_mps': LaunchConfiguration('max_speed_mps'),
        }]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('joy_dev'),
            'deadzone': LaunchConfiguration('joy_deadzone'),
            'autorepeat_rate': LaunchConfiguration('joy_autorepeat_rate'),
        }]
    )

    teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[teleop_cfg]
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        use_cmd_vel_arg,
        wheel_sep_arg,
        max_speed_arg,
        joy_dev_arg,
        joy_deadzone_arg,
        joy_autorepeat_arg,
        serial_bridge,
        joy_node,
        teleop,
    ])
