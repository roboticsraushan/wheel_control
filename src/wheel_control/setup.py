from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'wheel_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Arduino Mega + Cytron motor control with ROS 2 Humble bridge and keyboard teleop.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_motor_bridge = wheel_control.serial_motor_bridge:main',
            'teleop_keyboard = wheel_control.teleop_keyboard:main',
        ],
    },
)
