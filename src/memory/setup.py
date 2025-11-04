from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'memory'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raushan',
    maintainer_email='roboticsraushan@gmail.com',
    description='Semantic and episodic memory system for robot navigation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'floorplan_manager = memory.floorplan_manager_node:main',
            'floorplan_converter = memory.floorplan_converter_node:main',
            'semantic_memory = memory.semantic_memory_node:main',
            'episodic_memory = memory.episodic_memory_node:main',
            'map_loader_node = memory.map_loader_node:main',
        ],
    },
)
