from setuptools import setup
import os
from glob import glob

package_name = 'voice_llm_navigator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    license='Apache-2.0',
    author='You',
    author_email='you@example.com',
    description='Voice to LLM navigation bridge (MVP)',
    entry_points={
        'console_scripts': [
            'voice_navigation = voice_llm_navigator.voice_navigation_node:main',
        ],
    },
)
