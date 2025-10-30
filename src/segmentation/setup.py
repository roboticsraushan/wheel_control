from setuptools import find_packages, setup

package_name = 'segmentation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/segformer.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raushan',
    maintainer_email='roboticsraushan@gmail.com',
    description='Segmentation nodes for semantic segmentation (segformer).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'segformer_node_clean = segmentation.segformer_node_clean:main',
        ],
    },
)
