import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'so101_ros2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Register package with ament
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install calibration files
        (
            os.path.join('share', package_name, 'config', 'calibration'),
            glob('config/calibration/*.json'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nimrod',
    maintainer_email='nimicu21@gmail.com',
    description='ROS 2 bridge for SO101 robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follower_ros2_node = so101_ros2_bridge.follower_ros2_node:main',
            'leader_ros2_node = so101_ros2_bridge.leader_ros2_node:main',
        ],
    },
)
