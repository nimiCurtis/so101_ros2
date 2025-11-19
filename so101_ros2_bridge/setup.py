from setuptools import setup
from glob import glob
import os

package_name = 'so101_ros2_bridge'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config/calibration'), glob('config/calibration/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anton',
    maintainer_email='anton@example.com',
    description='SO101 ROS2 Bridge',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # node scripts
            'follower_ros2_node = so101_ros2_bridge.follower_ros2_node:main',
            'leader_ros2_node = so101_ros2_bridge.leader_ros2_node:main',
            'smolvla_inference_node = so101_ros2_bridge.smolvla_inference_node:main',
            'smolvla_test_node = so101_ros2_bridge.smolvla_test_node:main',
            'action_chunk_executor_node = so101_ros2_bridge.action_chunk_executor_node:main',
        ],
    },
)