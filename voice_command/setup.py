from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'voice_command'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required by ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # Install config files (triggers.yaml lives here after build)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='ROS 2 voice command node with configurable trigger words',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run voice_command voice_command_node
            'voice_command_node = voice_command.voice_command_node:main',
        ],
    },
)
