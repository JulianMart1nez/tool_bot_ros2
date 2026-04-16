from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xarm5_basic_position_cmd'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required for ament resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Minimal xArm5 Cartesian position command node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'move_to_pose = xarm5_basic_position_cmd.move_to_pose:main',
            'add_table_collision = xarm5_basic_position_cmd.add_table_collision:main',
            'grasp_pose_generator = xarm5_basic_position_cmd.grasp_pose_generator:main',
            'auto_home = xarm5_basic_position_cmd.auto_home:main',
            'apriltag_perception = xarm5_basic_position_cmd.apriltag_perception:main',
            'gripper_depth_monitor = xarm5_basic_position_cmd.gripper_depth_monitor:main',
            'fine_localization = xarm5_basic_position_cmd.fine_localization:main',
            'test_descent = xarm5_basic_position_cmd.test_descent:main',
        ],
    },
)
