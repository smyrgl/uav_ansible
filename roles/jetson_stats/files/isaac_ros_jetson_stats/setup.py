# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT
#
# uav_ansible: upstream's isaac_ros_common version-info hook (an ament-index
# lookup of isaac_ros_common plus a custom build_py command that only embeds
# build metadata) is removed so the package builds without Isaac ROS installed.
# Everything else is upstream release-4.6.
from glob import glob
from os import path

from setuptools import setup

package_name = 'isaac_ros_jetson_stats'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Isaac ROS Maintainers',
    maintainer_email='isaac-ros-maintainers@nvidia.com',
    description='Isaac ROS Jetson. Set of packages to control your jetson from your board',
    license='MIT',
    extras_require={
        'test': [
            'pytest'
        ]
    },
    entry_points={
        'console_scripts': [
            'jtop = isaac_ros_jetson_stats.ros2_jtop_node:main',
        ],
    },
)
