'''
Author: HarryWen
Date: 2022-05-28 02:52:35
FilePath: /bubble/src/bubble_contrib/bubble_debuger/setup.py
LastEditors: Ligcox
LastEditTime: 2022-07-04 15:36:13
License: GNU General Public License v3.0. See LICENSE file in root directory.
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved
'''
import os
from glob import glob
from setuptools import setup

package_name = 'bubble_debuger'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='zyhbum@foxmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'debuger = bubble_debuger.debuger:main',
            'point_debugger = bubble_debuger.point_debugger:main'
        ],
    },
)
