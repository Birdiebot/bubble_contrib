'''
Author: HarryWen
Date: 2022-05-28 23:47:51
FilePath: /bubble/src/bubble_contrib/bubble_decision/setup.py
LastEditors: HarryWen
LastEditTime: 2022-08-10 23:53:31
License: GNU General Public License v3.0. See LICENSE file in root directory.
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved
'''
import os
from glob import glob
from setuptools import setup

package_name = 'bubble_decision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*_launch.py')),
        (os.path.join('share' , package_name, 'config') , glob('config/*.yaml')),

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
            'decision = bubble_decision.decision:main'
        ],
    },
)
