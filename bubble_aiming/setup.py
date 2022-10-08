'''
Author: HarryWen
Date: 2022-05-28 02:52:46
FilePath: /bubble/src/bubble_contrib/bubble_aiming/setup.py
LastEditors: HarryWen
LastEditTime: 2022-06-04 16:42:06
License: GNU General Public License v3.0. See LICENSE file in root directory.
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved
'''
import os
from glob import glob
from setuptools import setup

package_name = 'bubble_aiming'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "utils"), glob('bubble_aiming/utils/*.py')),
        (os.path.join( 'share' , package_name, 'config') , glob('config/*.yaml')),
        (os.path.join('share', package_name,"launch"), glob('launch/*_launch.py')),

        (os.path.join('share', package_name, "predictor"), glob('predictor/*.py')),
        (os.path.join( 'share' , package_name, 'decision') , glob('decision/*.py')),
        (os.path.join('share', package_name,"compensator"), glob('compensator/*.py')),
        (os.path.join('lib','python3.8/site-packages', package_name, "utils"), glob('bubble_aiming/utils/*.py')),
        (os.path.join('lib','python3.8/site-packages', package_name, "predictor"), glob('bubble_aiming/predictor/*.py')),
        (os.path.join('lib','python3.8/site-packages', package_name, "decision"), glob('bubble_aiming/decision/*.py')),
        (os.path.join('lib','python3.8/site-packages', package_name, "compensator"), glob('bubble_aiming/compensator/*.py')),
        (os.path.join('lib','python3.6/site-packages', package_name, "utils"), glob('bubble_aiming/utils/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['aiming_node = bubble_aiming.aimingNode:main'],
    },
)