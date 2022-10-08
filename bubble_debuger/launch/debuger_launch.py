'''
Author: Ligcox
Date: 2022-02-07 05:59:07
FilePath: /bubble/src/bubble_contrib/bubble_debuger/launch/debuger_launch.py
LastEditors: HarryWen
LastEditTime: 2022-07-11 00:57:28
License: GNU General Public License v3.0. See LICENSE file in root directory.
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved
'''
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # log_level = 'info'
    config = os.path.join(
        get_package_share_directory('bubble_debuger'),
        'config',
        'config.yaml'
    )

    if (os.getenv('ROS_DISTRO') == "dashing") or (os.getenv('ROS_DISTRO') == "eloquent"):
        return LaunchDescription([Node(
            package='bubble_debuger',
            node_name='debuger',
            node_executable='debuger',
            output='screen',

        )])
    else:
        return LaunchDescription([Node(
            package='bubble_debuger',
            name='debuger',
            executable='debuger',
            output='screen',

        )])

# generate_launch_description()
