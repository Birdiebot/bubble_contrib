'''
Author: HarryWen
Date: 2022-05-28 19:25:42
FilePath: /bubble/src/bubble_contrib/bubble_aiming/launch/aiming_launch.py
LastEditors: HarryWen
LastEditTime: 2022-07-13 22:40:08
License: GNU General Public License v3.0. See LICENSE file in root directory.
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved
'''

import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

aiming_param_dir = LaunchConfiguration(
    'params_file', default=os.path.join(
        get_package_share_directory('bubble_aiming'), 'config', 'config.yaml')
)

arg_list = [
    DeclareLaunchArgument(
        'params_file',
        default_value=aiming_param_dir,
        description='Full path to aiming parameters file'
    ),

    DeclareLaunchArgument(
        "use_regular_send",
        default_value="False",
        description="Whether the node will send masseges to topic use fixed frequency",
        choices=['True', 'False']
    ),

    DeclareLaunchArgument(
        "use_synchronize",
        default_value="False",
        description="Whether sync massege in gimbal massege and targe massege",
        choices=['True', 'False']
    ),

    DeclareLaunchArgument(
        "debug_mode",
        default_value="True",
        description="Enable debug mode",
        choices=['True', 'False']
    ),
]


def generate_launch_description():
    if (os.getenv('ROS_DISTRO') == "dashing") or (os.getenv('ROS_DISTRO') == "eloquent"):
        return LaunchDescription(arg_list + [
            Node(
                package='bubble_aiming',
                node_name="aiming_node",
                node_executable='aiming_node',
                parameters=[
                    LaunchConfiguration("params_file"),

                ],
                output='screen',
            )
        ])
    else:
        return LaunchDescription(arg_list + [
            Node(
                package='bubble_aiming',
                name="aiming_node",
                executable='aiming_node',
                parameters=[LaunchConfiguration("params_file"), {
                        "use_regular_send": LaunchConfiguration("use_regular_send"),
                        "use_synchronize": LaunchConfiguration("use_synchronize")
                }],
                emulate_tty=True,

                output='screen',
            ),
        ])
