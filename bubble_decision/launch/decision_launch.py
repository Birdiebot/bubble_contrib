'''
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: Ligcox
Date: 2022-05-10 18:59:22
FilePath: /bubble/src/bubble_contrib/bubble_decision/launch/decision_launch.py
LastEditors: HarryWen
LastEditTime: 2022-08-11 01:15:00
E-mail: robomaster@birdiebot.top
'''

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node



aiming_param_dir = LaunchConfiguration(
    'params_file', default=os.path.join(
        get_package_share_directory('bubble_decision'), 'config', 'config.yaml')
)


arg_list = [
    DeclareLaunchArgument(
        'params_file',
        default_value=aiming_param_dir,
        description='Full path to aiming parameters file'
    ),

]

def generate_launch_description():
    if (os.getenv('ROS_DISTRO') == "dashing") or (os.getenv('ROS_DISTRO') == "eloquent"):
        return LaunchDescription(arg_list+[
            DeclareLaunchArgument(
                'robot_type',
                default_value='hero',
                description='Robot name, optional sentry_up| sentry_down| infantry| engineer| hero| air| radar| gather| standard.'
            ),

            Node(
                package='bubble_decision',
                node_name='bubble_decision',
                node_executable='decision',
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("params_file"),
                    {'robot_type': LaunchConfiguration('robot_type'),}]
            )
        ])
    else:
        return LaunchDescription(arg_list+[
            DeclareLaunchArgument(
                'robot_type',
                default_value='hero',
                description='Robot name, optional sentry_up| sentry_down| infantry| engineer| hero| air| radar| gather| standard.'
            ),

            Node(
                package='bubble_decision',
                name='bubble_decision',
                executable='decision',
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("params_file"),
                    {'robot_type': LaunchConfiguration('robot_type')}]
            )]
        )
