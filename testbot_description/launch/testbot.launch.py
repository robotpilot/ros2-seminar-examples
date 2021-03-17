#!/usr/bin/env python3
# Copyright 2021 OROCA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_display_config_file = os.path.join(
        get_package_share_directory('testbot_description'),
        'rviz',
        'testbot.rviz')
    urdf_file = os.path.join(
        get_package_share_directory('testbot_description'),
        'urdf',
        'testbot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_file = infp.read()

    ld = LaunchDescription()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': False},
            {'robot_description': robot_description_file}
        ],
        output='screen')

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_display_config_file],
        output='screen')

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz2)

    return ld
