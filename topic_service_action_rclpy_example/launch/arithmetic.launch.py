#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ros_namespace = LaunchConfiguration('ros_namespace')
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('topic_service_action_rclpy_example'),
            'param',
            'arithmetic_config.yaml'))

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'ros_namespace',
        #     default_value=os.environ['ROS_NAMESPACE'],
        #     description='Namespace for the robot'),

        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path of parameter file'),

        Node(
            # node_namespace=ros_namespace,
            package='topic_service_action_rclpy_example',
            node_executable='argument',
            node_name='argument',
            parameters=[param_dir],
            output='screen'),

        Node(
            # node_namespace=ros_namespace,
            package='topic_service_action_rclpy_example',
            node_executable='calculator',
            node_name='calculator',
            parameters=[param_dir],
            output='screen'),
    ])
