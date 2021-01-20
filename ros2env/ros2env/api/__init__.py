# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Canonical Ldt.
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


def get_ros_env_list():
    ros_version = os.getenv('ROS_VERSION', 'None')
    ros_distro = os.getenv('ROS_DISTRO', 'None')
    ros_python_version = os.getenv('ROS_PYTHON_VERSION', 'None')
    ros_env_list = 'ROS_VERSION        = {0}\n\
ROS_DISTRO         = {1}\n\
ROS_PYTHON_VERSION = {2}\n'.format(ros_version, ros_distro, ros_python_version)
    return ros_env_list


def get_dds_env_list():
    ros_domain_id = os.getenv('ROS_DOMAIN_ID', 'None')
    rmw_implementation = os.getenv('RMW_IMPLEMENTATION', 'None')
    dds_env_list = 'ROS_DOMAIN_ID      = {0}\n\
RMW_IMPLEMENTATION = {1}\n'.format(ros_domain_id, rmw_implementation)
    return dds_env_list


def get_all_env_list():
    ros_env_list = get_ros_env_list()
    dds_env_list = get_dds_env_list()
    all_env_list = ros_env_list + dds_env_list
    return all_env_list


def set_ros_env(env_name, env_value):
    os.environ[env_name] = env_value
    value = os.getenv(env_name, 'None')
    return '{0} = {1}'.format(env_name, value)
