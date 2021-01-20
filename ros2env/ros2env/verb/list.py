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

from ros2env.api import get_all_env_list
from ros2env.api import get_dds_env_list
from ros2env.api import get_ros_env_list
from ros2env.verb import VerbExtension


class ListVerb(VerbExtension):
    """Output a list of ROS environment variables."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '-a', '--all', action='store_true',
            help='Display all environment variables.')
        parser.add_argument(
            '-r', '--ros-env', action='store_true',
            help='Display the ROS environment variables.')
        parser.add_argument(
            '-d', '--dds-env', action='store_true',
            help='Display the DDS environment variables.')

    def main(self, *, args):
        if args.ros_env:
            message = get_ros_env_list()
        elif args.dds_env:
            message = get_dds_env_list()
        else:
            message = get_all_env_list()
        print(message)
