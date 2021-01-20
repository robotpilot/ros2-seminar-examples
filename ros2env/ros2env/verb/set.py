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
from ros2env.api import set_ros_env
from ros2env.verb import VerbExtension


class SetVerb(VerbExtension):
    """Set ROS environment variables."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'env_name', help='Name of the environment variable')
        parser.add_argument(
            'value', help='Value of the environment variable')

    def main(self, *, args):
        if args.env_name or args.value:
            message = set_ros_env(args.env_name, args.value)
            print('[Changed ROS environment variable]:')
            print(message)
        message = get_all_env_list()
        print('\n[Current ROS environment variable]:')
        print(message)
