// Copyright 2021 OROCA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "calculator/calculator.hpp"


void print_help()
{
  printf("For ROS 2 topic subscriber, service server, action server rclcpp examples:\n");
  printf("calculator [-h]\n");
  printf("Options:\n");
  printf("\t-h Help           : Print this help function.\n");
}


int main(int argc, char * argv[])
{
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_help();
    return 0;
  }

  rclcpp::init(argc, argv);

  auto calculator = std::make_shared<Calculator>();

  rclcpp::spin(calculator);

  rclcpp::shutdown();

  return 0;
}
