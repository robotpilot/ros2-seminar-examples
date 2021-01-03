// Copyright 2020 ROBOTIS CO., LTD.
//
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

#include "arithmetic/operator.hpp"

using namespace std::chrono_literals;


Operator::Operator()
: Node("operator")
{
  arithmetic_service_client_ = this->create_client<msg_srv_action_interface_example::srv::ArithmeticOperator>("arithmetic_operator");
  while (!arithmetic_service_client_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }

  auto request = std::make_shared<msg_srv_action_interface_example::srv::ArithmeticOperator::Request>();
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> distribution(1, 4);
  request->arithmetic_operator = distribution(gen);

  using ServiceResponseFuture =
    rclcpp::Client<msg_srv_action_interface_example::srv::ArithmeticOperator>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Result %.2f", response->arithmetic_result);
      rclcpp::shutdown();
    };

  auto future_result = arithmetic_service_client_->async_send_request(request, response_received_callback);
}

Operator::~Operator()
{

}

void print_help()
{
  printf("For operator node:\n");
  printf("node_name [-h] [-c comment]\n");
  printf("Options:\n");
  printf("\t-h Help           : Print this help function.\n");
}

int main(int argc, char * argv[])
{
  if (rcutils_cli_option_exist(argv, argv + argc, "-h"))
  {
    print_help();
    return 0;
  }

  rclcpp::init(argc, argv);

  auto operators = std::make_shared<Operator>();

  rclcpp::spin(operators);

  rclcpp::shutdown();

  return 0;
}

