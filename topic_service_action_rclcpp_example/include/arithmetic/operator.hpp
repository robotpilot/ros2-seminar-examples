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


#ifndef TOPIC_SERVICE_ACTION_RCLCPP_EXAMPLES__ARITHMETIC_OPERATOR_HPP_
#define TOPIC_SERVICE_ACTION_RCLCPP_EXAMPLES__ARITHMETIC_OPERATOR_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "msg_srv_action_interface_example/srv/arithmetic_operator.hpp"


class Operator : public rclcpp::Node
{
 public:
  explicit Operator(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Operator();

 private:
  rclcpp::Client<msg_srv_action_interface_example::srv::ArithmeticOperator>::SharedPtr arithmetic_service_client_;
};
#endif  // TOPIC_SERVICE_ACTION_RCLCPP_EXAMPLES__ARITHMETIC_OPERATOR_HPP_
