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


#ifndef TOPIC_SERVICE_ACTION_RCLCPP_EXAMPLES__CALCULATOR_CALCULATOR_HPP_
#define TOPIC_SERVICE_ACTION_RCLCPP_EXAMPLES__CALCULATOR_CALCULATOR_HPP_

#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "msg_srv_action_interface_example/msg/arithmetic_argument.hpp"
#include "msg_srv_action_interface_example/srv/arithmetic_operator.hpp"
#include "msg_srv_action_interface_example/action/arithmetic_checker.hpp"


class Calculator : public rclcpp::Node
{
 public:
  Calculator();
  virtual ~Calculator();

 private:
  float calculate_given_formula(const float & a, const float & b, const int8_t & operators);

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const msg_srv_action_interface_example::action::ArithmeticChecker::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<msg_srv_action_interface_example::action::ArithmeticChecker>> goal_handle);
  void execute_checker(const std::shared_ptr<rclcpp_action::ServerGoalHandle<msg_srv_action_interface_example::action::ArithmeticChecker>> goal_handle);
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<msg_srv_action_interface_example::action::ArithmeticChecker>> goal_handle);

  rclcpp::Subscription<msg_srv_action_interface_example::msg::ArithmeticArgument>::SharedPtr
    arithmetic_argument_subscriber_;

  rclcpp::Service<msg_srv_action_interface_example::srv::ArithmeticOperator>::SharedPtr
    arithmetic_argument_server_;

  rclcpp_action::Server<msg_srv_action_interface_example::action::ArithmeticChecker>::SharedPtr
    arithmetic_action_server_;

  float argument_a_;
  float argument_b_;

  int8_t argument_operator_;
  float argument_result_;

  std::string argument_formula_;
  std::vector<std::string> operator_;
};
#endif  // TOPIC_SERVICE_ACTION_RCLCPP_EXAMPLES__CALCULATOR_CALCULATOR_HPP_
