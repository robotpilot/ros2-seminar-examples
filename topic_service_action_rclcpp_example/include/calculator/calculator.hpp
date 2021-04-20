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

#ifndef CALCULATOR__CALCULATOR_HPP_
#define CALCULATOR__CALCULATOR_HPP_

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
  using ArithmeticArgument = msg_srv_action_interface_example::msg::ArithmeticArgument;
  using ArithmeticOperator = msg_srv_action_interface_example::srv::ArithmeticOperator;
  using ArithmeticChecker = msg_srv_action_interface_example::action::ArithmeticChecker;
  using GoalHandleArithmeticChecker = rclcpp_action::ServerGoalHandle<ArithmeticChecker>;

  explicit Calculator(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Calculator();

  float calculate_given_formula(const float & a, const float & b, const int8_t & operators);

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ArithmeticChecker::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleArithmeticChecker> goal_handle);
  void execute_checker(const std::shared_ptr<GoalHandleArithmeticChecker> goal_handle);

  rclcpp::Subscription<ArithmeticArgument>::SharedPtr
    arithmetic_argument_subscriber_;

  rclcpp::Service<ArithmeticOperator>::SharedPtr
    arithmetic_argument_server_;

  rclcpp_action::Server<ArithmeticChecker>::SharedPtr
    arithmetic_action_server_;

  float argument_a_;
  float argument_b_;

  int8_t argument_operator_;
  float argument_result_;

  std::string argument_formula_;
  std::vector<std::string> operator_;
};
#endif  // CALCULATOR__CALCULATOR_HPP_
