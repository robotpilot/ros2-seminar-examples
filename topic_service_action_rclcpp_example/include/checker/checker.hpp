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


#ifndef TOPIC_SERVICE_ACTION_RCLCPP_EXAMPLES__CHECKER_CHECKER_HPP_
#define TOPIC_SERVICE_ACTION_RCLCPP_EXAMPLES__CHECKER_CHECKER_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "msg_srv_action_interface_example/action/arithmetic_checker.hpp"

class Checker : public rclcpp::Node
{
 public:
  Checker(float goal_sum, const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Checker();

 private:
  void send_goal_total_sum(float goal_sum);

  void get_arithmetic_action_goal(
    std::shared_future<rclcpp_action::ClientGoalHandle<msg_srv_action_interface_example::action::ArithmeticChecker>::SharedPtr> future);

  void get_arithmetic_action_feedback(
    rclcpp_action::ClientGoalHandle<msg_srv_action_interface_example::action::ArithmeticChecker>::SharedPtr,
    const std::shared_ptr<const msg_srv_action_interface_example::action::ArithmeticChecker::Feedback> feedback);

  void get_arithmetic_action_result(
    const rclcpp_action::ClientGoalHandle<msg_srv_action_interface_example::action::ArithmeticChecker>::WrappedResult & result);

  rclcpp_action::Client<msg_srv_action_interface_example::action::ArithmeticChecker>::SharedPtr arithmetic_action_client_;
};
#endif  // TOPIC_SERVICE_ACTION_RCLCPP_EXAMPLES__CHECKER_CHECKER_HPP_

