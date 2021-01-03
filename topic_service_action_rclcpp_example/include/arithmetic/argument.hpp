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


#ifndef TOPIC_SERVICE_ACTION_RCLCPP_EXAMPLES__ARITHMETIC_ARGUMENT_HPP_
#define TOPIC_SERVICE_ACTION_RCLCPP_EXAMPLES__ARITHMETIC_ARGUMENT_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "msg_srv_action_interface_example/msg/arithmetic_argument.hpp"

using namespace std::chrono_literals;


class Argument : public rclcpp::Node
{
 public:
  Argument()
  : Node("argument"),
    min_random_num_(0.0),
    max_random_num_(0.0)
  {
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = this->get_parameter("qos_depth").get_value<int8_t>();
    this->declare_parameter("min_random_num", 0.0);
    min_random_num_ = this->get_parameter("min_random_num").get_value<float>();
    this->declare_parameter("max_random_num", 9.0);
    max_random_num_ = this->get_parameter("max_random_num").get_value<float>();
    this->update_parameter();

    const auto QOS_RKL10V
      = rclcpp::QoS(rclcpp::KeepLast(qos_depth))
        .reliable()
        .durability_volatile();

    arithmetic_argument_publisher_ =
      this->create_publisher<msg_srv_action_interface_example::msg::ArithmeticArgument>("arithmetic_argument", QOS_RKL10V);

    timer_ = this->create_wall_timer(1s, std::bind(&Argument::publish_random_arighmetic_arguments, this));
  }

  virtual ~Argument()
  {

  }

 private:
  void publish_random_arighmetic_arguments()
  {
    msg_srv_action_interface_example::msg::ArithmeticArgument msg;
    msg.stamp = this->now();
    msg.argument_a = min_random_num_ + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_random_num_-min_random_num_)));
    msg.argument_b = min_random_num_ + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_random_num_-min_random_num_)));
    arithmetic_argument_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published argument_a %.2f", msg.argument_a);
    RCLCPP_INFO(this->get_logger(), "Published argument_b %.2f", msg.argument_b);
  }

  void update_parameter()
  {
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
    while (!parameters_client_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    auto param_event_callback =
      [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
        {
          for (auto & changed_parameter : event->changed_parameters)
          {
            if (changed_parameter.name == "min_random_num")
            {
              auto value = rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double();
              min_random_num_ = value;
            }
            else if (changed_parameter.name == "max_random_num")
            {
              auto value = rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double();
              max_random_num_ = value;
            }
          }
        };

    parameter_event_sub_ = parameters_client_->on_parameter_event(param_event_callback);
  }

  float min_random_num_;
  float max_random_num_;

  rclcpp::Publisher<msg_srv_action_interface_example::msg::ArithmeticArgument>::SharedPtr arithmetic_argument_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
};
#endif  // TOPIC_SERVICE_ACTION_RCLCPP_EXAMPLES__ARITHMETIC_ARGUMENT_HPP_
