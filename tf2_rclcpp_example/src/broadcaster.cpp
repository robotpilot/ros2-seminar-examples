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

#include <memory>

#include "broadcaster.hpp"

using namespace std::chrono_literals;

Arm::Arm()
: rclcpp::Node("arm"),
  move_(true)
{
  RCLCPP_INFO(this->get_logger(), "Move Arm!");
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  auto set_move =
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
    {
      move_ = request->data;

      if (request->data) {
        response->message = "Move Arm!";
      } else {
        response->message = "Stop Arm!";
      }
    };
  move_service_server_ = create_service<std_srvs::srv::SetBool>("move", set_move);

  auto broadcast =
    [this]() -> void
    {
      static double rad = 0.0;

      tf_stamped_list_.clear();

      geometry_msgs::msg::TransformStamped tf_stamped;

      tf_stamped.header.stamp = this->now();
      tf_stamped.header.frame_id = "world";
      tf_stamped.child_frame_id = "pan";
      tf_stamped.transform.translation.x = 0.0;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.0;

      tf2::Quaternion quaternion;
      quaternion.setRPY(0, 0, 2 * sin(rad));

      tf_stamped.transform.rotation.x = quaternion.x();
      tf_stamped.transform.rotation.y = quaternion.y();
      tf_stamped.transform.rotation.z = quaternion.z();
      tf_stamped.transform.rotation.w = quaternion.w();

      tf_stamped_list_.push_back(tf_stamped);

      tf_stamped.header.frame_id = "pan";
      tf_stamped.child_frame_id = "tilt";
      tf_stamped.transform.translation.x = 0.0;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.3;

      quaternion.setRPY(0, 2 * sin(rad), 0);

      tf_stamped.transform.rotation.x = quaternion.x();
      tf_stamped.transform.rotation.y = quaternion.y();
      tf_stamped.transform.rotation.z = quaternion.z();
      tf_stamped.transform.rotation.w = quaternion.w();

      tf_stamped_list_.push_back(tf_stamped);

      tf_stamped.header.frame_id = "tilt";
      tf_stamped.child_frame_id = "end-effector";
      tf_stamped.transform.translation.x = 0.0;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.1;

      quaternion.setRPY(0, 0, 0);

      tf_stamped.transform.rotation.x = quaternion.x();
      tf_stamped.transform.rotation.y = quaternion.y();
      tf_stamped.transform.rotation.z = quaternion.z();
      tf_stamped.transform.rotation.w = quaternion.w();

      tf_stamped_list_.push_back(tf_stamped);

      tf_broadcaster_->sendTransform(tf_stamped_list_);

      if (move_) {
        rad += 0.01;
      }
    };
  timer_ = this->create_wall_timer(10ms, broadcast);
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = std::make_shared<Arm>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
