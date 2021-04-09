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

#include "static_broadcaster.hpp"

using namespace std::chrono_literals;

Base::Base()
: rclcpp::Node("base")
{
  RCLCPP_INFO(this->get_logger(), "Make static tf (world->base)");
  node_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

  auto static_broadcast =
    [this]() -> void
    {
      geometry_msgs::msg::TransformStamped transform_stamped;

      transform_stamped.header.stamp = this->now();
      transform_stamped.header.frame_id = "world";
      transform_stamped.child_frame_id = "base";
      transform_stamped.transform.translation.x = 0.0;
      transform_stamped.transform.translation.y = 0.0;
      transform_stamped.transform.translation.z = 3.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, 1.57);

      transform_stamped.transform.rotation.x = q.x();
      transform_stamped.transform.rotation.y = q.y();
      transform_stamped.transform.rotation.z = q.z();
      transform_stamped.transform.rotation.w = q.w();

      static_tf_broadcaster_->sendTransform(transform_stamped);
    };

  timer_ = this->create_wall_timer(10ms, static_broadcast);
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = std::make_shared<Base>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
