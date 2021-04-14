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

#include "listener.hpp"

using namespace std::chrono_literals;

Listener::Listener()
: rclcpp::Node("tf2_listener"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  auto listen =
    [this]() -> void
    {
      geometry_msgs::msg::TransformStamped tf_world_pan;
      geometry_msgs::msg::TransformStamped tf_pan_world;

      try {
        tf_world_pan = tf_buffer_.lookupTransform("pan", "world", tf2::timeFromSec(0));

        tf2::Quaternion quaternion(
          tf_world_pan.transform.rotation.x,
          tf_world_pan.transform.rotation.y,
          tf_world_pan.transform.rotation.z,
          tf_world_pan.transform.rotation.w);

        tf2::Matrix3x3 world_pan_mat(quaternion);
        double roll, pitch, yaw;
        world_pan_mat.getRPY(roll, pitch, yaw);

        RCLCPP_INFO(
          this->get_logger(),
          "tf_world_pan :\n\tOrigin %.3f %.3f %.3f\n\tRotation(deg) %.3f %.3f %.3f",
          tf_world_pan.transform.translation.x,
          tf_world_pan.transform.translation.y,
          tf_world_pan.transform.translation.z,
          roll * RAD_TO_DEG,
          pitch * RAD_TO_DEG,
          yaw * RAD_TO_DEG);

        tf_pan_world = tf_buffer_.lookupTransform("world", "pan", tf2::timeFromSec(0));

        tf2::Transform tf_inverse;
        tf2::fromMsg(tf_pan_world.transform, tf_inverse);

        tf_inverse = tf_inverse.inverse();

        tf2::Matrix3x3 pan_world_mat(tf_inverse.getRotation());
        pan_world_mat.getRPY(roll, pitch, yaw);

        RCLCPP_INFO(
          this->get_logger(),
          "tf_pan_world(inverse) :\n\tOrigin %.3f %.3f %.3f\n\tRotation(deg) %.3f %.3f %.3f",
          tf_pan_world.transform.translation.x,
          tf_pan_world.transform.translation.y,
          tf_pan_world.transform.translation.z,
          roll * RAD_TO_DEG,
          pitch * RAD_TO_DEG,
          yaw * RAD_TO_DEG);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        rclcpp::sleep_for(1s);
      }
    };

  timer_ = this->create_wall_timer(500ms, listen);
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = std::make_shared<Listener>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
