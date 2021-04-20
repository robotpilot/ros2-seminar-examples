# Copyright 2021 OROCA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_rclpy_example.conversions import euler_to_quaternion
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class Base(Node):

    def __init__(self):
        super().__init__('base')

        self.get_logger().info('Make static tf (world->base)')
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.static_tf_broadcaster.sendTransform(self.broadcast())

    def broadcast(self):
        tf_stamped = TransformStamped()

        tf_stamped.header.stamp = self.get_clock().now().to_msg()
        tf_stamped.header.frame_id = 'world'
        tf_stamped.child_frame_id = 'base'
        tf_stamped.transform.translation.x = 0.0
        tf_stamped.transform.translation.y = 0.3
        tf_stamped.transform.translation.z = 0.3

        quaternion = euler_to_quaternion(0.0, 0.0, 1.57)

        tf_stamped.transform.rotation.x = quaternion[0]
        tf_stamped.transform.rotation.y = quaternion[1]
        tf_stamped.transform.rotation.z = quaternion[2]
        tf_stamped.transform.rotation.w = quaternion[3]

        return (tf_stamped)


def main():
    rclpy.init()
    try:
        base = Base()
        try:
            rclpy.spin(base)
        except KeyboardInterrupt:
            base.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            base.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
