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

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from tf2_rclpy_example.conversions import euler_to_quaternion
from tf2_ros.transform_broadcaster import TransformBroadcaster


class Arm(Node):

    def __init__(self):
        super().__init__('arm')

        self.get_logger().info('Move Arm!')
        self.move = True
        self.rad = 0.0
        self.tf_broadcaster = TransformBroadcaster(self)

        self.move_service_server = self.create_service(
            SetBool,
            'move',
            self.set_move)

        self.timer = self.create_timer(0.010, self.broadcast)

    def set_move(self, request, response):
        self.move = request.data

        if request.data is True:
            response.message = 'Move Arm!'
        else:
            response.message = 'Stop Arm!'

    def broadcast(self):
        tf_stamped_list = []
        tf_stamped = TransformStamped()

        now = self.get_clock().now()
        tf_stamped.header.stamp = now.to_msg()

        tf_stamped.header.frame_id = 'world'
        tf_stamped.child_frame_id = 'pan'
        tf_stamped.transform.translation.x = 0.0
        tf_stamped.transform.translation.y = 0.0
        tf_stamped.transform.translation.z = 0.0

        quaternion = euler_to_quaternion(0.0, 0.0, 2.0 * math.sin(self.rad))

        tf_stamped.transform.rotation.x = quaternion[0]
        tf_stamped.transform.rotation.y = quaternion[1]
        tf_stamped.transform.rotation.z = quaternion[2]
        tf_stamped.transform.rotation.w = quaternion[3]

        tf_stamped_list.append(tf_stamped)
        tf_stamped = TransformStamped()

        tf_stamped.header.stamp = now.to_msg()

        tf_stamped.header.frame_id = 'pan'
        tf_stamped.child_frame_id = 'tilt'
        tf_stamped.transform.translation.x = 0.0
        tf_stamped.transform.translation.y = 0.0
        tf_stamped.transform.translation.z = 0.3

        quaternion = euler_to_quaternion(0.0, 2.0 * math.sin(self.rad), 0.0)

        tf_stamped.transform.rotation.x = quaternion[0]
        tf_stamped.transform.rotation.y = quaternion[1]
        tf_stamped.transform.rotation.z = quaternion[2]
        tf_stamped.transform.rotation.w = quaternion[3]

        tf_stamped_list.append(tf_stamped)
        tf_stamped = TransformStamped()

        tf_stamped.header.stamp = now.to_msg()

        tf_stamped.header.frame_id = 'tilt'
        tf_stamped.child_frame_id = 'end-effector'
        tf_stamped.transform.translation.x = 0.0
        tf_stamped.transform.translation.y = 0.0
        tf_stamped.transform.translation.z = 0.0

        quaternion = euler_to_quaternion(0.0, 0.0, 0.0)

        tf_stamped.transform.rotation.x = quaternion[0]
        tf_stamped.transform.rotation.y = quaternion[1]
        tf_stamped.transform.rotation.z = quaternion[2]
        tf_stamped.transform.rotation.w = quaternion[3]

        tf_stamped_list.append(tf_stamped)

        self.tf_broadcaster.sendTransform(tf_stamped_list)

        if self.move is True:
            self.rad += 0.01


def main():
    rclpy.init()
    try:
        arm = Arm()
        try:
            rclpy.spin(arm)
        except KeyboardInterrupt:
            arm.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            arm.move_service_server.destroy()
            arm.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
