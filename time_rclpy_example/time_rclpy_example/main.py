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

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('minimal_publisher')
    time_publisher = self.create_publisher(Header, 'time', 10)
    Header msg

    rate = node.create_rate(1.0)  # Frequency
    Time past = self.get_clock().now()

    while not runner.done:
        Time now = self.get_clock().now()
        node.get_logger().info('sec %ld nsec %ld' % now.seconds(), now.nanoseconds())

        if ((now - past).nanoseconds() * 1e-9) > 5:
            node.get_logger().info('sec %ld nsec %ld' % now.seconds(), now.nanoseconds())
            past = self.get_clock().now()

        Duration duration(1,0)
        msg.stamp = self.get_clock().now().to_msg()
        time_publisher.publish(msg)

        rclpy.spin_some(node)
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
