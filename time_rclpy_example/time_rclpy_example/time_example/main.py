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
from std_msgs.msg import Header


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('time_example_node')
    time_publisher = node.create_publisher(Header, 'time', 10)
    msg = Header()

    rate = node.create_rate(1.0)
    duration = Duration(seconds=1, nanoseconds=0)
    past = node.get_clock().now()

    try:
        while rclpy.ok():
            now = node.get_clock().now()
            seconds, nanoseconds = now.seconds_nanoseconds()
            node.get_logger().info('sec {0} nsec {1}'.format(seconds, nanoseconds))

            if ((now - past).nanoseconds * 1e-9) > 5:
                node.get_logger().info('Over 5 seconds!')
                past = node.get_clock().now()

            msg.stamp = (now + duration).to_msg()
            time_publisher.publish(msg)

            rclpy.spin_once(node)
            rate.sleep()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
