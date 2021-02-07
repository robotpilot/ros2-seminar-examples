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
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from std_msgs.msg import String


class LoggerUsage(Node):

    def __init__(self):
        super().__init__('logger_usage_demo')
        self.pub = self.create_publisher(String, 'logging_demo_count', 10)
        self.timer = self.create_timer(0.500, self.on_timer)
        self.count = 0

    def on_timer(self):
        self.get_logger().log(
            'Timer callback called (this will only log once)',
            LoggingSeverity.INFO,
            once=True)

        msg = String()
        msg.data = 'Current count: {0}'.format(self.count)

        self.get_logger().info('Publishing: {0}'.format(msg.data))
        self.pub.publish(msg)

        # DEBUG FUNCTION
        if self.debug_function_to_evaluate():
            self.get_logger().debug('Count divides into 12 (function evaluated to true)')

        # DEBUG EXPRESSION
        if self.count % 2 == 0:
            self.get_logger().debug('Count is even (expression evaluated to true)')

        self.count += 1
        if self.count > 15:
            self.get_logger().warn('Reseting count to 0')
            self.count = 0

    def debug_function_to_evaluate(self):
        return is_divisor_of_twelve(self.count, self.get_logger())


def is_divisor_of_twelve(val, logger):
    if val == 0:
        logger.error('Modulo divisor cannot be 0')
        return False

    return (12 % val) == 0


def main(args=None):
    rclpy.init(args=args)
    node = LoggerUsage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
