# Copyright 2020 ROBOTIS CO., LTD.
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

import random

import rclpy
from rclpy.node import Node

from msg_srv_action_interface_example.srv import ArithmeticOperator


class Operator(Node):

    def __init__(self):
        super().__init__('operator')
        self.arithmetic_service_client = self.create_client(
            ArithmeticOperator,
            'arithmetic_operator')
        self.request = ArithmeticOperator.Request()
        self.timer = self.create_timer(1.0, self.send_request)

    def send_request(self):
        while not self.arithmetic_service_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('The arithmetic_operator service not available.')
        self.request.arithmetic_operator = random.randint(1, 4)
        self.future = self.arithmetic_service_client.call_async(self.request)


def main(args=None):
    rclpy.init(args=args)
    try:
        operator = Operator()
        try:
            while rclpy.ok():
                rclpy.spin_once(operator)
                if operator.future.done():
                    try:
                        response = operator.future.result()
                    except Exception as e:
                        operator.get_logger().info('Service call failed {}'.format(e))
                    else:
                        operator.get_logger().info('Result: {}'.format(response.arithmetic_result))
        except KeyboardInterrupt:
            operator.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            operator.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
