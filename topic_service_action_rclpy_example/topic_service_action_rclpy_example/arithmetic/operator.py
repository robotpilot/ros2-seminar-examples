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

from msg_srv_action_interface_example.srv import ArithmeticOperator
import rclpy
from rclpy.node import Node


class Operator(Node):

    def __init__(self):
        super().__init__('operator')

        self.arithmetic_service_client = self.create_client(
            ArithmeticOperator,
            'arithmetic_operator')

        while not self.arithmetic_service_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('The arithmetic_operator service not available.')

        self.client_futures = []
        self.request = ArithmeticOperator.Request()

    def send_request(self):
        self.request.arithmetic_operator = random.randint(1, 4)
        self.client_futures.append(self.arithmetic_service_client.call_async(self.request))


def main(args=None):
    rclpy.init(args=args)

    operator = Operator()
    operator.send_request()

    try:
        while rclpy.ok():
            rclpy.spin_once(operator)
            incomplete_futures = []
            for future in operator.client_futures:
                if future.done():
                    response = future.result()
                    operator.get_logger().info('Result: {}'.format(response.arithmetic_result))
                else:
                    incomplete_futures.append(future)
            operator.client_futures = incomplete_futures
    except KeyboardInterrupt:
        operator.get_logger().info('Keyboard Interrupt (SIGINT)')

    operator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
