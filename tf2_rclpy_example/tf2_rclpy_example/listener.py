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
from rclpy.node import Node
import rclpy.time
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Listener(Node):

    def __init__(self):
        super().__init__('tf2_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.500, self.listen)

    async def listen(self):
        try:
            transform = await self.tf_buffer.lookup_transform_async(
                'pan',
                'world',
                rclpy.time.Time())
            self.get_logger().info('Got {}'.format(repr(transform)))
        except LookupException as e:
            self.get_logger().error('failed to get transform {}'.format(repr(e)))


def main():
    rclpy.init()
    try:
        listener = Listener()
        try:
            rclpy.spin(listener)
        except KeyboardInterrupt:
            listener.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            listener.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
