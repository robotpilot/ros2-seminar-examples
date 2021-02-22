#!/usr/bin/env python3
# Copyright 2019 OROCA
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

import os

from ament_index_python.resources import get_resource
from geometry_msgs.msg import Twist
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QShortcut
from python_qt_binding.QtWidgets import QWidget
import rclpy
from rclpy.qos import QoSProfile
from std_srvs.srv import SetBool


class ExamplesWidget(QWidget):

    def __init__(self, node):
        super(ExamplesWidget, self).__init__()
        self.setObjectName('ExamplesWidget')

        self.node = node

        self.REDRAW_INTERVAL = 30
        self.PUBLISH_INTERVAL = 100
        self.CMD_VEL_X_FACTOR = 1000.0
        self.CMD_VEL_YAW_FACTOR = -10.0

        pkg_name = 'rqt_example'
        ui_filename = 'rqt_example.ui'
        topic_name = 'cmd_vel'
        service_name = 'led_control'

        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)

        self.pub_velocity = Twist()
        self.pub_velocity.linear.x = 0.0
        self.pub_velocity.angular.z = 0.0
        self.sub_velocity = Twist()
        self.sub_velocity.linear.x = 0.0
        self.sub_velocity.angular.z = 0.0

        self.slider_x.setValue(0)
        self.lcd_number_x.display(0.0)
        self.lcd_number_yaw.display(0.0)

        qos = QoSProfile(depth=10)
        self.publisher = self.node.create_publisher(Twist, topic_name, qos)
        self.subscriber = self.node.create_subscription(Twist, topic_name, self.get_velocity, qos)
        self.service_server = self.node.create_service(SetBool, service_name, self.set_led_status)
        self.service_client = self.node.create_client(SetBool, service_name)

        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self.send_velocity)
        self.publish_timer.start(self.PUBLISH_INTERVAL)

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_indicators)
        self.update_timer.start(self.REDRAW_INTERVAL)

        self.push_button_w.pressed.connect(self.increase_linear_x)
        self.push_button_x.pressed.connect(self.decrease_linear_x)
        self.push_button_a.pressed.connect(self.increase_angular_z)
        self.push_button_d.pressed.connect(self.decrease_angular_z)
        self.push_button_s.pressed.connect(self.set_stop)

        self.push_button_w.setShortcut('w')
        self.push_button_x.setShortcut('x')
        self.push_button_a.setShortcut('a')
        self.push_button_d.setShortcut('d')
        self.push_button_s.setShortcut('s')

        self.shortcut_space = QShortcut(QKeySequence(Qt.Key_Space), self)
        self.shortcut_space.setContext(Qt.ApplicationShortcut)
        self.shortcut_space.activated.connect(self.push_button_s.pressed)

        self.radio_button_led_on.clicked.connect(self.call_led_service)
        self.radio_button_led_off.clicked.connect(self.call_led_service)

        self.radio_button_led_on.setShortcut('o')
        self.radio_button_led_off.setShortcut('f')

    def get_velocity(self, msg):
        self.sub_velocity = msg

    def set_led_status(self, request, response):
        if request.data:
            self.push_button_led_status.setText('ON')
            self.push_button_led_status.setStyleSheet('color: rgb(255, 170, 0);')
            response.success = True
            response.message = 'LED ON'
        elif not request.data:
            self.push_button_led_status.setText('OFF')
            self.push_button_led_status.setStyleSheet('')
            response.success = True
            response.message = 'LED OFF'
        else:
            response.success = False
        return response

    def increase_linear_x(self):
        self.pub_velocity.linear.x += 0.1

    def decrease_linear_x(self):
        self.pub_velocity.linear.x -= 0.1

    def increase_angular_z(self):
        self.pub_velocity.angular.z += 0.1

    def decrease_angular_z(self):
        self.pub_velocity.angular.z -= 0.1

    def set_stop(self):
        self.pub_velocity.linear.x = 0.0
        self.pub_velocity.angular.z = 0.0

    def call_led_service(self):
        request = SetBool.Request()

        if self.radio_button_led_on.isChecked():
            request.data = True
        elif self.radio_button_led_off.isChecked():
            request.data = False

        wait_count = 1
        while not self.service_client.wait_for_service(timeout_sec=0.5):
            if wait_count > 5:
                return
            self.node.get_logger().error('Service not available #{0}'.format(wait_count))
            wait_count += 1

        future = self.service_client.call_async(request)

        while rclpy.ok():
            if future.done():
                if future.result() is not None:
                    response = future.result()
                    self.node.get_logger().info(
                        'Result of service call: {0}'.format(response.message))
                else:
                    self.node.get_logger().error('Error calling service')
                break

    def send_velocity(self):
        twist = Twist()
        twist.linear.x = self.pub_velocity.linear.x
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.pub_velocity.angular.z
        self.publisher.publish(twist)

    def update_indicators(self):
        self.slider_x.setValue(self.sub_velocity.linear.x * self.CMD_VEL_X_FACTOR)
        self.dial_yaw.setValue(self.sub_velocity.angular.z * self.CMD_VEL_YAW_FACTOR)
        self.lcd_number_x.display(self.sub_velocity.linear.x)
        self.lcd_number_yaw.display(self.sub_velocity.angular.z)

    def shutdown_widget(self):
        self.update_timer.stop()
        self.publish_timer.stop()
        self.node.destroy_client(self.service_client)
        self.node.destroy_service(self.service_server)
        self.node.destroy_subscription(self.subscriber)
        self.node.destroy_publisher(self.publisher)
