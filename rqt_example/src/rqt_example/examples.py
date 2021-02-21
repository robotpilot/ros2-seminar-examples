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

from rqt_example.examples_widget import ExamplesWidget

from rqt_gui_py.plugin import Plugin


class Examples(Plugin):

    def __init__(self, context):
        super(Examples, self).__init__(context)
        self.setObjectName('RQt example')
        self.widget = ExamplesWidget(context.node)
        serial_number = context.serial_number()
        if serial_number > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + ' ({0})'.format(serial_number))
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        print('Shutdown the RQt example.')
        self.widget.shutdown_widget()
