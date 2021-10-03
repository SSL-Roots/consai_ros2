# coding: UTF-8

# Copyright 2021 Roots
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
from consai_visualizer.field_widget import FieldWidget
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QWidget
from qt_gui.plugin import Plugin


class Visualizer(Plugin):

    def __init__(self, context):
        super(Visualizer, self).__init__(context)
        self.setObjectName('Visualizer')

        self._context = context
        self._widget = QWidget()

        # widgetを読み込む
        # FieldWidgetはカスタムウィジェットとしてuiファイルに設定済み
        pkg_name = 'consai_visualizer'
        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(
            package_path, 'share', pkg_name, 'resource', 'visualizer.ui')
        loadUi(ui_file, self._widget, {'FieldWidget': FieldWidget})

        # rqtのUIにwidgetを追加する
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # UIのイベントと関数を接続する
        self._widget.check_box_geometry.stateChanged.connect(self._clicked_geometry)
        self._widget.check_box_detection.stateChanged.connect(self._clicked_detection)
        self._widget.check_box_detection_tracked.stateChanged.connect(self._clicked_detection_tracked)

        # 30 msec周期で描画を更新する
        self._timer = QTimer()
        self._timer.timeout.connect(self._widget.field_widget.update)
        self._timer.start(30)

    def _clicked_geometry(self):
        if self._widget.check_box_geometry.isChecked():
            pass
        else:
            pass

    def _clicked_detection(self):
        if self._widget.check_box_detection.isChecked():
            pass
        else:
            pass

    def _clicked_detection_tracked(self):
        if self._widget.check_box_detection_tracked.isChecked():
            pass
        else:
            pass
