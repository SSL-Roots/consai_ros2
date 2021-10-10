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
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import QWidget
from qt_gui.plugin import Plugin
from robocup_ssl_msgs.msg import DetectionFrame
from robocup_ssl_msgs.msg import GeometryData
from robocup_ssl_msgs.msg import Replacement


class Visualizer(Plugin):

    def __init__(self, context):
        super(Visualizer, self).__init__(context)
        self.setObjectName('Visualizer')

        self._node = context.node
        self._logger = self._node.get_logger()

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

        # loggerをセット
        self._widget.field_widget.set_logger(self._logger)

        # Subscriber、Publisherの作成
        self._sub_geometry = self._node.create_subscription(
            GeometryData, "geometry", self._callback_geometry, 10)
        self._sub_detection = self._node.create_subscription(
            DetectionFrame, "detection", self._callback_detection, 10)
        self._widget.field_widget.set_pub_replacement(self._node.create_publisher(Replacement, 'replacement', 1))

        # UIのイベントと関数を接続する
        self._widget.check_box_geometry.stateChanged.connect(self._clicked_geometry)
        self._widget.check_box_detection.stateChanged.connect(self._clicked_detection)
        self._widget.check_box_detection_tracked.stateChanged.connect(self._clicked_detection_tracked)
        self._widget.check_box_replacement.stateChanged.connect(self._clicked_replacement)

        # チェックボックスを操作する
        self._widget.check_box_geometry.setCheckState(Qt.Checked)
        self._widget.check_box_detection.setCheckState(Qt.Checked)

        # 16 msec周期で描画を更新する
        self._timer = QTimer()
        self._timer.timeout.connect(self._widget.field_widget.update)
        self._timer.start(16)

    def _clicked_geometry(self):
        if self._widget.check_box_geometry.isChecked():
            self._widget.field_widget.set_can_draw_geometry(True)
        else:
            self._widget.field_widget.set_can_draw_geometry(False)

    def _clicked_detection(self):
        if self._widget.check_box_detection.isChecked():
            self._widget.field_widget.set_can_draw_detection(True)
        else:
            self._widget.field_widget.set_can_draw_detection(False)

    def _clicked_detection_tracked(self):
        if self._widget.check_box_detection_tracked.isChecked():
            pass
        else:
            pass
    
    def _clicked_replacement(self):
        if self._widget.check_box_replacement.isChecked():
            self._widget.field_widget.set_can_draw_replacement(True)
        else:
            self._widget.field_widget.set_can_draw_replacement(False)

    def _callback_geometry(self, msg):
        self._widget.field_widget.set_field(msg.field)

    def _callback_detection(self, msg):
        self._widget.field_widget.set_detection(msg)
