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


from functools import partial
import os

from ament_index_python.resources import get_resource
from consai_msgs.msg import GoalPose
from consai_visualizer.field_widget import FieldWidget
import consai_visualizer.referee_parser as ref_parser
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import QWidget
from qt_gui.plugin import Plugin
from robocup_ssl_msgs.msg import DetectionFrame
from robocup_ssl_msgs.msg import GeometryData
from robocup_ssl_msgs.msg import Referee
from robocup_ssl_msgs.msg import Replacement
from robocup_ssl_msgs.msg import TrackedFrame


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
            GeometryData, 'geometry',
            self._widget.field_widget.set_field, 10)
        self._sub_detection = self._node.create_subscription(
            DetectionFrame, 'detection',
            self._widget.field_widget.set_detection, 10)
        self._sub_detection_tracked = self._node.create_subscription(
            TrackedFrame, 'detection_tracked',
            self._widget.field_widget.set_detection_tracked, 10)
        self._sub_referee = self._node.create_subscription(
            Referee, 'referee',
            self._callback_referee, 10)

        self._sub_goal_pose = []
        for i in range(16):
            topic_name = 'robot' + str(i) + '/goal_pose'
            self._sub_goal_pose.append(self._node.create_subscription(
                GoalPose, topic_name,
                partial(self._widget.field_widget.set_goal_pose, robot_id=i), 10))

        self._widget.field_widget.set_pub_replacement(
            self._node.create_publisher(Replacement, 'replacement', 1))

        # UIのイベントと関数を接続する
        self._widget.check_box_geometry.stateChanged.connect(self._clicked_geometry)
        self._widget.check_box_detection.stateChanged.connect(self._clicked_detection)
        self._widget.check_box_detection_tracked.stateChanged.connect(
            self._clicked_detection_tracked)
        self._widget.check_box_replacement.stateChanged.connect(self._clicked_replacement)

        # チェックボックスを操作する
        self._widget.check_box_geometry.setCheckState(Qt.Checked)
        self._widget.check_box_detection.setCheckState(Qt.Unchecked)
        self._widget.check_box_detection_tracked.setCheckState(Qt.Checked)

        # 16 msec周期で描画を更新する
        self._timer = QTimer()
        self._timer.timeout.connect(self._widget.field_widget.update)
        self._timer.start(16)

        # 5000 msec周期で描画情報をリセットする
        self._reset_timer = QTimer()
        self._reset_timer.timeout.connect(self._widget.field_widget.reset_topics)
        self._reset_timer.start(5000)

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
            self._widget.field_widget.set_can_draw_detection_tracked(True)
        else:
            self._widget.field_widget.set_can_draw_detection_tracked(False)

    def _clicked_replacement(self):
        if self._widget.check_box_replacement.isChecked():
            self._widget.field_widget.set_can_draw_replacement(True)
        else:
            self._widget.field_widget.set_can_draw_replacement(False)

    def _callback_referee(self, msg):
        self._widget.label_ref_stage.setText(ref_parser.parse_stage(msg.stage))
        self._widget.label_ref_command.setText(ref_parser.parse_command(msg.command))
        if len(msg.stage_time_left) > 0:
            self._widget.label_ref_stage_time_left.setText(
                ref_parser.parse_stage_time_left(msg.stage_time_left[0]))
        if len(msg.current_action_time_remaining) > 0:
            self._widget.label_ref_action_time_remaining.setText(
                ref_parser.parse_action_time_remaining(msg.current_action_time_remaining[0]))

        # チーム情報の解析
        self._widget.label_ref_b_team_red_num.setText(
            ref_parser.parse_red_cards(msg.blue.red_cards))
        self._widget.label_ref_y_team_red_num.setText(
            ref_parser.parse_red_cards(msg.yellow.red_cards))
        self._widget.label_ref_b_team_yellow_num.setText(
            ref_parser.parse_yellow_cards(msg.blue.yellow_cards))
        self._widget.label_ref_y_team_yellow_num.setText(
            ref_parser.parse_yellow_cards(msg.yellow.yellow_cards))
        self._widget.label_ref_b_team_yellow_time.setText(
            ref_parser.parse_yellow_card_times(msg.blue.yellow_card_times))
        self._widget.label_ref_y_team_yellow_time.setText(
            ref_parser.parse_yellow_card_times(msg.yellow.yellow_card_times))
        self._widget.label_ref_b_team_timeouts.setText(
            ref_parser.parse_timeouts(msg.blue.timeouts))
        self._widget.label_ref_y_team_timeouts.setText(
            ref_parser.parse_timeouts(msg.yellow.timeouts))
        self._widget.label_ref_b_team_timeout_time.setText(
            ref_parser.parse_timeout_time(msg.blue.timeout_time))
        self._widget.label_ref_y_team_timeout_time.setText(
            ref_parser.parse_timeout_time(msg.yellow.timeout_time))

        # ボール配置の目標位置をセット
        if ref_parser.is_ball_placement(msg.command):
            if len(msg.designated_position) > 0:
                self._widget.field_widget.set_designated_position(msg.designated_position[0])
