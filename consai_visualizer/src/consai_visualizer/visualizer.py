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
import time

from ament_index_python.resources import get_resource
from consai_msgs.msg import GoalPoses
from consai_msgs.msg import NamedTargets
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

from frootspi_msgs.msg import BatteryVoltage


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
        self._sub_named_targets = self._node.create_subscription(
            NamedTargets, 'named_targets',
            self._widget.field_widget.set_named_targets, 10)

        self._sub_battery_voltage = []
        for i in range(16):
            topic_name = 'robot' + str(i) + '/battery_voltage'
            self._sub_battery_voltage.append(self._node.create_subscription(
                BatteryVoltage, topic_name,
                partial(self._callback_battery_voltage, robot_id=i), 10))

        self._sub_kicker_voltage = []
        for i in range(16):
            topic_name = 'robot' + str(i) + '/kicker_voltage'
            self._sub_kicker_voltage.append(self._node.create_subscription(
                BatteryVoltage, topic_name,
                partial(self._callback_kicker_voltage, robot_id=i), 10))

        self._sub_goal_poses = self._node.create_subscription(
                GoalPoses, 'goal_poses', self._widget.field_widget.set_goal_poses, 10)
        self._sub_final_goal_poses = self._node.create_subscription(
                GoalPoses, 'final_goal_poses', self._widget.field_widget.set_final_goal_poses, 10)

        self._widget.field_widget.set_pub_replacement(
            self._node.create_publisher(Replacement, 'replacement', 1))

         # Parameterを設定する
        self._widget.field_widget.set_invert(self._node.declare_parameter('invert', False).value)

        # UIのイベントと関数を接続する
        self._widget.check_box_geometry.stateChanged.connect(self._clicked_geometry)
        self._widget.check_box_detection.stateChanged.connect(self._clicked_detection)
        self._widget.check_box_detection_tracked.stateChanged.connect(
            self._clicked_detection_tracked)
        self._widget.check_box_replacement.stateChanged.connect(self._clicked_replacement)

        # チェックボックスは複数あるので、文字列をメソッドに変換してconnect文を簡単にする
        for team in ["blue", "yellow"]:
            for robot_id in range(11):
                method = "self._widget.chbox_turnon_" + team + str(robot_id) + ".stateChanged.connect"
                eval(method)(
                    partial(self._clicked_robot_turnon, team=="yellow", robot_id )
                )

            for turnon in ["on", "off"]:
                method = "self._widget.btn_all_" + turnon + "_" + team + ".clicked.connect"
                eval(method)(
                    partial(self._set_all_robot_turnon, team=="yellow", turnon=="on")
                )

        # チェックボックスを操作する
        self._widget.check_box_geometry.setCheckState(Qt.Checked)
        self._widget.check_box_detection.setCheckState(Qt.Unchecked)
        self._widget.check_box_detection_tracked.setCheckState(Qt.Checked)

        # 16 msec周期で描画を更新する
        self._timer = QTimer()
        self._timer.timeout.connect(self._widget.field_widget.update)
        self._timer.timeout.connect(self._update_label_bot_num)
        self._timer.start(16)

        # 5000 msec周期で描画情報をリセットする
        self._reset_timer = QTimer()
        self._reset_timer.timeout.connect(self._widget.field_widget.reset_topics)
        self._reset_timer.start(5000)

        # ロボットの死活監視
        # 1秒以上バッテリーの電圧が来ていないロボットは死んだとみなす
        self.latest_update_time = [0] * 16
        self._reset_timer = QTimer()
        self._reset_timer.timeout.connect(self._update_robot_synthetics)
        self._reset_timer.start(1000)

        self.latest_battery_voltage = [0] * 16
        self.latest_kicker_voltage = [0] * 16
        

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

    def _clicked_robot_turnon(self, is_yellow, robot_id, state):
        self._widget.field_widget.append_robot_replacement(is_yellow, robot_id, state==Qt.Checked)

    def _set_all_robot_turnon(self, is_yellow, turnon):
        team = "yellow" if is_yellow else "blue"
        checked = Qt.Checked if turnon else Qt.Unchecked
        for robot_id in range(11):
            method = "self._widget.chbox_turnon_" + team + str(robot_id) + ".setCheckState"
            eval(method)(checked)

    def _update_label_bot_num(self):
        self._widget.label_b_bot_num.setText(str(self._widget.field_widget.get_blue_robot_num()))
        self._widget.label_y_bot_num.setText(str(self._widget.field_widget.get_yellow_robot_num()))

    def _callback_referee(self, msg):
        self._widget.label_ref_stage.setText(ref_parser.parse_stage(msg.stage))
        self._widget.label_ref_command.setText(ref_parser.parse_command(msg.command))
        if msg.stage_time_left:
            self._widget.label_ref_stage_time_left.setText(
                ref_parser.parse_stage_time_left(msg.stage_time_left[0]))
        if msg.current_action_time_remaining:
            self._widget.label_ref_action_time_remaining.setText(
                ref_parser.parse_action_time_remaining(msg.current_action_time_remaining[0]))

        # チーム情報の解析
        if msg.blue.max_allowed_bots:
            self._widget.label_ref_b_allowed_bots.setText(
                ref_parser.parse_allowed_bots(msg.blue.max_allowed_bots[0]))
        if msg.yellow.max_allowed_bots:
            self._widget.label_ref_y_allowed_bots.setText(
                ref_parser.parse_allowed_bots(msg.yellow.max_allowed_bots[0]))

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

    def _callback_battery_voltage(self, msg, robot_id):
        self.latest_battery_voltage[robot_id] = msg.voltage
        # for synthetics
        self.latest_update_time[robot_id] = time.time()

    def _callback_kicker_voltage(self, msg, robot_id):
        self.latest_kicker_voltage[robot_id] = msg.voltage

    def battery_voltage_to_percentage(self, voltage):
        MAX_VOLTAGE = 16.8
        MIN_VOLTAGE = 14.8
        percentage = (voltage - MIN_VOLTAGE) / (MAX_VOLTAGE-MIN_VOLTAGE) * 100
        if percentage < 0:
            percentage = 0
        elif percentage > 100:
            percentage = 100
        return int(percentage)

    def kicker_voltage_to_percentage(self, voltage):
        MAX_VOLTAGE = 200
        MIN_VOLTAGE = 0
        percentage = (voltage - MIN_VOLTAGE) / (MAX_VOLTAGE-MIN_VOLTAGE) * 100
        if percentage < 0:
            percentage = 0
        elif percentage > 100:
            percentage = 100
        return int(percentage)

    def _update_robot_synthetics(self):
        # n秒以上バッテリーの電圧が来ていないロボットは死んだとみなす
        now = time.time()

        for i in range(16):
            diff_time = now - self.latest_update_time[i]

            try: 
                getattr(self._widget, f"robot{i}_battery_voltage").setValue(self.battery_voltage_to_percentage(self.latest_battery_voltage[i]))
                getattr(self._widget, f"robot{i}_kicker_voltage").setValue(self.kicker_voltage_to_percentage(self.latest_kicker_voltage[i]))
            except AttributeError:
                # ロボット状態表示UIは12列しか用意されておらず、ID=12以降が来るとエラーになるため回避
                pass

            if diff_time > 3.0:  # 死んだ判定
                # DEATH
                try:
                    getattr(self._widget, f"robot{i}_connection_status").setText("❌")
                    getattr(self._widget, f"robot{i}_battery_voltage").setValue(0)
                    getattr(self._widget, f"robot{i}_kicker_voltage").setValue(0)
                except AttributeError:
                    # ロボット状態表示UIは12列しか用意されておらず、ID=12以降が来るとエラーになるため回避
                    pass
            else:
                # ALIVE
                try:
                    getattr(self._widget, f"robot{i}_connection_status").setText("👍")
                except AttributeError:
                    # ロボット状態表示UIは12列しか用意されておらず、ID=12以降が来るとエラーになるため回避
                    pass
