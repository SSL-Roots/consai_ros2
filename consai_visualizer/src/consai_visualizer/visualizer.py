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
from consai_visualizer_msgs.msg import Objects
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QTreeWidgetItem
from qt_gui.plugin import Plugin
from robocup_ssl_msgs.msg import DetectionFrame
from robocup_ssl_msgs.msg import GeometryData
from robocup_ssl_msgs.msg import Referee
from robocup_ssl_msgs.msg import Replacement
from robocup_ssl_msgs.msg import TrackedFrame
from rqt_py_common.ini_helper import pack, unpack
import rclpy

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
        self._add_visualizer_layer("caption", "caption")

        # Subscriber、Publisherの作成
        # self._sub_geometry = self._node.create_subscription(
        #     GeometryData, 'geometry',
        #     self._widget.field_widget.set_field, 10)
        # self._sub_detection = self._node.create_subscription(
        #     DetectionFrame, 'detection',
        #     self._widget.field_widget.set_detection, 10)
        # self._sub_detection_tracked = self._node.create_subscription(
        #     TrackedFrame, 'detection_tracked',
        #     self._widget.field_widget.set_detection_tracked, 10)
        # self._sub_referee = self._node.create_subscription(
        #     Referee, 'referee',
        #     self._callback_referee, 10)
        # self._sub_named_targets = self._node.create_subscription(
        #     NamedTargets, 'named_targets',
        #     self._widget.field_widget.set_named_targets, 10)

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

        self._sub_visualize_objects = self._node.create_subscription(
            Objects, 'visualizer_objects',
            self._callback_visualizer_objects,
            rclpy.qos.qos_profile_sensor_data)

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
                method = "self._widget.chbox_turnon_" + team + \
                    str(robot_id) + ".stateChanged.connect"
                eval(method)(
                    partial(self._clicked_robot_turnon, team == "yellow", robot_id)
                )

            for turnon in ["on", "off"]:
                method = "self._widget.btn_all_" + turnon + "_" + team + ".clicked.connect"
                eval(method)(
                    partial(self._set_all_robot_turnon, team == "yellow", turnon == "on")
                )

        # チェックボックスを操作する
        self._widget.check_box_geometry.setCheckState(Qt.Checked)
        self._widget.check_box_detection.setCheckState(Qt.Unchecked)
        self._widget.check_box_detection_tracked.setCheckState(Qt.Checked)

        # レイヤーツリーの初期設定
        self._widget.layer_widget.itemChanged.connect(self._layer_state_changed)

        # 16 msec周期で描画を更新する
        self._timer = QTimer()
        self._timer.timeout.connect(self._widget.field_widget.update)
        self._timer.timeout.connect(self._update_label_bot_num)
        self._timer.start(16)

        # 5000 msec周期で描画情報をリセットする
        self._draw_reset_timer = QTimer()
        self._draw_reset_timer.timeout.connect(self._widget.field_widget.reset_topics)
        self._draw_reset_timer.start(5000)

        # ロボットの死活監視
        # 1秒以上バッテリーの電圧が来ていないロボットは死んだとみなす
        self.latest_update_time = [0] * 16
        self._reset_timer = QTimer()
        self._reset_timer.timeout.connect(self._update_robot_synthetics)
        self._reset_timer.start(1000)

        self.latest_battery_voltage = [0] * 16
        self.latest_kicker_voltage = [0] * 16

    def save_settings(self, plugin_settings, instance_settings):
        # UIを終了するときに実行される関数

        # layerとsub layerをカンマで結合して保存
        active_layers = self._extract_active_layers()
        combined_layers = list(map(lambda x: x[0] + "," + x[1], active_layers))
        instance_settings.set_value('active_layers', pack(combined_layers))

    def restore_settings(self, plugin_settings, instance_settings):
        # UIが起動したときに実行される関数

        # カンマ結合されたlayerを復元してセット
        combined_layers = unpack(instance_settings.value('active_layers', []))
        active_layers = list(map(lambda x: x.split(","), combined_layers))
        for (layer, sub_layer) in active_layers:
            self._add_visualizer_layer(layer, sub_layer, Qt.Checked)

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
        self._widget.field_widget.append_robot_replacement(
            is_yellow, robot_id, state == Qt.Checked)

    def _layer_state_changed(self):
        # レイヤーのチェックボックスが変更されたときに呼ばれる
        # 一括でON/OFFすると項目の数だけ実行される
        active_layers = self._extract_active_layers()
        self._widget.field_widget.set_active_layers(active_layers)

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

    def _callback_visualizer_objects(self, msg):
        # ここでレイヤーを更新する
        self._add_visualizer_layer(msg.layer, msg.sub_layer)
        self._widget.field_widget.set_visualizer_objects(msg)

    def _add_visualizer_layer(self, layer: str, sub_layer: str, state=Qt.Unchecked):
        # レイヤーに重複しないように項目を追加する
        if layer == "" or sub_layer == "":
            self._logger.warning(
                "layer={} or sub_layer={} is empty".format(layer, sub_layer))
            return

        parents = self._widget.layer_widget.findItems(layer, Qt.MatchExactly, 0)

        if len(parents) == 0:
            new_parent = QTreeWidgetItem(self._widget.layer_widget)
            new_parent.setText(0, layer)
            new_parent.setFlags(new_parent.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)

            new_child = QTreeWidgetItem(new_parent)
        else:
            for i in range(parents[0].childCount()):
                if sub_layer == parents[0].child(i).text(0):
                    return
            new_child = QTreeWidgetItem(parents[0])

        new_child.setText(0, sub_layer)
        new_child.setFlags(new_child.flags() | Qt.ItemIsUserCheckable)
        new_child.setCheckState(0, state)

    def _extract_active_layers(self) -> list[tuple[str, str]]:
        # チェックが入ったレイヤーを抽出する
        active_layers = []
        for index in range(self._widget.layer_widget.topLevelItemCount()):
            parent = self._widget.layer_widget.topLevelItem(index)
            if parent.checkState(0) == Qt.Unchecked:
                continue

            for child_index in range(parent.childCount()):
                child = parent.child(child_index)
                if child.checkState(0) != Qt.Checked:
                    continue
                active_layers.append((parent.text(0), child.text(0)))

        return active_layers

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
                getattr(self._widget, f"robot{i}_battery_voltage").setValue(
                    self.battery_voltage_to_percentage(self.latest_battery_voltage[i]))
                getattr(self._widget, f"robot{i}_kicker_voltage").setValue(
                    self.kicker_voltage_to_percentage(self.latest_kicker_voltage[i]))
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
