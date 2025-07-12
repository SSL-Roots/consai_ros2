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


from ament_index_python.resources import get_resource
from consai_visualizer.field_widget import FieldWidget
from consai_visualizer_msgs.msg import Objects
from frootspi_msgs.msg import BatteryVoltage
from functools import partial
import json
import math
import os
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtCore import QPointF
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QTreeWidgetItem
from qt_gui.plugin import Plugin
import rclpy
from robocup_ssl_msgs.msg import BallReplacement
from robocup_ssl_msgs.msg import Replacement
from robocup_ssl_msgs.msg import RobotReplacement
import time


class Visualizer(Plugin):
    def __init__(self, context):
        super(Visualizer, self).__init__(context)
        self.setObjectName("Visualizer")

        self._node = context.node
        self._logger = self._node.get_logger()
        self._context = context

        self._widget = QWidget()

        # widgetを読み込む
        # FieldWidgetはカスタムウィジェットとしてuiファイルに設定済み
        pkg_name = "consai_visualizer"
        _, package_path = get_resource("packages", pkg_name)
        ui_file = os.path.join(package_path, "share", pkg_name, "resource", "visualizer.ui")
        loadUi(ui_file, self._widget, {"FieldWidget": FieldWidget})

        # rqtのUIにwidgetを追加する
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (" (%d)" % context.serial_number()))
        context.add_widget(self._widget)

        # loggerをセット
        self._widget.field_widget.set_logger(self._logger)
        self._add_visualizer_layer("caption", "caption")

        # Subscriber、Publisherの作成
        self._sub_battery_voltage = []
        for i in range(16):
            topic_name = "robot" + str(i) + "/battery_voltage"
            self._sub_battery_voltage.append(
                self._node.create_subscription(
                    BatteryVoltage,
                    topic_name,
                    partial(self._callback_battery_voltage, robot_id=i),
                    rclpy.qos.qos_profile_sensor_data,
                )
            )

        self._sub_kicker_voltage = []
        for i in range(16):
            topic_name = "robot" + str(i) + "/kicker_voltage"
            self._sub_kicker_voltage.append(
                self._node.create_subscription(
                    BatteryVoltage,
                    topic_name,
                    partial(self._callback_kicker_voltage, robot_id=i),
                    rclpy.qos.qos_profile_sensor_data,
                )
            )

        self._sub_visualize_objects = self._node.create_subscription(
            Objects, "visualizer_objects", self._callback_visualizer_objects, rclpy.qos.qos_profile_sensor_data
        )

        self._pub_replacement = self._node.create_publisher(Replacement, "replacement", 10)

        # Parameterを設定する
        self._widget.field_widget.set_invert(self._node.declare_parameter("invert", False).value)

        for team in ["blue", "yellow"]:
            for turnon in ["on", "off"]:
                method = "self._widget.btn_all_" + turnon + "_" + team + ".clicked.connect"
                eval(method)(partial(self._publish_all_robot_turnon_replacement, team == "yellow", turnon == "on"))

        # レイヤーツリーの初期設定
        self._widget.layer_widget.itemChanged.connect(self._layer_state_changed)

        # 16 msec周期で描画を更新する
        self._timer = QTimer()
        self._timer.timeout.connect(self._widget.field_widget.update)
        self._timer.timeout.connect(self._publish_replacement)
        self._timer.start(16)

        # ロボットの死活監視
        # 1秒以上バッテリーの電圧が来ていないロボットは死んだとみなす
        self.latest_update_time = [0] * 16
        self._reset_timer = QTimer()
        self._reset_timer.timeout.connect(self._update_robot_synthetics)
        self._reset_timer.start(1000)

        self.latest_battery_voltage = [0] * 16
        self.latest_kicker_voltage = [0] * 16

        # 独自設定ファイル（installディレクトリからsrc/consai_ros2に移動）
        pkg_name = "consai_visualizer"
        _, package_path = get_resource("packages", pkg_name)
        # /home/shuta/ros2_ws/install/consai_visualizer -> /home/shuta/ros2_ws/src/consai_ros2
        base_path = package_path.replace("install/consai_visualizer", "src/consai_ros2")
        self._custom_settings_file = os.path.join(base_path, "consai_visualizer_settings.json")

        # 起動時の復元処理中は保存を無効にするフラグ
        self._is_loading_settings = False
        # 初期化完了フラグ（新しいレイヤー追加時の保存を防ぐ）
        self._initialization_complete = False

    def save_settings(self, plugin_settings, instance_settings):
        # RQT標準の保存（何もしない）
        pass

    def _save_settings_to_file(self):
        # 独自設定ファイルに保存
        try:
            active_layers = self._extract_active_layers()
            if not active_layers:
                self._logger.info("No active layers to save")
                return

            # レイヤー情報を準備
            layer_data = [{"layer": layer, "sub_layer": sub_layer} for layer, sub_layer in active_layers]

            settings = {"active_layers": layer_data, "timestamp": time.time()}

            # ディレクトリが存在することを確認
            settings_dir = os.path.dirname(self._custom_settings_file)
            if not os.path.exists(settings_dir):
                os.makedirs(settings_dir)
                self._logger.info(f"Created directory: {settings_dir}")

            # JSONファイルに保存
            self._logger.info(f"Attempting to save to: {self._custom_settings_file}")
            with open(self._custom_settings_file, "w") as f:
                json.dump(settings, f, indent=2)

            # ファイルが実際に作成されたか確認
            if os.path.exists(self._custom_settings_file):
                self._logger.info(f"Successfully saved {len(layer_data)} layers to {self._custom_settings_file}")
            else:
                self._logger.error(f"File was not created: {self._custom_settings_file}")

        except Exception as e:
            self._logger.error(f"Failed to save custom settings to {self._custom_settings_file}: {e}")
            import traceback

            self._logger.error(f"Traceback: {traceback.format_exc()}")

    def restore_settings(self, plugin_settings, instance_settings):
        # UIが起動したときに実行される関数
        self._load_custom_settings()

    def _load_custom_settings(self):
        # 独自設定ファイルから読み込み
        self._is_loading_settings = True  # 読み込み中フラグをON
        try:
            if not os.path.exists(self._custom_settings_file):
                self._logger.info("No custom settings file found, using defaults")
                self._load_default_settings()
                return

            with open(self._custom_settings_file, "r") as f:
                settings = json.load(f)

            layer_data = settings.get("active_layers", [])
            if not layer_data:
                self._logger.info("No layers in custom settings, using defaults")
                self._load_default_settings()
                return

            # レイヤーを復元
            for item in layer_data:
                if isinstance(item, dict) and "layer" in item and "sub_layer" in item:
                    layer = item["layer"].strip()
                    sub_layer = item["sub_layer"].strip()
                    if layer and sub_layer:
                        self._add_visualizer_layer(layer, sub_layer, Qt.Checked)

            self._logger.info(f"Restored {len(layer_data)} layers from custom settings")

        except Exception as e:
            self._logger.error(f"Failed to load custom settings: {e}")
            self._load_default_settings()
        finally:
            self._is_loading_settings = False  # 読み込み完了、フラグをOFF
            # 起動から少し時間をおいて初期化完了とする（動的レイヤー追加を待つ）
            QTimer.singleShot(3000, self._complete_initialization)

    def _load_default_settings(self):
        # デフォルト設定をロードする
        default_layers = [("caption", "caption")]

        for (layer, sub_layer) in default_layers:
            self._add_visualizer_layer(layer, sub_layer, Qt.Checked)

        self._logger.info("Loaded default layer settings")

    def _complete_initialization(self):
        # 初期化完了、これ以降は保存を有効にする
        self._initialization_complete = True
        self._logger.info("Initialization complete, settings saving enabled")

    def _layer_state_changed(self):
        # レイヤーのチェックボックスが変更されたときに呼ばれる
        # 一括でON/OFFすると項目の数だけ実行される
        active_layers = self._extract_active_layers()
        self._widget.field_widget.set_active_layers(active_layers)

        # 設定読み込み中でなく、かつ初期化完了後なら保存
        if not self._is_loading_settings and self._initialization_complete:
            self._save_settings_to_file()

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
            self._logger.warning("layer={} or sub_layer={} is empty".format(layer, sub_layer))
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

    def _publish_replacement(self) -> None:
        # 描画領域のダブルクリック操作が完了したら、grSimのReplacement情報をpublishする
        if not self._widget.field_widget.get_mouse_double_click_updated():
            return
        start, end = self._widget.field_widget.get_mouse_double_click_points()
        self._widget.field_widget.reset_mouse_double_click_updated()

        # チェックが入ったボタン情報を解析する
        button = self._widget.radio_buttons.checkedButton()
        if button.text() == "NONE":
            return
        elif button.text() == "Ball":
            self._publish_ball_replacement(start, end)
            return
        else:
            is_yellow = False
            if button.text()[0] == "Y":
                is_yellow = True
            robot_id = int(button.text()[1:])
            self._publish_robot_replacement(start, end, is_yellow, robot_id)
            return

    def _publish_ball_replacement(self, start: QPointF, end: QPointF) -> None:
        velocity = end - start
        ball_replacement = BallReplacement()
        ball_replacement.x.append(start.x())
        ball_replacement.y.append(start.y())
        ball_replacement.vx.append(velocity.x())
        ball_replacement.vy.append(velocity.y())
        replacement = Replacement()
        replacement.ball.append(ball_replacement)
        self._pub_replacement.publish(replacement)

    def _publish_robot_replacement(self, start: QPointF, end: QPointF, is_yellow: bool, robot_id: int) -> None:

        theta_deg = math.degrees(math.atan2(end.y() - start.y(), end.x() - start.x()))

        robot_replacement = RobotReplacement()
        robot_replacement.x = start.x()
        robot_replacement.y = start.y()
        robot_replacement.dir = theta_deg
        robot_replacement.id = robot_id
        robot_replacement.yellowteam = is_yellow
        robot_replacement.turnon.append(True)
        replacement = Replacement()
        replacement.robots.append(robot_replacement)
        self._pub_replacement.publish(replacement)

    def _publish_all_robot_turnon_replacement(self, is_yellow, turnon):
        # turnon時はフィールド内に、turnoff時はフィールド外に、ID順でロボットを並べる
        OFFSET_X = 0.2
        team_offset_x = -3.0 if is_yellow else 0.0
        turnon_offset_y = -4.6 if turnon else -5.6

        replacement = Replacement()
        for i in range(11):
            robot_replacement = RobotReplacement()
            robot_replacement.x = team_offset_x + OFFSET_X * i
            robot_replacement.y = turnon_offset_y
            robot_replacement.dir = 90.0
            robot_replacement.id = i
            robot_replacement.yellowteam = is_yellow
            robot_replacement.turnon.append(turnon)
            replacement.robots.append(robot_replacement)
        self._pub_replacement.publish(replacement)

    def _battery_voltage_to_percentage(self, voltage):
        MAX_VOLTAGE = 16.8
        MIN_VOLTAGE = 14.8
        percentage = (voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100
        if percentage < 0:
            percentage = 0
        elif percentage > 100:
            percentage = 100
        return int(percentage)

    def _kicker_voltage_to_percentage(self, voltage):
        MAX_VOLTAGE = 200
        MIN_VOLTAGE = 0
        percentage = (voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100
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
                    self._battery_voltage_to_percentage(self.latest_battery_voltage[i])
                )
                getattr(self._widget, f"robot{i}_kicker_voltage").setValue(
                    self._kicker_voltage_to_percentage(self.latest_kicker_voltage[i])
                )
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
