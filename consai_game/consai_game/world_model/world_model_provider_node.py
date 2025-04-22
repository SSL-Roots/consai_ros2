#!/usr/bin/env python3
# coding: UTF-8

# Copyright 2025 Roots
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

"""
WorldModelProviderNode モジュール.

このモジュールは ROS2 ノードとして動作する.
Referee メッセージや TrackedFrame を受け取り, ワールドモデルをリアルタイムに更新する.
"""

import json
import threading

from consai_game.utils.process_info import process_info
from consai_game.world_model.referee_model import parse_referee_msg
from consai_game.world_model.world_model import WorldModel

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from robocup_ssl_msgs.msg import Referee
from robocup_ssl_msgs.msg import TrackedFrame

from std_msgs.msg import String


class WorldModelProviderNode(Node):
    """
    WorldModelの状態を他ノードに提供するための ROS2 ノード.

    Referee や TrackedFrame メッセージを受け取り WorldModel を更新する.
    consai_param/rule トピックからフィールド設定を受信し, フィールド寸法と関連情報を更新する.
    """

    def __init__(self, update_hz: float = 10, team_is_yellow: bool = False, goalie_id: int = 0):
        """
        初期化.

        WorldModelProviderNode を初期化する.
        """
        super().__init__("world_model_provider_node")
        self.lock = threading.Lock()

        self.timer = self.create_timer(1.0 / update_hz, self.update)

        self.world_model = WorldModel()
        self.world_model.game_config.our_team_is_yellow = team_is_yellow
        self.world_model.robots.our_team_is_yellow = team_is_yellow
        self.world_model.game_config.goalie_id = goalie_id

        self.sub_referee = self.create_subscription(Referee, "referee", self.callback_referee, 10)
        self.sub_detection_traced = self.create_subscription(
            TrackedFrame, "detection_tracked", self.callback_detection_traced, 10
        )

        qos_profile = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
        )

        self._sub_param_rule = self.create_subscription(
            String, "consai_param/rule", self.callback_param_rule, qos_profile
        )

    def update(self) -> None:
        """
        タイマーにより定期的に呼び出され、WorldModelの状態を更新する.

        ロボットのアクティビティやボール位置を再計算する.
        """
        with self.lock:
            self.get_logger().debug(f"WorldModelProvider update, {process_info()}")
            self.world_model.robot_activity.update(self.world_model.robots)
            # ボールの位置情報を更新
            self.world_model.ball_position.update_position(
                self.world_model.ball, self.world_model.field, self.world_model.field_points
            )
            self.world_model.ball_activity.update(
                ball=self.world_model.ball,
                robots=self.world_model.robots,
                robot_activity=self.world_model.robot_activity,
            )
            # 最適なシュートターゲットを更新
            self.world_model.kick_target.update(
                self.world_model.ball,
                self.world_model.robots,
            )

    def callback_referee(self, msg: Referee) -> None:
        """メッセージ Referee を受信して WorldModel に反映する."""
        with self.lock:
            self.world_model.referee = parse_referee_msg(
                msg=msg,
                prev_data=self.world_model.referee,
                our_team_is_yellow=self.world_model.game_config.our_team_is_yellow,
            )

    def callback_detection_traced(self, msg: TrackedFrame) -> None:
        """メッセージ TrackedFrame を受信してロボットとボールの状態を更新する."""
        with self.lock:
            self.world_model.robots.parse_frame(msg)
            self.world_model.ball.parse_frame(msg)

    def callback_param_rule(self, msg: String) -> None:
        """トピック consai_param/rule からのパラメータを受け取り、フィールドの寸法を更新する."""
        with self.lock:
            param_dict = json.loads(msg.data)
            self.world_model.field.length = param_dict["field"]["length"]
            self.world_model.field.width = param_dict["field"]["width"]
            self.world_model.field.goal_width = param_dict["field"]["goal_width"]
            self.world_model.field.penalty_depth = param_dict["field"]["penalty_depth"]
            self.world_model.field.penalty_width = param_dict["field"]["penalty_width"]

            self.world_model.field.half_length = self.world_model.field.length / 2
            self.world_model.field.half_width = self.world_model.field.width / 2
            self.world_model.field.half_goal_width = self.world_model.field.goal_width / 2
            self.world_model.field.half_penalty_depth = self.world_model.field.penalty_depth / 2
            self.world_model.field.half_penalty_width = self.world_model.field.penalty_width / 2

            self.world_model.field_points = self.world_model.field_points.create_field_points(self.world_model.field)
            self.world_model.kick_target.update_goal_pos_list(self.world_model.field)
