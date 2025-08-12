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
import copy
import json
import threading
from typing import Optional

from consai_game.utils.process_info import process_info
from consai_game.evaluation.evaluation import Evaluation
from consai_game.world_model.world_model import WorldModel

from rclpy.node import Node


class EvaluationProviderNode(Node):
    """
    """

    def __init__(self, update_hz: float = 10):
        """
        初期化.

        EvaluationProviderNode を初期化する.
        """
        super().__init__("evaluation_provider_node")
        self.lock = threading.Lock()

        self.timer = self.create_timer(1.0 / update_hz, self.update)

        self.world_model = WorldModel()
        self.evaluation = Evaluation()
        self.evaluation.meta.update_rate = update_hz

        # subscribeするトピック
        # self.msg_param_rule: Optional[String] = None
        # self.msg_param_control: Optional[String] = None
        # self.msg_param_strategy: Optional[String] = None
        # self.msg_motion_commands = MotionCommandArray()

        # consai_referee_parserのための補助情報
        # self.pub_referee_info = self.create_publisher(RefereeSupportInfo, "parsed_referee/referee_support_info", 10)

        # self.sub_referee = self.create_subscription(Referee, "referee", self.callback_referee, 10)
        # self.sub_detection_traced = self.create_subscription(
        #     TrackedFrame, "detection_tracked", self.callback_detection_traced, 10
        # )

        # qos_profile = QoSProfile(
        #     depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
        # )

        # self.sub_param_rule = self.create_subscription(
        #     String, "consai_param/rule", self.callback_param_rule, qos_profile
        # )
        # self.sub_param_control = self.create_subscription(
        #     String, "consai_param/control", self.callback_param_control, qos_profile
        # )
        # self.sub_param_strategy = self.create_subscription(
        #     String, "consai_param/strategy", self.callback_param_strategy, qos_profile
        # )
        # # motion_commandのsubscriber
        # self._sub_motion_command = self.create_subscription(
        #     MotionCommandArray, "motion_commands", self.callback_motion_commands, 10
        # )

    def update(self) -> None:
        """
        タイマーにより定期的に呼び出され、WorldModelの状態を更新する.

        ロボットのアクティビティやボール位置を再計算する.
        """
        with self.lock:
            self.get_logger().debug(f"EvaluationProvider update, {process_info()}")
            # メタ情報を更新
            self.evaluation.meta.update_counter += 1

            # 最適なシュートターゲットを更新
            self.evaluation.kick_target.update(
                self.world_model.ball,
                self.world_model.robots,
            )
