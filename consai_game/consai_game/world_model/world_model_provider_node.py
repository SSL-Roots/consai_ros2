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

from consai_game.utils.process_info import process_info
from consai_game.world_model.world_model import WorldModel
import json
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from robocup_ssl_msgs.msg import Referee
from robocup_ssl_msgs.msg import TrackedFrame
from std_msgs.msg import String
import threading


class WorldModelProviderNode(Node):
    def __init__(self, update_hz: float = 10, team_is_yellow: bool = False):
        super().__init__('world_model_provider_node')
        self.lock = threading.Lock()

        self.timer = self.create_timer(1.0/update_hz, self.update)

        self.world_model = WorldModel()
        self.world_model.set_our_team_is_yellow(team_is_yellow)

        self.sub_referee = self.create_subscription(
            Referee, 'referee', self.callback_referee, 10)
        self.sub_detection_traced = self.create_subscription(
            TrackedFrame, 'detection_tracked', self.callback_detection_traced, 10)

        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self._sub_param_rule = self.create_subscription(
            String, 'consai_param/rule', self.callback_param_rule, qos_profile)

    def update(self) -> None:
        with self.lock:
            self.get_logger().debug(f'WorldModelProvider update, {process_info()}')
            self.world_model.robot_activity.update(self.world_model.robots)

    def callback_referee(self, msg: Referee) -> None:
        with self.lock:
            self.world_model.referee.parse_msg(msg)

    def callback_detection_traced(self, msg: TrackedFrame) -> None:
        with self.lock:
            self.world_model.robots.parse_frame(msg)
            self.world_model.ball.parse_frame(msg)
            # ボールの位置情報を更新
            self.world_model.ball_position.update_position(self.world_model.ball.pos)
            # ボールがフィールド内にあるかどうかを更新
            self.world_model.ball_is_on_field = not self.world_model.ball_position.is_outside()


    def callback_param_rule(self, msg: String) -> None:
        with self.lock:
            param_dict = json.loads(msg.data)
            self.world_model.field.length = param_dict['field']['length']
            self.world_model.field.width = param_dict['field']['width']
            self.world_model.field.goal_width = param_dict['field']['goal_width']
            self.world_model.field.penalty_depth = param_dict['field']['penalty_depth']
            self.world_model.field.penalty_width = param_dict['field']['penalty_width']
            self.world_model.field_points = self.world_model.field_points.create_field_points(
                self.world_model.field
            )
