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
from consai_game.world_model.referee_model import parse_referee_msg
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommandArray
from consai_msgs.msg import RefereeSupportInfo

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

    def __init__(self, update_hz: float = 10, team_is_yellow: bool = False, goalie_id: int = 0, invert: bool = False):
        """
        初期化.

        WorldModelProviderNode を初期化する.
        """
        super().__init__("world_model_provider_node")
        self.lock = threading.Lock()

        self.timer = self.create_timer(1.0 / update_hz, self.update)

        self.world_model = WorldModel()
        self.world_model.game_config.our_team_is_yellow = team_is_yellow
        self.world_model.game_config.invert = invert
        self.world_model.robots.our_team_is_yellow = team_is_yellow
        self.world_model.game_config.goalie_id = goalie_id
        self.world_model.meta.update_rate = update_hz

        # subscribeするトピック
        self.msg_referee = Referee()
        self.msg_detection_traced = TrackedFrame()
        self.msg_param_rule: Optional[String] = None
        self.msg_param_control: Optional[String] = None
        self.msg_param_strategy: Optional[String] = None
        self.msg_motion_commands = MotionCommandArray()

        # consai_referee_parserのための補助情報
        self.pub_referee_info = self.create_publisher(RefereeSupportInfo, "parsed_referee/referee_support_info", 10)

        self.sub_referee = self.create_subscription(Referee, "referee", self.callback_referee, 10)
        self.sub_detection_traced = self.create_subscription(
            TrackedFrame, "detection_tracked", self.callback_detection_traced, 10
        )

        qos_profile = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
        )

        self.sub_param_rule = self.create_subscription(
            String, "consai_param/rule", self.callback_param_rule, qos_profile
        )
        self.sub_param_control = self.create_subscription(
            String, "consai_param/control", self.callback_param_control, qos_profile
        )
        self.sub_param_strategy = self.create_subscription(
            String, "consai_param/strategy", self.callback_param_strategy, qos_profile
        )
        # motion_commandのsubscriber
        self._sub_motion_command = self.create_subscription(
            MotionCommandArray, "motion_commands", self.callback_motion_commands, 10
        )

    def update(self) -> None:
        """
        タイマーにより定期的に呼び出され、WorldModelの状態を更新する.

        ロボットのアクティビティやボール位置を再計算する.
        """
        with self.lock:
            self.get_logger().debug(f"WorldModelProvider update, {process_info()}")
            # メタ情報を更新
            self.world_model.meta.update_counter += 1

            # ゲーム設定を更新
            self.update_game_config()

            # トラッキング情報を更新
            self.world_model.robots.parse_frame(self.msg_detection_traced)
            self.world_model.ball.parse_frame(self.msg_detection_traced)

            # レフェリー情報を更新
            self.world_model.referee = parse_referee_msg(
                msg=self.msg_referee,
                prev_data=self.world_model.referee,
                our_team_is_yellow=self.world_model.game_config.our_team_is_yellow,
                invert=self.world_model.game_config.invert,
                ball_is_moving=self.world_model.ball_activity.ball_is_moving,
                ball_pos=copy.deepcopy(self.world_model.ball.pos),
            )

            # フィールド情報を更新
            self.update_field_model()

            # ボールの位置情報を更新
            self.world_model.ball_position.update_position(
                self.world_model.ball, self.world_model.field, self.world_model.field_points
            )
            # ボールの活動状態を更新
            self.world_model.ball_activity.update(
                ball=self.world_model.ball,
                robots=self.world_model.robots,
                referee=self.world_model.referee,
                game_config=self.world_model.game_config,
                field_points=self.world_model.field_points,
            )
            # 最適なシュートターゲットを更新
            self.world_model.kick_target.update(
                self.world_model.ball,
                self.world_model.robots,
            )
            # 敵ロボットの驚異度を更新
            self.world_model.threats.update(
                ball=self.world_model.ball,
                robots=self.world_model.robots,
            )
            # ロボットの活動状態を更新
            self.world_model.robot_activity.update(
                robots=self.world_model.robots,
                ball=self.world_model.ball,
                ball_activity=self.world_model.ball_activity,
                game_config=self.world_model.game_config,
                referee=self.world_model.referee,
            )

            # ロボットが目標位置が到達したか更新
            self.world_model.robot_activity.update_our_robots_arrived(
                self.world_model.robots.our_visible_robots, self.msg_motion_commands.commands
            )

            self.publish_referee_support_info()

    def publish_referee_support_info(self) -> None:
        """RefereeSupportInfo をパブリッシュする."""
        msg = RefereeSupportInfo()
        msg.ball_pos = self.world_model.ball.pos
        msg.placement_pos.x = self.world_model.referee.placement_pos.x
        msg.placement_pos.y = self.world_model.referee.placement_pos.y
        msg.blue_robot_num = self.world_model.robots.blue_robot_num
        msg.yellow_robot_num = self.world_model.robots.yellow_robot_num

        self.pub_referee_info.publish(msg)

    def callback_referee(self, msg: Referee) -> None:
        """refereeメッセージを受信する."""
        with self.lock:
            self.msg_referee = msg

    def callback_detection_traced(self, msg: TrackedFrame) -> None:
        """detection_tracedメッセージを受信する."""
        with self.lock:
            self.msg_detection_traced = msg

    def callback_param_rule(self, msg: String) -> None:
        """param/ruleメッセージを受信する."""
        with self.lock:
            self.msg_param_rule = msg

    def callback_param_control(self, msg: String) -> None:
        """param/controlメッセージを受信する."""
        with self.lock:
            self.msg_param_control = msg

    def callback_param_strategy(self, msg: String) -> None:
        """トピック consai_param/strategy からのパラメータを受け取り、モデルを更新する."""
        with self.lock:
            self.msg_param_strategy = msg

    def callback_motion_commands(self, msg: MotionCommandArray) -> None:
        """トピック motion_commands からのパラメータを受け取り更新する."""
        with self.lock:
            self.msg_motion_commands = msg

    def update_field_model(self) -> None:
        """self.msg_param_ruleを元にフィールドモデルを更新する."""
        if self.msg_param_rule is None:
            return

        param_dict = json.loads(self.msg_param_rule.data)
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
        self.world_model.kick_target.update_field_pos_list(self.world_model.field)

    def update_game_config(self) -> None:
        """self.msg_param_control、self.msg_param_strategyを元にゲーム設定を更新する."""
        if self.msg_param_control is not None:
            param_dict = json.loads(self.msg_param_control.data)
            self.world_model.game_config.robot_max_linear_vel = param_dict["soft_limits"]["velocity_xy"]
            self.world_model.game_config.robot_max_angular_vel = param_dict["soft_limits"]["velocity_theta"]
            self.world_model.game_config.robot_max_linear_accel = param_dict["soft_limits"]["acceleration_xy"]
            self.world_model.game_config.robot_max_angular_accel = param_dict["soft_limits"]["acceleration_theta"]

        if self.msg_param_strategy is not None:
            param_dict = json.loads(self.msg_param_strategy.data)
            self.world_model.game_config.gravity = param_dict["physics"]["gravity"]
            self.world_model.game_config.ball_friction_coeff = param_dict["physics"]["ball_friction_coeff"]

            # キック力パラメータの設定
            if "kick_power" in param_dict:
                kick_power = param_dict["kick_power"]
                self.world_model.game_config.max_kick_power = kick_power.get("max_kick_power", 6.0)
                self.world_model.game_config.min_pass_power = kick_power.get("min_pass_power", 2.0)
                self.world_model.game_config.tapping_kick_power = kick_power.get("tapping_kick_power", 2.0)
