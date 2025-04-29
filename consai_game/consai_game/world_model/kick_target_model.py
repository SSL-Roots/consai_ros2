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
キックターゲットを管理するモジュール.

シュートを試みるための最適なターゲット位置を計算し, キックターゲットの成功率を更新する.
"""

import numpy as np
from operator import attrgetter

from dataclasses import dataclass, field

from consai_msgs.msg import State2D

from consai_tools.geometry import geometry_tools as tool

from consai_game.utils.geometry import Point
from consai_game.world_model.ball_model import BallModel
from consai_game.world_model.field_model import Field
from consai_game.world_model.robots_model import Robot, RobotsModel


@dataclass
class ShootTarget:
    """キックターゲットの位置と成功率を保持するデータクラス."""

    pos: State2D = field(default_factory=State2D)
    success_rate: int = 0


@dataclass
class PassTarget:
    """パスをするロボットの位置と成功率を保持するデータクラス."""

    robot_id: int = 0
    robot_pos: State2D = field(default_factory=State2D)
    success_rate: int = 0


class KickTargetModel:
    """キックターゲットを保持するクラス."""

    def __init__(self):
        """KickTargetModelの初期化関数."""
        self.hysteresis_distance = 0.3
        self.robot_radius = 0.09

        # shoot_targetの位置と成功率を保持するリスト
        self.shoot_target_list: list[ShootTarget] = []
        self._goal_pos_list = [ShootTarget()]

        # pass_targetの位置と成功率を保持するリスト
        self.pass_target_list: list[PassTarget] = []

    def update_field_pos_list(self, field: Field) -> None:
        """フィールドの位置候補を更新する関数."""
        quarter_width = field.half_goal_width / 2
        one_eighth_width = field.half_goal_width / 4

        self._goal_pos_list = [
            ShootTarget(pos=Point(field.half_length, 0.0)),
            ShootTarget(pos=Point(field.half_length, one_eighth_width)),
            ShootTarget(pos=Point(field.half_length, -one_eighth_width)),
            ShootTarget(pos=Point(field.half_length, quarter_width)),
            ShootTarget(pos=Point(field.half_length, -quarter_width)),
            ShootTarget(pos=Point(field.half_length, quarter_width + one_eighth_width)),
            ShootTarget(pos=Point(field.half_length, -(quarter_width + one_eighth_width))),
        ]

        self._half_width = field.half_width

    def update(
        self,
        ball_model: BallModel,
        robots_model: RobotsModel,
    ) -> None:
        """キックターゲットを更新する関数."""
        # 最も成功するshoot_targetの座標を取得
        self.best_shoot_target = self._search_shoot_pos(ball=ball_model, robots=robots_model, search_ours=True)

        self.best_pass_target = self._search_pass_robot(ball=ball_model, robots=robots_model, search_ours=False)

    def _obstacle_exists(self, target: State2D, ball: BallModel, robots: dict[int, Robot], tolerance) -> bool:
        """ターゲット位置に障害物（ロボット）が存在するかを判定する関数."""
        for robot in robots.values():
            if tool.is_on_line(pose=robot.pos, line_pose1=ball.pos, line_pose2=target, tolerance=tolerance):
                return True
        return False

    def _update_shoot_scores(self, ball: BallModel, robots: RobotsModel, search_ours: bool) -> list[ShootTarget]:
        """各シュートターゲットの成功率を計算し, リストを更新する関数."""
        TOLERANCE = self.robot_radius  # ロボット半径
        MARGIN = 1.8  # ディフェンスエリアの距離分マージンを取る
        MAX_DISTANCE_SCORE = 55  # スコア計算時のシュートターゲットの最大スコア
        MAX_ANGLE_SCORE = 45  # スコア計算時のシュートターゲットの最大角度スコア

        for target in self._goal_pos_list:
            score = 0
            if (
                self._obstacle_exists(
                    target=target.pos, ball=ball, robots=robots.our_visible_robots, tolerance=TOLERANCE
                )
                and search_ours
            ):
                target.success_rate = score
            elif self._obstacle_exists(
                target=target.pos, ball=ball, robots=robots.their_visible_robots, tolerance=TOLERANCE
            ):
                target.success_rate = score
            else:
                # ボールからの角度（目標方向がゴール方向と合っているか）
                angle = abs(tool.get_angle(ball.pos, target.pos))
                score += max(0, MAX_ANGLE_SCORE - np.rad2deg(angle) * 0.5)  # 小さい角度（正面）ほど高得点

                # 距離（近いほうが成功率が高そう）
                distance = tool.get_distance(ball.pos, target.pos)
                score += max(
                    0, MAX_DISTANCE_SCORE - (distance - MARGIN) * MAX_DISTANCE_SCORE / 6
                )  # ディフェンスエリア外から6m以内ならOK
                target.success_rate = int(score)

    def _sort_kick_targets_by_success_rate(self, targets: list[ShootTarget]) -> list[ShootTarget]:
        """スコアの高いターゲット順にソートする関数."""
        return sorted(targets, key=attrgetter("success_rate"), reverse=True)

    def _search_shoot_pos(self, ball: BallModel, robots: RobotsModel, search_ours=True) -> ShootTarget:
        """ボールからの直線上にロボットがいないシュート位置を返す関数."""
        RATE_MARGIN = 50  # ヒステリシスのためのマージン
        last_shoot_target_list = self.shoot_target_list.copy()
        self._update_shoot_scores(ball=ball, robots=robots, search_ours=search_ours)
        shoot_target_list = self._goal_pos_list.copy()
        self.shoot_target_list = self._sort_kick_targets_by_success_rate(shoot_target_list)

        if not last_shoot_target_list:
            return self.shoot_target_list[0]

        if self.shoot_target_list[0].pos == last_shoot_target_list[0].pos:
            return self.shoot_target_list[0]

        # ヒステリシス処理
        if self.shoot_target_list[0].success_rate > last_shoot_target_list[0].success_rate + RATE_MARGIN:
            return self.shoot_target_list[0]
        return last_shoot_target_list[0]

    def _is_robot_inside_pass_area(self, ball: BallModel, robot: Robot) -> bool:
        """味方ロボットがパスを出すロボットとハーフライン両サイドを結んでできる五角形のエリア内にいるかを判別する関数"""
        if robot.pos.x < 0.0:
            return False

        upper_side_slope, upper_side_intercept, flag = tool.get_line_parameter(ball.pos, Point(0.0, self._half_width))
        lower_side_slope, lower_side_intercept, flag = tool.get_line_parameter(ball.pos, Point(0.0, -self._half_width))

        if upper_side_slope is None or lower_side_slope is None:
            if ball.pos.x > robot.pos.x:
                return False
        else:
            upper_y_on_line = upper_side_intercept + upper_side_slope * robot.pos.x
            lower_y_on_line = lower_side_intercept + lower_side_slope * robot.pos.x
            if robot.pos.y < upper_y_on_line and robot.pos.y < lower_y_on_line:
                return False
        return True

    def _update_pass_scores(self, ball: BallModel, robots: RobotsModel, search_ours: bool) -> None:
        """各パスターゲットの成功率を計算し, リストを更新する関数."""
        TOLERANCE = self.robot_radius * 2  # ロボット直径
        MARGIN = 1.8  # ディフェンスエリアの距離分マージンを取る
        MAX_DISTANCE_SCORE = 55  # スコア計算時のシュートターゲットの最大スコア
        MAX_ANGLE_SCORE = 45  # スコア計算時のシュートターゲットの最大角度スコア
        pass_target_list = []

        for robot in robots.our_visible_robots.values():
            score = 0
            if (
                self._obstacle_exists(
                    target=robot.pos, ball=ball, robots=robots.our_visible_robots, tolerance=TOLERANCE
                )
                and search_ours
            ):
                score = 0
            elif self._obstacle_exists(
                target=robot.pos, ball=ball, robots=robots.their_visible_robots, tolerance=TOLERANCE
            ):
                score = 0
            elif tool.get_distance(ball.pos, robot.pos) < 0.5:
                score = 0
            elif self._is_robot_inside_pass_area(ball, robot) is False:
                score = 0
            else:
                # ボールとパスを受けるロボットの距離
                distance = tool.get_distance(ball.pos, robot.pos)
                score += max(
                    0, MAX_DISTANCE_SCORE - (distance - MARGIN) * MAX_DISTANCE_SCORE / 4
                )  # ディフェンスエリア外から4m以内ならOK
                # ボールからの角度（目標方向がロボット方向と合っているか）
                angle = abs(tool.get_angle(ball.pos, robot.pos))
                score += max(0, MAX_ANGLE_SCORE - np.rad2deg(angle) * 0.5)  # 小さい角度ほど高得点
                # ロボットと相手ゴールの距離
                distance = tool.get_distance(robot.pos, self._goal_pos_list[0].pos)
                score -= max(0, 20 - (distance - MARGIN) * 10)  # ボールからディフェンスエリアまで2m以内だったら減点
            pass_target_list.append(
                PassTarget(
                    robot_id=robot.robot_id,
                    robot_pos=robot.pos,
                    success_rate=int(score),
                )
            )

        # スコアの高いターゲット順にソート
        self.pass_target_list = sorted(pass_target_list, key=attrgetter("success_rate"), reverse=True)

    def _search_pass_robot(self, ball: BallModel, robots: RobotsModel, search_ours=False) -> PassTarget:
        """パスをするロボットのIDと位置を返す関数."""
        RATE_MARGIN = 50
        last_pass_target_list = self.pass_target_list.copy()
        self._update_pass_scores(ball=ball, robots=robots, search_ours=search_ours)

        if not last_pass_target_list:
            if not self.pass_target_list:
                return PassTarget(robot_id=-1)
            return self.pass_target_list[0]

        if self.pass_target_list[0].robot_pos == last_pass_target_list[0].robot_pos:
            return self.pass_target_list[0]

        # ヒステリシス処理
        if self.pass_target_list[0].success_rate > last_pass_target_list[0].success_rate + RATE_MARGIN:
            return self.pass_target_list[0]
        return last_pass_target_list[0]
