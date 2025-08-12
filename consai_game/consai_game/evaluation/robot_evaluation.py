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


import numpy as np
from consai_msgs.msg import State2D

from consai_tools.geometry import geometry_tools as tools

from consai_game.world_model.world_model import WorldModel

from consai_game.utils.generate_dummy_ball_position import generate_dummy_ball_position

from consai_game.world_model.robots_model import Robot, RobotsModel
from consai_game.world_model.ball_model import BallModel
from consai_game.world_model.field_model import Field, FieldPoints
from dataclasses import dataclass
from typing import List, Dict

from consai_game.utils.geometry import Point

from consai_game.world_model.ball_activity_model import BallActivityModel
from consai_game.world_model.game_config_model import GameConfigModel


# kick.py
def robot_is_backside(robot_id: int, world_model: WorldModel, target_pos: State2D, angle_ball_to_robot_threshold: int) -> bool:
    """ボールからターゲットを見て、ロボットが後側に居るかを判定する."""

    robot_pos = world_model.robots.our_robots.get(robot_id).pos
    
    # ボールが消えることを想定して、仮想的なボール位置を生成する
    ball_pos = generate_dummy_ball_position(ball=world_model.ball, robot_pos=robot_pos)

    # ボールからターゲットへの座標系を作成
    trans = tools.Trans(ball_pos, tools.get_angle(ball_pos, target_pos))
    tr_robot_pos = trans.transform(robot_pos)

    # ボールから見たロボットの位置の角度
    # ボールの後方にいれば角度は90度以上
    tr_ball_to_robot_angle = tools.get_angle(State2D(x=0.0, y=0.0), tr_robot_pos)

    if abs(tr_ball_to_robot_angle) > np.deg2rad(angle_ball_to_robot_threshold):
        return True
    return False

# kick.py
def robot_is_on_kick_line(
    robot_id: int, world_model: WorldModel, target_pos: State2D, width_threshold: float
) -> bool:
    """ボールからターゲットまでの直線上にロボットが居るかを判定する.

    ターゲットまでの距離が遠いと、角度だけで狙いを定めるのは難しいため、位置を使って判定する.
    """
    MINIMAL_THETA_THRESHOLD = 45  # 最低限満たすべきロボットの角度

    robot_pos = world_model.robots.our_robots.get(robot_id).pos

    # ボールが消えることを想定して、仮想的なボール位置を生成する
    ball_pos = generate_dummy_ball_position(ball=world_model.ball, robot_pos=robot_pos)
    
    # ボールからターゲットへの座標系を作成
    trans = tools.Trans(ball_pos, tools.get_angle(ball_pos, target_pos))
    tr_robot_pos = trans.transform(robot_pos)
    tr_robot_theta = trans.transform_angle(robot_pos.theta)

    # ボールより前にロボットが居る場合
    if tr_robot_pos.x > 0.0:
        return False

    # ターゲットを向いていない
    if abs(tr_robot_theta) > np.deg2rad(MINIMAL_THETA_THRESHOLD):
        return False

    if abs(tr_robot_pos.y) > width_threshold:
        return False

    return True


# dribble.py
def ball_is_front(ball_pos: State2D, robot_pos: State2D, target_pos: State2D) -> bool:
    """ボールがロボットの前にあるかどうかを判定する."""
    FRONT_DIST_THRESHOLD = 0.15  # 正面方向にどれだけ離れることを許容するか
    SIDE_DIST_THRESHOLD = 0.05  # 横方向にどれだけ離れることを許容するか

    # ロボットを中心に、ターゲットを+x軸とした座標系を作る
    trans = tools.Trans(robot_pos, tools.get_angle(robot_pos, target_pos))
    tr_ball_pos = trans.transform(ball_pos)

    # ボールがロボットの後ろにある
    if tr_ball_pos.x < 0:
        return False

    # ボールが正面から離れすぎている
    if tr_ball_pos.x > FRONT_DIST_THRESHOLD:
        return False

    # ボールが横方向に離れすぎている
    if abs(tr_ball_pos.y) > SIDE_DIST_THRESHOLD:
        return False
    return True


# threats_model.py
@dataclass
class Threat:
    score: int  # 0以上
    robot_id: int  # 相手のロボットID

class ThreatsModel:
    def __init__(self, field: Field, field_points: FieldPoints):
        self.threats: List[Threat] = []
        self._field = field
        self._field_points = field_points
        self._prev_scores: Dict[int, float] = {}  # ロボットIDごとの前回のスコア
        self._alpha = 0.1  # ローパスフィルターの係数（0-1、小さいほど変化が遅い）

    def _apply_low_pass_filter(self, robot_id: int, new_score: float) -> float:
        """ローパスフィルターを適用してスコアを平滑化する

        Args:
            robot_id: ロボットID
            new_score: 新しいスコア

        Returns:
            平滑化されたスコア
        """
        if robot_id not in self._prev_scores:
            self._prev_scores[robot_id] = new_score
            return new_score

        # 前回のスコアと新しいスコアを重み付けして結合
        filtered_score = self._prev_scores[robot_id] * (1 - self._alpha) + new_score * self._alpha
        self._prev_scores[robot_id] = filtered_score
        return filtered_score

    def update(self, ball: BallModel, robots: RobotsModel):
        """敵ロボットの驚異度を更新する

        Args:
            ball: ボールの情報
            robots: ロボットのリスト
        """
        self.threats = []

        # 敵ロボットのみを対象とする（is_visibleがtrueのものだけ）
        for robot_id, robot in robots.their_robots.items():
            if not robot.is_visible:
                continue

            # A: ゴールへの距離を計算
            goal = State2D(x=self._field_points.our_goal_top.x, y=0.0)
            goal_distance = tools.get_distance(goal, robot.pos)

            # B: シュートできるか（ゴールとの間に障害物があるか）
            can_shoot = True
            for other_robot in robots.our_robots.values():
                if not other_robot.is_visible:
                    continue
                # ロボットとゴールを結ぶ直線上に他のロボットがいるかチェック
                if tools.is_on_line(
                    pose=other_robot.pos, line_pose1=robot.pos, line_pose2=goal, tolerance=0.1  # ロボットの半径を考慮
                ):
                    can_shoot = False
                    break

            # C: ボールとの距離を計算
            ball_distance = tools.get_distance(robot.pos, ball.pos)

            # 各要素のスコアを計算
            # A: ゴールへの距離（近いほど高スコア）
            max_distance = self._field.length
            score_a = int((max_distance - goal_distance) * 100 / max_distance)

            # B: シュートできるか（できる場合高スコア）
            score_b = 100 if can_shoot else 0

            # C: ボールとの距離（近いほど高スコア）
            max_ball_distance = self._field.length
            score_c = int((max_ball_distance - ball_distance) * 100 / max_ball_distance)

            # 総合スコアを計算
            # Bは一旦無視
            total_score = int(score_a * 0.8 + score_b * 0.0 + score_c * 0.2)

            # ローパスフィルターを適用
            filtered_score = self._apply_low_pass_filter(robot_id, total_score)

            threat = Threat(score=int(filtered_score), robot_id=robot_id)
            self.threats.append(threat)

        # スコアの高い順にソート
        self.threats.sort(key=lambda x: x.score, reverse=True)


# kick_target_model.py
def _obstacle_exists(target: State2D, ball: BallModel, robots: dict[int, Robot], tolerance) -> bool:
    """ターゲット位置に障害物（ロボット）が存在するかを判定する関数."""
    for robot in robots.values():
        if tools.is_on_line(pose=robot.pos, line_pose1=ball.pos, line_pose2=target, tolerance=tolerance):
            return True
    return False

def _is_robot_inside_pass_area(ball: BallModel, robot: Robot, _half_width: Field) -> bool:
    """味方ロボットがパスを出すロボットとハーフライン両サイドを結んでできる五角形のエリア内にいるかを判別する関数"""

    if robot.pos.x < 0.0:
        return False

    upper_side_slope, upper_side_intercept, flag = tools.get_line_parameter(ball.pos, Point(0.0, _half_width))
    lower_side_slope, lower_side_intercept, flag = tools.get_line_parameter(ball.pos, Point(0.0, -_half_width))

    if upper_side_slope is None or lower_side_slope is None:
        if ball.pos.x > robot.pos.x:
            return False
    else:
        upper_y_on_line = upper_side_intercept + upper_side_slope * robot.pos.x
        lower_y_on_line = lower_side_intercept + lower_side_slope * robot.pos.x
        if robot.pos.y < upper_y_on_line and robot.pos.y < lower_y_on_line:
            return False
    return True


# robot_activity_model.py
"""未完了."""
@dataclass
class ReceiveScore:
    """ボールをどれだけ受け取りやすいかを保持するデータクラス."""

    robot_id: int = 0
    intercept_time: float = float("inf")  # あと何秒後にボールを受け取れるか

def calc_ball_receive_score_list(
    robots: dict[int, Robot], ball: BallModel, ball_activity: BallActivityModel, game_config: GameConfigModel
) -> list[ReceiveScore]:
    """ロボットごとにボールを受け取れるスコアを計算する."""

    # ボールが動いていない場合は、スコアをデフォルト値にする
    if not ball_activity.ball_is_moving:
        return [ReceiveScore(robot_id=robot.robot_id) for robot in robots.values()]

    score_list = []
    for robot in robots.values():
        score_list.append(
            ReceiveScore(
                robot_id=robot.robot_id,
                intercept_time=calc_intercept_time(robot, ball, game_config),
            )
        )

        # intercept_timeが小さい順にソート
        score_list.sort(key=lambda x: x.intercept_time)
        return score_list

def calc_intercept_time(robot: Robot, ball: BallModel, game_config: GameConfigModel) -> float:
    """ロボットがボールを受け取るまでの時間を計算する関数."""

    # ボールを中心に、ボールの速度方向を+x軸にした座標系を作る
    trans = tools.Trans(ball.pos, tools.get_vel_angle(ball.vel))

    # ロボットの位置を変換
    tr_robot_pos = trans.transform(robot.pos)

    # TODO: ボールを後ろから追いかけて受け取れるようになったら、計算を変更する
    if tr_robot_pos.x < 0:
        return float("inf")

    # ロボットからボール軌道まで垂線を引き、
    # その交点にボールが到達するまでの時間を計算する
    ball_arrival_distance = tr_robot_pos.x
    intercept_time = ball_arrival_distance / tools.get_norm(ball.vel)

    # ボールが到達するまでの時間で、ロボットがどれだけ移動できるかを計算する
    # TODO: ロボットの現在速度、加速度を考慮すべき
    available_distance = intercept_time * game_config.robot_max_linear_vel

    # ボール軌道からロボットまでの距離
    robot_arrival_distance = abs(tr_robot_pos.y)

    # ボールが到着するまでにロボットが移動できれば、intercept_timeを返す
    if available_distance >= robot_arrival_distance:
        return intercept_time
    return float("inf")
