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
ボールの状態とボール保持者を管理するモジュール.

ボールの状態や移動状況を更新し, ボールを保持しているロボットを追跡する機能を提供する.
"""

from copy import deepcopy
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

from consai_tools.geometry import geometry_tools as tools

from consai_game.world_model.ball_model import BallModel
from consai_game.world_model.robots_model import Robot, RobotsModel
from consai_game.world_model.referee_model import RefereeModel
from consai_game.world_model.game_config_model import GameConfigModel
from consai_game.world_model.field_model import FieldPoints

from consai_msgs.msg import State2D


class BallState(Enum):
    """ボールの状態を表す列挙型."""

    FREE = 0  # だれにも所持されていない自由な状態
    OURS = auto()  # 自チームが所持している状態
    THEIRS = auto()  # 相手チームが所持している状態
    OURS_KICKED = auto()  # 自チームのロボットにキックされて転がっている状態
    THEIRS_KICKED = auto()  # 相手チームのロボットにキックされて転がっている状態
    LOOSE = auto()  # 意図せず転がっている状態


@dataclass
class BallHolder:
    """ボールを持っているロボットの情報を保持するクラス."""

    is_our_team: bool
    robot: Robot


class BallActivityModel:
    """ボールの活動状態を保持するクラス."""

    # ボールが動いたと判断する距離
    # ここが小さすぎると、ノイズによって動いた判定になってしまう
    BALL_MOVING_DIST_THRESHOLD = 0.3
    HAS_BALL_DISTANCE_THRESHOLD = BALL_MOVING_DIST_THRESHOLD + 0.05  # ボールとロボットの距離の閾値
    HAS_BALL_MARGIN = 0.1  # ヒステリシス処理に使用する
    MOVING_VELOCITY_THRESHOLD = 0.1  # ボールが動いているとみなす速度の閾値
    MOVING_VELOCITY_MARGIN = 0.05  # ヒステリシス処理に使用する
    # 速度に対するボール移動量を算出する比率[s]: 実質的に移動時間
    MOVEMENT_GAIN = 0.1

    def __init__(self):
        """BallActivityModelの初期化処理."""
        self.ball_state = BallState.FREE
        self.ball_holder: Optional[BallHolder] = None
        self.ball_is_moving = False
        self.ball_is_on_placement_area = False  # ボールがプレースメントエリアにあるか

        # ボールの移動量
        self.ball_movement = State2D()
        # ボールの将来の予測位置
        self.next_ball_pos = State2D()
        # ボールの軌道角度
        self.angle_trajectory = 0.0

        # ボールが最終的に止まる予測位置
        self.ball_stop_position = State2D()
        # ボールが相手のゴールに入るか
        self.ball_will_enter_their_goal = False

        # ボール移動判定用の変数
        self.last_ball_pos_to_detect_moving: Optional[State2D] = None

    def update(
        self,
        ball: BallModel,
        robots: RobotsModel,
        referee: RefereeModel,
        game_config: GameConfigModel,
        field_points: FieldPoints,
    ):
        """ボールの様々な状態を更新するメソッド."""
        # ボール保持者が有効か確認する
        if not self.validate_and_update_ball_holder(ball, robots):
            self.ball_holder = None

        # ボール保持者を探索する
        self.ball_holder = self.search_new_ball_holder(
            ball=ball,
            robots=robots,
        )
        # ボールの移動状態を更新する
        self.ball_is_moving = self.is_ball_moving(ball)

        # ボールの予測位置を更新する
        self.prediction_next_ball_pos(ball)

        # 最終的なボール状態を更新する
        self.update_ball_state()

        # ボールがプレースメントエリアにあるかを更新する
        self.update_ball_on_placement_area(ball, referee)

        # ボールの最終的な停止位置を予測する
        self.ball_stop_position = self.predict_ball_stop_position(ball=ball, game_config=game_config)

        # ボールが相手のゴールに入るかを判定する
        self.ball_will_enter_their_goal = self.is_ball_will_enter_their_goal(
            ball=ball,
            field_points=field_points,
        )

    def update_ball_state(self):
        """ボールの状態を更新するメソッド."""
        # ボールが動いている場合は、前回の状態をもとに、今の状態を判定する
        if self.ball_is_moving:
            if self.ball_state in (BallState.OURS, BallState.OURS_KICKED):
                self.ball_state = BallState.OURS_KICKED
            elif self.ball_state in (BallState.THEIRS, BallState.THEIRS_KICKED):
                self.ball_state = BallState.THEIRS_KICKED
            elif self.ball_state == BallState.FREE:
                self.ball_state = BallState.LOOSE
        else:
            # ボールが止まっている場合は、今の状況で判定する
            if self.ball_holder:
                if self.ball_holder.is_our_team:
                    self.ball_state = BallState.OURS
                else:
                    self.ball_state = BallState.THEIRS
            else:
                self.ball_state = BallState.FREE

    def validate_and_update_ball_holder(self, ball: BallModel, robots: RobotsModel) -> bool:
        """ボール保持者の状態を検証し、更新するメソッド."""
        if not self.ball_holder:
            return False

        # ボール保持者が存在するか
        if self.ball_holder.is_our_team:
            if self.ball_holder.robot.robot_id not in [robot.robot_id for robot in robots.our_visible_robots.values()]:
                return False
        else:
            if self.ball_holder.robot.robot_id not in [
                robot.robot_id for robot in robots.their_visible_robots.values()
            ]:
                return False

        # ボール保持者がボールに近いか
        if self.ball_holder.is_our_team:
            robot = robots.our_robots[self.ball_holder.robot.robot_id]
        else:
            robot = robots.their_robots[self.ball_holder.robot.robot_id]

        distance = tools.get_distance(ball.pos, robot.pos)

        # ヒステリシス性をもたせるために、しきい値を広げる
        if distance > self.HAS_BALL_DISTANCE_THRESHOLD + self.HAS_BALL_MARGIN:
            return False

        # ボール保持者の情報を更新する
        self.ball_holder.robot = robot

        return True

    def search_new_ball_holder(self, ball: BallModel, robots: RobotsModel) -> Optional[BallHolder]:
        """ボールを持っているロボットを探索するメソッド."""
        # 現在のボール保持者の距離を計算
        ball_holder_distance = float("inf")
        if self.ball_holder:
            ball_holder_distance = tools.get_distance(ball.pos, self.ball_holder.robot.pos)

        # 新しいボール保持者候補を探索
        new_ball_holder = self.nearest_robot(ball=ball, robots=robots)
        # 新しい保持者が見つからなければ、現在の保持者を返す
        if not new_ball_holder:
            return self.ball_holder

        new_ball_holder_distance = tools.get_distance(ball.pos, new_ball_holder.robot.pos)

        # ボールから離れていれば、現在の保持者を返す
        if new_ball_holder_distance > self.HAS_BALL_DISTANCE_THRESHOLD:
            return self.ball_holder

        # ヒステリシス性を考慮して、よりボールに近いロボットを新しい候補者にする
        if new_ball_holder_distance < ball_holder_distance - self.HAS_BALL_MARGIN:
            return new_ball_holder

        return self.ball_holder

    def nearest_robot(self, ball: BallModel, robots: RobotsModel) -> Optional[BallHolder]:
        """チームの中で、ボールに最も近いロボットを取得するメソッド."""
        nearest_our_robot, our_distance = self.nearest_robot_of_team(
            ball=ball,
            visible_robots=robots.our_visible_robots,
        )

        nearest_their_robot, their_distance = self.nearest_robot_of_team(
            ball=ball,
            visible_robots=robots.their_visible_robots,
        )

        # ロボットがいない場合
        if not nearest_our_robot and not nearest_their_robot:
            return None

        # 片方のチームにだけロボットが居る場合
        if nearest_our_robot and not nearest_their_robot:
            return BallHolder(is_our_team=True, robot=nearest_our_robot)
        elif nearest_their_robot and not nearest_our_robot:
            return BallHolder(is_our_team=False, robot=nearest_their_robot)

        # 両方のチームにロボットがいる場合
        if our_distance < their_distance:
            return BallHolder(is_our_team=True, robot=nearest_our_robot)
        else:
            return BallHolder(is_our_team=False, robot=nearest_their_robot)

    def nearest_robot_of_team(self, ball: BallModel, visible_robots: dict[int, Robot]) -> tuple[Optional[Robot], float]:
        """チームの中で、ボールに最も近いロボットを取得するメソッド."""
        nearest_robot = None
        nearest_distance = float("inf")

        for robot in visible_robots.values():
            distance = tools.get_distance(ball.pos, robot.pos)

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_robot = robot

        return nearest_robot, nearest_distance

    def prediction_next_ball_pos(self, ball: BallModel):
        """
        次のボールの位置を予測するメソッド

        暫定的に0.1[m]移動すると仮定
        """
        # 将来の位置
        _future_ball_pos = State2D()
        _future_ball_pos.x = ball.pos.x + ball.vel.x
        _future_ball_pos.y = ball.pos.y + ball.vel.y
        # 軌道角度を計算
        self.angle_trajectory = tools.get_angle(ball.pos, _future_ball_pos)

        # ボール移動量
        self.ball_movement.x = ball.vel.x * self.MOVEMENT_GAIN  # * np.cos(self.angle_trajectory)
        self.ball_movement.y = ball.vel.y * self.MOVEMENT_GAIN  # * np.sin(self.angle_trajectory)

        # 予測位置を算出
        self.next_ball_pos.x = ball.pos.x + self.ball_movement.x
        self.next_ball_pos.y = ball.pos.y + self.ball_movement.y

    def is_ball_moving(self, ball: BallModel) -> bool:
        """ボールが動いているかを判定するメソッド."""
        if not ball.is_visible:
            return False

        vel_norm = tools.get_norm(ball.vel)

        if vel_norm < self.MOVING_VELOCITY_THRESHOLD:
            # ボールの速度が小さいときは、ボールが停止したと判断して、位置をキャプチャする
            if self.last_ball_pos_to_detect_moving is None:
                self.last_ball_pos_to_detect_moving = deepcopy(ball.pos)
        else:
            # ボールの速度が大きくて、前回の位置情報を持っていないときは
            # 移動が継続していると判断してTrueを返す
            if self.last_ball_pos_to_detect_moving is None:
                return True

        # 停止時のボール位置からの移動距離
        move_distance = tools.get_distance(ball.pos, self.last_ball_pos_to_detect_moving)

        if move_distance > self.BALL_MOVING_DIST_THRESHOLD:
            # 一定距離以上離れたら、動いたと判定してキャプチャした位置をリセット
            self.last_ball_pos_to_detect_moving = None
            return True

        # 一定距離移動してなければ、ボールは止まっていると判断
        return False

    def update_ball_on_placement_area(self, ball: BallModel, referee: RefereeModel):
        """ボールがプレースメントエリアにあるかを更新するメソッド."""
        ON_AREA_THRESHOLD = 0.15  # Rule 5.2に基づく
        DISTANCE_MARGIN = 0.05  # ヒステリシス処理に使用する

        if not ball.is_visible:
            self.ball_is_on_placement_area = False
            return

        # ボールがプレースメントエリアにない場合は、しきい値を厳しくする
        threshold = ON_AREA_THRESHOLD - DISTANCE_MARGIN

        if self.ball_is_on_placement_area:
            # ボールgあプレースメントエリアにある場合は、しきい値を緩くする
            threshold = ON_AREA_THRESHOLD

        if tools.get_distance(ball.pos, referee.placement_pos) < threshold:
            self.ball_is_on_placement_area = True
        else:
            self.ball_is_on_placement_area = False

    def predict_ball_stop_position(self, ball: BallModel, game_config: GameConfigModel) -> State2D:
        """ボールが止まる位置を予測するメソッド."""
        # ボールの速度が小さい場合は、現在の位置を返す
        if not self.ball_is_moving:
            return ball.pos

        # ボールを中心に、ボール速度方向への座標系を作成
        trans = tools.Trans(ball.pos, tools.get_vel_angle(ball.vel))

        vel_norm = tools.get_norm(ball.vel)

        # 減速距離
        a = game_config.ball_friction_coeff * game_config.gravity
        distance = (vel_norm ** 2) / (2 * a)

        return trans.inverted_transform(State2D(x=distance, y=0.0))

    def is_ball_will_enter_their_goal(self, ball: BallModel, field_points: FieldPoints) -> bool:
        """ボールが相手のゴールに入るかを判定するメソッド."""
        # ボールが動いていない場合は、Falseを返す
        if not self.ball_is_moving:
            return False

        # 2つの線が交差するかで判定する
        return tools.is_intersect(
            p1=ball.pos, p2=self.ball_stop_position, q1=field_points.their_goal_top, q2=field_points.their_goal_bottom
        )

    @property
    def is_our_team_ball_holder(self) -> bool:
        """ボール保持者が自分チームか判定を返す関数"""
        if self.ball_holder is None:
            return False
        else:
            return self.ball_holder.is_our_team

    @property
    def is_their_team_ball_holder(self) -> bool:
        """ボール保持者が相手チームか判定を返す関数"""
        if self.ball_holder is None:
            return False
        else:
            return not self.ball_holder.is_our_team
