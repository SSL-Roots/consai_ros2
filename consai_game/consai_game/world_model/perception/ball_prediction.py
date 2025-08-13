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

"""予測を管理するモジュール."""

from consai_tools.geometry import geometry_tools as tools

from consai_game.world_model.ball_model import BallModel
from consai_game.world_model.game_config_model import GameConfigModel
from consai_game.world_model.field_model import FieldPoints

from consai_msgs.msg import State2D


class BallPrediction:
    """ボールの位置や動きを予測するクラス."""

    # 速度に対するボール移動量を算出する比率[s]: 実質的に移動時間
    MOVEMENT_GAIN = 0.1

    def update(self, ball_is_moving: bool, field_points: FieldPoints, game_config: GameConfigModel):
        """ボールやフィールドなどの状態を更新するメソッド."""

        self.ball_is_moving = ball_is_moving
        self.field_points = field_points
        self.game_config = game_config

    def next_ball_pos(self, ball: BallModel):
        """
        次のボールの位置を予測するメソッド

        暫定的に0.1[m]移動すると仮定
        """

        # TODO: 本来であれば角度から移動量を求めるべきだが暫定的に定量動くと予測している
        # 将来の位置
        # _future_ball_pos = State2D()
        # _future_ball_pos.x = ball.pos.x + ball.vel.x
        # _future_ball_pos.y = ball.pos.y + ball.vel.y
        # # 軌道角度を計算
        # angle_trajectory = tools.get_angle(ball.pos, _future_ball_pos)

        # 予測位置を算出
        next_ball_pos = State2D()
        next_ball_pos.x = ball.pos.x + ball.vel.x * self.MOVEMENT_GAIN  # * np.cos(self.angle_trajectory)
        next_ball_pos.y = ball.pos.y + ball.vel.y * self.MOVEMENT_GAIN  # * np.sin(self.angle_trajectory)

        return next_ball_pos

    def ball_stop_position(self, ball: BallModel) -> State2D:
        """ボールが止まる位置を予測するメソッド."""

        # ボールの速度が小さい場合は、現在の位置を返す
        if not self.ball_is_moving:
            return ball.pos

        # ボールを中心に、ボール速度方向への座標系を作成
        trans = tools.Trans(ball.pos, tools.get_vel_angle(ball.vel))

        vel_norm = tools.get_norm(ball.vel)

        # 減速距離
        a = self.game_config.ball_friction_coeff * self.game_config.gravity
        distance = (vel_norm ** 2) / (2 * a)

        return trans.inverted_transform(State2D(x=distance, y=0.0))

    def is_ball_will_enter_their_goal(self, ball: BallModel) -> bool:
        """ボールが相手のゴールに入るかを判定(予測)するメソッド."""

        # ボールが動いていない場合は、Falseを返す
        if not self.ball_is_moving:
            return False

        # 2つの線が交差するかで判定する
        return tools.is_intersect(
            p1=ball.pos,
            p2=self.ball_stop_position(ball),
            q1=self.field_points.their_goal_top,
            q2=self.field_points.their_goal_bottom,
        )
