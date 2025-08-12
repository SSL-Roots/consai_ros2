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


# ball_activity_model.py
def prediction_next_ball_pos(ball: BallModel):
    """
    次のボールの位置を予測するメソッド

    暫定的に0.1[m]移動すると仮定
    """

    # 速度に対するボール移動量を算出する比率[s]: 実質的に移動時間
    MOVEMENT_GAIN = 0.1
    # ボールの移動量
    ball_movement = State2D()
    # ボールの将来の予測位置
    next_ball_pos = State2D()
    # 将来の位置
    _future_ball_pos = State2D()
    _future_ball_pos.x = ball.pos.x + ball.vel.x
    _future_ball_pos.y = ball.pos.y + ball.vel.y
    # ボール移動量
    ball_movement.x = ball.vel.x * MOVEMENT_GAIN  # * np.cos(self.angle_trajectory)
    ball_movement.y = ball.vel.y * MOVEMENT_GAIN  # * np.sin(self.angle_trajectory)

    # 予測位置を算出
    next_ball_pos.x = ball.pos.x + ball_movement.x
    next_ball_pos.y = ball.pos.y + ball_movement.y

def predict_ball_stop_position(ball: BallModel, game_config: GameConfigModel) -> State2D:
    """ボールが止まる位置を予測するメソッド."""

    ball_is_moving = False

    # ボールの速度が小さい場合は、現在の位置を返す
    if not ball_is_moving:
        return ball.pos

    # ボールを中心に、ボール速度方向への座標系を作成
    trans = tools.Trans(ball.pos, tools.get_vel_angle(ball.vel))

    vel_norm = tools.get_norm(ball.vel)

    # 減速距離
    a = game_config.ball_friction_coeff * game_config.gravity
    distance = (vel_norm ** 2) / (2 * a)

    return trans.inverted_transform(State2D(x=distance, y=0.0))

def is_ball_will_enter_their_goal(ball: BallModel, field_points: FieldPoints) -> bool:
    """ボールが相手のゴールに入るかを判定するメソッド."""

    ball_is_moving = False

    # ボールが最終的に止まる予測位置
    ball_stop_position = State2D()
    # ボールが動いていない場合は、Falseを返す
    if not ball_is_moving:
        return False

    # 2つの線が交差するかで判定する
    return tools.is_intersect(
        p1=ball.pos, p2=ball_stop_position, q1=field_points.their_goal_top, q2=field_points.their_goal_bottom
    )