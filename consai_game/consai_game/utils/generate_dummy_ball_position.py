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
ボールが消えたことを想定して、仮想的なボール位置を生成するモジュール
"""

from consai_game.world_model.ball_model import BallModel
from consai_msgs.msg import State2D
from consai_tools.geometry import geometry_tools as tool


def generate_dummy_ball_position(ball: BallModel, robot_pos: State2D) -> State2D:
    """ボールが消えていた場合、ロボットがボールを持っている前提の位置を返す"""

    # ボールが見えていたらそのまま返す
    if ball.is_visible:
        return ball.pos

    dist_robot_to_ball = 0.10

    # ロボットを中心に、ロボットの向きをX+軸とする座標系を作る
    trans = tool.Trans(robot_pos, robot_pos.theta)

    # ロボットの前方、ドリブラー付近にボールがあると仮定する
    return trans.inverted_transform(State2D(x=dist_robot_to_ball, y=0.0))
