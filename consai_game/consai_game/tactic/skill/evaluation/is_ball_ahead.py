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


from consai_msgs.msg import State2D
from consai_tools.geometry import geometry_tools as tool


def is_ball_ahead(ball_pos: State2D, robot_pos: State2D, target_pos: State2D) -> bool:
    """ボールがロボットの前にあるかどうかを判定する."""
    FRONT_DIST_THRESHOLD = 0.15  # 正面方向にどれだけ離れることを許容するか
    SIDE_DIST_THRESHOLD = 0.05  # 横方向にどれだけ離れることを許容するか

    # ロボットを中心に、ターゲットを+x軸とした座標系を作る
    trans = tool.Trans(robot_pos, tool.get_angle(robot_pos, target_pos))
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
