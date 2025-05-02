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
ゴーリーの動作をまとめたTactic
"""

from consai_game.core.tactic.tactic_base import TacticBase
from consai_game.tactic.ball_clear import BallClear
from consai_game.tactic.defend_goal import DefendGoal
from consai_game.tactic.goalie_positioning import GoaliePositioning
from consai_game.world_model.world_model import WorldModel

from consai_msgs.msg import MotionCommand


class CompositeGoalie(TacticBase):
    def __init__(self):
        super().__init__()
        self.positioning = GoaliePositioning()
        self.defend_goal = DefendGoal()
        self.ball_clear = BallClear()

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        super().reset(robot_id)
        # 所有するTacticも初期化する
        self.positioning.reset(robot_id)
        self.defend_goal.reset(robot_id)
        self.ball_clear.reset(robot_id)

    def run(self, world_model: WorldModel) -> MotionCommand:
        """状況に応じて実行するtacticを切り替える関数"""

        if world_model.ball_position.is_in_our_side() and world_model.ball_activity.ball_is_moving:
            # ボールが自分再度にありボールが動いている場合はゴールを守る
            return self.defend_goal.run(world_model)
        elif world_model.ball_position.is_in_our_defense_area():
            # ボールがディフェンスエリアにある場合はボールクリア
            return self.ball_clear.run(world_model)
        else:
            # デフォルトはゴーリーのポジショニングを実行
            return self.positioning.run(world_model)
