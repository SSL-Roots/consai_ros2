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
from consai_game.world_model.field_model import Field
from consai_game.world_model.ball_model import BallModel
from consai_game.world_model.ball_activity_model import BallActivityModel
from consai_game.world_model.field_model import FieldPoints
from consai_msgs.msg import State2D
from consai_tools.geometry import geometry_tools as tools
from consai_msgs.msg import MotionCommand


class CompositeGoalie(TacticBase):
    """ゴーリーの動作をまとめたTactic"""

    # ロボットの半径[m]
    ROBOT_RADIUS = 0.1

    def __init__(self):
        """Initialize the DefendGoal tactic."""
        super().__init__()
        self.positioning = GoaliePositioning()
        self.defend_goal = DefendGoal()
        self.ball_clear = BallClear()
        self.goal_with_margin = 0.5

    def reset(self, robot_id: int) -> None:
        """Reset the tactic state for the specified robot."""
        super().reset(robot_id)
        # 所有するTacticも初期化する
        self.positioning.reset(robot_id)
        self.defend_goal.reset(robot_id)
        self.ball_clear.reset(robot_id)

    def run(self, world_model: WorldModel) -> MotionCommand:
        """状況に応じて実行するtacticを切り替える関数"""

        def avoid_goal(command):
            """ロボットがゴールラインより後ろにいる場合に回避するための位置を生成"""
            robot_pos = world_model.robots.our_robots.get(self.robot_id).pos

            if robot_pos.x < -world_model.field.half_length + self.ROBOT_RADIUS:
                command.desired_pose.x = -world_model.field.half_length + self.ROBOT_RADIUS
                y = robot_pos.y
                if y < -world_model.field.half_goal_width:
                    command.desired_pose.y = -(world_model.field.half_goal_width + self.ROBOT_RADIUS)
                elif world_model.field.half_goal_width < y:
                    command.desired_pose.y = world_model.field.half_goal_width + self.ROBOT_RADIUS
            return command

        field = world_model.field
        field_points = world_model.field_points
        ball = world_model.ball
        ball_activity = world_model.ball_activity

        if self._is_likely_to_score(field, field_points, ball, ball_activity):
            # ボールがゴールに入りそうならブロック
            return avoid_goal(self.defend_goal.run(world_model))
        elif (
            world_model.ball_position.is_in_our_defense_area()
            and not world_model.ball_position.is_outside_with_margin()
        ):
            # ボールがディフェンスエリアにある場合はボールクリア
            return avoid_goal(self.ball_clear.run(world_model))
        else:
            # ゴーリーのポジショニングを実行
            return avoid_goal(self.positioning.run(world_model))

    def _is_likely_to_score(
        self, field: Field, field_points: FieldPoints, ball: BallModel, ball_activity: BallActivityModel
    ) -> bool:
        """ボールがゴールに入りそうかどうかを判定する"""
        goal_y_top = field.half_goal_width
        goal_y_bottom = -field.half_goal_width
        ball_pos = ball.pos
        ball_stop_position = ball_activity.ball_stop_position

        # ゴールの上端・下端の座標
        # マージンを足して少し広く取る
        goal_top_with_margin = State2D(x=-field.half_length, y=goal_y_top + self.goal_with_margin)
        goal_bottom_with_margin = State2D(x=-field.half_length, y=goal_y_bottom - self.goal_with_margin)

        # ボール進行方向がゴールに交差するかどうかを判定する
        intersection = tools.get_line_intersection(
            ball_pos, ball_stop_position, goal_top_with_margin, goal_bottom_with_margin
        )

        # ボールがゴールに到達するか
        # 到着点がディフェンスエリア側にありそうかどうか
        if intersection is not None and ball_stop_position.x < field_points.our_defense_area.top_right.x:
            return True
        else:
            return False
