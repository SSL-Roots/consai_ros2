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


from consai_game.world_model.ball_model import BallModel
from consai_game.world_model.robots_model import RobotsModel, Robot
from consai_game.world_model.robot_activity_model import RobotActivityModel
from consai_tools.geometry import geometry_tools as tools
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional


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
    robot: Robot = field(default_factory=Robot)


class BallActivityModel:
    """ボールの活動状態を保持するクラス."""

    def __init__(self):
        self.ball_state = BallState.FREE
        self.ball_holder: Optional[BallHolder] = None

    def update(self, ball: BallModel, robots: RobotsModel, robot_activity: RobotActivityModel):
        """ボールの状態を更新するメソッド."""
        # ボール保持者を探索する
        self.ball_holder = self.search_ball_holder(
            ball=ball,
            robots=robots,
            robot_activity=robot_activity,
        )

        if self.ball_holder:
            if self.ball_holder.is_our_team:
                self.ball_state = BallState.OURS
            else:
                self.ball_state = BallState.THEIRS
        else:
            self.ball_state = BallState.FREE

    def search_ball_holder(
        self, ball: BallModel, robots: RobotsModel, robot_activity: RobotActivityModel
    ) -> Optional[BallHolder]:
        """ボールを持っているロボットを探索するメソッド."""
        # DISTANCE_THRESHOLD = 0.2  # ボールとロボットの距離の閾値

        ball_holder = None

        nearest_our_robot, our_distance = self.nearest_robot(
            ball=ball,
            robots=robots.our_robots,
            visible_ids=robot_activity.ordered_our_visible_robots,
        )
        nearest_their_robot, their_distance = self.nearest_robot(
            ball=ball,
            robots=robots.their_robots,
            visible_ids=robot_activity.ordered_their_visible_robots,
        )

        # present_holder_distance = float("inf")
        # if self.ball_holder:
        #     present_holder_distance = tools.get_distance(ball.pos, self.ball_holder.robot.pos)

        # ロボットがいない場合
        if not nearest_our_robot and not nearest_their_robot:
            return None

        return ball_holder

    def nearest_robot(
        self, ball: BallModel, robots: dict[int, Robot], visible_ids: list[id]
    ) -> tuple[Optional[Robot], float]:
        """ボールに最も近いロボットを取得するメソッド."""
        nearest_robot = None
        nearest_distance = float("inf")

        for robot_id in visible_ids:
            robot = robots[robot_id]
            distance = tools.get_distance(ball.pos, robot.pos)

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_robot = robot

        return nearest_robot, nearest_distance

    def is_same_robot(self, robot_a: Robot, robot_b: Robot) -> bool:
        """ロボットが同じかどうかを判定するメソッド."""
        is_same_id = robot_a.robot_id == robot_b.robot_id
        is_same_team = robot_a.is_yellow == robot_b.is_yellow
        return is_same_id and is_same_team
