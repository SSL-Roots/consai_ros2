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
from dataclasses import dataclass
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
    robot: Robot


class BallActivityModel:
    """ボールの活動状態を保持するクラス."""

    HAS_BALL_DISTANCE_THRESHOLD = 0.2  # ボールとロボットの距離の閾値
    HAS_BALL_MARGIN = 0.1  # ヒステリシス処理に使用する

    def __init__(self):
        self.ball_state = BallState.FREE
        self.ball_holder: Optional[BallHolder] = None

    def update(self, ball: BallModel, robots: RobotsModel, robot_activity: RobotActivityModel):
        """ボールの状態を更新するメソッド."""
        # ボール保持者が有効か確認する
        if not self.validate_and_update_ball_holder(ball, robots, robot_activity):
            self.ball_holder = None

        # ボール保持者を探索する
        self.ball_holder = self.search_new_ball_holder(
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

    def validate_and_update_ball_holder(
        self, ball: BallModel, robots: RobotsModel, robot_activity: RobotActivityModel
    ) -> bool:
        """ボール保持者の状態を検証し、更新するメソッド."""
        if not self.ball_holder:
            return False

        # ボール保持者が存在するか
        if self.ball_holder.is_our_team:
            if self.ball_holder.robot.robot_id not in robot_activity.ordered_our_visible_robots:
                return False
        else:
            if self.ball_holder.robot.robot_id not in robot_activity.ordered_their_visible_robots:
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

    def search_new_ball_holder(
        self, ball: BallModel, robots: RobotsModel, robot_activity: RobotActivityModel
    ) -> Optional[BallHolder]:
        """ボールを持っているロボットを探索するメソッド."""
        # 現在のボール保持者の距離を計算
        ball_holder_distance = float("inf")
        if self.ball_holder:
            ball_holder_distance = tools.get_distance(ball.pos, self.ball_holder.robot.pos)

        # 新しいボール保持者候補を探索
        new_ball_holder = self.nearest_robot(
            ball=ball,
            robots=robots,
            robot_activity=robot_activity,
        )
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

    def nearest_robot(
        self, ball: BallModel, robots: RobotsModel, robot_activity: RobotActivityModel
    ) -> Optional[BallHolder]:
        """チームの中で、ボールに最も近いロボットを取得するメソッド."""
        nearest_our_robot, our_distance = self.nearest_robot_of_team(
            ball=ball,
            robots=robots.our_robots,
            visible_ids=robot_activity.ordered_our_visible_robots,
        )

        nearest_their_robot, their_distance = self.nearest_robot_of_team(
            ball=ball,
            robots=robots.their_robots,
            visible_ids=robot_activity.ordered_their_visible_robots,
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

    def nearest_robot_of_team(
        self, ball: BallModel, robots: dict[int, Robot], visible_ids: list[id]
    ) -> tuple[Optional[Robot], float]:
        """チームの中で、ボールに最も近いロボットを取得するメソッド."""
        nearest_robot = None
        nearest_distance = float("inf")

        for robot_id in visible_ids:
            robot = robots[robot_id]
            distance = tools.get_distance(ball.pos, robot.pos)

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_robot = robot

        return nearest_robot, nearest_distance
