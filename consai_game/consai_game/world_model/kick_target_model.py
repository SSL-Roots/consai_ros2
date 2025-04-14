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

from consai_game.world_model.field_model import Field, FieldPoints
from consai_game.world_model.robots_model import Robot, RobotsModel
from consai_game.world_model.ball_model import BallModel
from consai_game.world_model.robot_activity_model import RobotActivityModel
from consai_tools.geometry import geometry_tools as tool
from consai_msgs.msg import State2D
from consai_game.utils.geometry import Point


class KickTargetModel:
    """キックターゲットを保持するクラス."""

    def __init__(self, goalie_id: int):
        self.hysteresis_distance = 0.05
        self.robot_radius = 0.18
        self._goalie_id = goalie_id

        self._field = self.field_model = Field()
        self._field_points = self.field_points = FieldPoints()

        self._set_goal_pos_list()

        self._present_shoot_pos_list = []
        self._present_clear_pos_list = []

    def _set_goal_pos_list(self) -> None:
        self.half_length = self._field.length / 2
        self.goal_width = self._field.goal_width
        self.half_goal_width = self.goal_width / 2

        self._goal_pos_list = [
            Point(self.half_length, self.half_goal_width),
            Point(self.half_goal_width, 0.0),
            Point(self.half_length, -self.half_goal_width),
        ]

    def get_shoot_pos_list(self) -> list[State2D]:
        return self._present_shoot_pos_list

    def get_clear_pos_list(self) -> list[State2D]:
        return self._present_clear_pos_list

    def update(
        self,
        ball_model: BallModel,
        robots_model: RobotsModel,
        robot_activity_model: RobotActivityModel,
    ) -> None:
        """キックターゲットを更新する."""
        self._ball = ball_model
        self._our_robots = robots_model.our_robots
        self._their_robots = robots_model.their_robots
        self._our_visible_robots = robot_activity_model.our_visible_robots
        self._their_visible_robots = robot_activity_model.their_visible_robots

        self._present_shoot_pos_list = self._search_shoot_pos_list()
        self._present_clear_pos_list = self._search_clear_pos_list()

    def _search_forward_robots(
        self,
        pos: State2D,
        search_offsset=0.0,
        search_our_robots=True,
        exclude_id=-1,
        our_goalie_id=-1,
    ) -> list[int]:
        # 指定した座標より前にいるロボットIDのリストを返す関数

        def search(
            pos: State2D,
            robots: dict[int, State2D],
            activity_robot_id: list[int],
            search_offsset=0.0,
        ) -> list[int]:
            # 指定した座標より前にいるロボットIDのリストを返す関数
            forward_robots_id = []
            for robot_id in activity_robot_id:
                if robot_id == exclude_id or robot_id == our_goalie_id:
                    continue
                if pos.x < robots.get(robot_id).x + search_offsset:
                    forward_robots_id.append(robot_id)

            return forward_robots_id

        if search_our_robots:
            return search(
                pos, self._our_robots, self._our_visible_robots, search_offsset
            )
        else:
            return search(pos, self._their_robots, self._their_visible_robots)

    def _search_shoot_pos_list(self, search_ours=False) -> list[State2D]:
        # ボールからの直線上にロボットがいないシュート位置リストを返す
        TOLERANCE = self.robot_radius  # ロボット半径

        shoot_pos_list = []

        def obstacle_exists(target: State2D, robots: dict[int, Robot]) -> bool:
            for robot in robots.values():
                if tool.is_on_line(robot.pos, self._ball.pos, target, TOLERANCE):
                    return True
            return False

        for target in self._goal_pos_list:
            if obstacle_exists(target, self._our_robots) and search_ours:
                continue
            if obstacle_exists(target, self._their_robots):
                continue
            shoot_pos_list.append(target)

        # ヒステリシス処理
        if self._last_shoot_pos is not None:
            still_valid = not obstacle_exists(self._last_shoot_pos, self._their_robots)
            if still_valid:
                shoot_pos_list.sort(
                    key=lambda p: tool.get_distance(p, self._last_shoot_pos)
                )
                if (
                    shoot_pos_list
                    and tool.get_distance(shoot_pos_list[0], self._last_shoot_pos)
                    < self.hysteresis_distance
                ):
                    shoot_pos_list = [self._last_shoot_pos] + [
                        p for p in shoot_pos_list if p != self._last_shoot_pos
                    ]

        self._last_shoot_pos = shoot_pos_list[0] if shoot_pos_list else None
        return shoot_pos_list

    def _search_clear_pos_list(self, search_ours=False) -> list[State2D]:
        # ボールからの直線上にロボットがいないシュート位置リストを返す
        TOLERANCE = self.robot_radius  # ロボット半径

        clear_pos_list = []

        def obstacle_exists(target: State2D, robots: dict[int, State2D]) -> bool:
            # ロボットがシュートポジションを妨害しているか判定
            for robot in robots.values():
                if tool.is_on_line(robot.pos(), self._ball.pos(), target, TOLERANCE):
                    return True
            return False

        for target in self._field_points.corners:
            if obstacle_exists(target, self._our_robots) and search_ours:
                continue
            if obstacle_exists(target, self._their_robots):
                continue
            clear_pos_list.append(target)

        # ヒステリシス処理
        if self._last_clear_pos is not None:
            still_valid = not obstacle_exists(self._last_clear_pos, self._their_robots)
            if still_valid:
                clear_pos_list.sort(
                    key=lambda p: tool.get_distance(p, self._last_clear_pos)
                )
                if (
                    clear_pos_list
                    and tool.get_distance(clear_pos_list[0], self._last_clear_pos)
                    < self.hysteresis_distance
                ):
                    clear_pos_list = [self._last_clear_pos] + [
                        p for p in clear_pos_list if p != self._last_clear_pos
                    ]

        self._last_clear_pos = clear_pos_list[0] if clear_pos_list else None
        return clear_pos_list
