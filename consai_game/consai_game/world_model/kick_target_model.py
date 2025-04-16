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
import math


class KickTargetModel:
    """キックターゲットを保持するクラス."""

    def __init__(self, field: Field, field_points: FieldPoints):
        self.hysteresis_distance = 0.3
        self.robot_radius = 0.09
        # 自チームのゴーリーIDどっかでわからんかったっけ？
        self._goalie_id = 0

        self._field = field
        self._field_points = field_points

        self.half_length = self._field.length / 2
        self.half_width = self._field.width / 2
        self.goal_width = self._field.goal_width
        self.target_goal = self.goal_width / 4

        self._set_goal_pos_list()
        self._set_clear_pos_list()

        self._last_shoot_pos_list: list = None
        self._last_clear_pos_list: list = None
        self._last_pass_id_list: list = None
        self._present_shoot_pos_list = []
        self._present_clear_pos_list = []
        self._present_receiver_id_list = []

    def _set_goal_pos_list(self) -> None:
        self._goal_pos_list = [
            Point(self.half_length, 0.0),
            Point(self.half_length, self.target_goal),
            Point(self.half_length, -self.target_goal),
        ]

    def _set_clear_pos_list(self) -> None:
        self._clear_pos_list = [
            Point(self.half_length, self.half_width),
            Point(self.half_length, -self.half_width),
        ]

    def get_shoot_pos_list(self) -> list[State2D]:
        return self._present_shoot_pos_list

    def get_clear_pos_list(self) -> list[State2D]:
        return self._present_clear_pos_list

    def get_receiver_id_list(self) -> list[int]:
        return self._present_receiver_id_list

    def update(
        self,
        my_robot_id: int,
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
        self._present_receiver_id_list = self._search_receivers_list(my_robot_id, search_offset=self.robot_radius)

    def _search_forward_robots(
        self, pos: State2D, search_offsset=0.0, search_our_robots=True, exclude_id=-1, our_goalie_id=-1
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
                if pos.x < robots.get(robot_id).pos.x + search_offsset:
                    forward_robots_id.append(robot_id)

            return forward_robots_id

        if search_our_robots:
            return search(pos, self._our_robots, self._our_visible_robots, search_offsset)
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
        if self._last_shoot_pos_list is not None:
            last_shoot_pos = self._last_shoot_pos_list[0]
            still_valid = not obstacle_exists(last_shoot_pos, self._their_robots)
            if still_valid:
                shoot_pos_list.sort(key=lambda p: tool.get_distance(p, last_shoot_pos))
                if shoot_pos_list and tool.get_distance(shoot_pos_list[0], last_shoot_pos) < self.hysteresis_distance:
                    shoot_pos_list = [last_shoot_pos] + [p for p in shoot_pos_list if p != last_shoot_pos]

        self._last_shoot_pos_list = shoot_pos_list.copy() if shoot_pos_list else None
        return shoot_pos_list

    def _search_clear_pos_list(self, search_ours=False) -> list[State2D]:
        # ボールからの直線上にロボットがいないシュート位置リストを返す
        TOLERANCE = self.robot_radius  # ロボット半径

        clear_pos_list = []

        def obstacle_exists(target: State2D, robots: dict[int, State2D]) -> bool:
            # ロボットがシュートポジションを妨害しているか判定
            for robot in robots.values():
                if tool.is_on_line(robot.pos, self._ball.pos, target, TOLERANCE):
                    return True
            return False

        for target in self._clear_pos_list:
            if obstacle_exists(target, self._our_robots) and search_ours:
                continue
            if obstacle_exists(target, self._their_robots):
                continue
            clear_pos_list.append(target)

        # ヒステリシス処理
        if self._last_clear_pos_list is not None:
            last_clear_pos = self._last_clear_pos_list[0]
            still_valid = not obstacle_exists(last_clear_pos, self._their_robots)
            if still_valid:
                clear_pos_list.sort(key=lambda p: tool.get_distance(p, last_clear_pos))
                if clear_pos_list and tool.get_distance(clear_pos_list[0], last_clear_pos) < self.hysteresis_distance:
                    clear_pos_list = [last_clear_pos] + [p for p in clear_pos_list if p != last_clear_pos]

        self._last_clear_pos_list = clear_pos_list.copy() if clear_pos_list else None
        return clear_pos_list

    def _search_receivers_list(self, my_robot_id: int, search_offset=0.0) -> list[int]:
        # パス可能なロボットIDのリストを返す関数

        # 計算上の相手ロボットの半径（通常の倍の半径（直径）に設定）
        robot_r = 0.4
        # ロボットの位置座標取得から実際にパスを出すまでの想定時間
        dt = 0.5

        # パサーがフィールドにいない場合
        if my_robot_id not in self._our_robots.keys():
            return []

        my_robot_pos = self._our_robots[my_robot_id].pos

        # パサーよりも前にいる味方ロボットIDのリストを取得
        forward_our_id_list = self._search_forward_robots(
            my_robot_pos, search_offset, search_our_robots=True, exclude_id=my_robot_id, our_goalie_id=self._goalie_id
        )
        forward_their_id_list = self._search_forward_robots(my_robot_pos, search_our_robots=False)

        # パサーよりも前にいるロボットがいなければ空のリストを返す
        if len(forward_our_id_list) == 0:
            return []

        # パス可能なロボットIDを格納するリスト
        receiver_id_list: list[int] = []

        # 各レシーバー候補ロボットに対してパス可能か判定
        for friend_id in forward_our_id_list:
            friend_pos = self._our_robots[friend_id].pos
            trans_me_to_friend = tool.Trans(my_robot_pos, tool.get_angle(my_robot_pos, friend_pos))

            # パサーとレシーバー候補ロボットの間にいる相手ロボットを計算対象とするときの処理
            obstacle_their_id_list = []
            for their_robot_id in forward_their_id_list:
                their_robot_pos = self._their_robots[their_robot_id].pos
                if their_robot_pos.x < friend_pos.x:
                    obstacle_their_id_list.append(their_robot_id)

            if len(obstacle_their_id_list) == 0:
                receiver_id_list.append(friend_id)
                continue

            # 相手ロボットが存在するときの処理
            # 対象となる相手ロボット全てに対してパスコースを妨げるような動きをしているか計算
            check_count = 0
            for obstacle_robot_id in obstacle_their_id_list:
                obstacle_pos = self._their_robots[obstacle_robot_id].pos
                obstacle_vel = self._their_robots[obstacle_robot_id].vel
                obstacle_vel_norm = math.hypot(obstacle_vel.x, obstacle_vel.y)
                obstacle_pos_MtoF = trans_me_to_friend.transform(obstacle_pos)

                # 共有点を持つか判定
                common_point = -1
                if abs(obstacle_pos_MtoF.y) < robot_r + obstacle_vel_norm * dt:
                    common_point = 1

                # 共有点を持たないときの処理
                if common_point < 0:
                    # 対象としている相手ロボットすべてにパスコースが妨害されないときの処理
                    if check_count >= len(obstacle_their_id_list) - 1:
                        # 何台の相手ロボットに妨害されないかをカウントする変数をリセット
                        check_count = 0
                        # パスできる味方ロボットとしてリストに格納
                        receiver_id_list.append(friend_id)
                    # まだすべてのロボットに対して計算を行っていない場合の処理
                    else:
                        # 何台の相手ロボットに妨害されないかをカウントする変数をインクリメント
                        check_count += 1
                # 共有点を持つときの処理
                else:
                    # 何台の相手ロボットに妨害されないかをカウントする変数をリセット
                    check_count = 0
                    break

        # ヒステリシス処理
        if self._last_pass_id_list is not None:
            still_valid = len(receiver_id_list) > 0
            if still_valid:
                receiver_id_list.sort(key=lambda id: self._last_pass_id_list.index(id))
                if receiver_id_list and receiver_id_list[0] == self._last_pass_id_list[0]:
                    receiver_id_list = [self._last_pass_id_list[0]] + [
                        id for id in receiver_id_list if id != self._last_pass_id_list[0]
                    ]
        # 受け手ロボットのIDを更新
        self._last_pass_id_list = receiver_id_list.copy()

        return receiver_id_list
