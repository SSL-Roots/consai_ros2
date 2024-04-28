# Copyright 2024 Roots
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

from consai_examples.observer.pos_vel import PosVel
from consai_msgs.msg import State2D
from consai_tools.geometry import geometry_tools as tool
import math


class PassShootObserver:
    def __init__(self, goalie_id):
        self._our_robots: dict[int, PosVel] = {}
        self._their_robots: dict[int, PosVel] = {}

        self._field_half_length = 6.0
        self._field_half_width = 4.5
        self._goal_pos_list = [
            State2D(x=self._field_half_length, y=0.0),
            State2D(x=self._field_half_length, y=0.45),
            State2D(x=self._field_half_length, y=-0.45)
        ]
        self._their_top_corner = State2D(
            x=self._field_half_length, y=self._field_half_width)
        self._their_bottom_corner = State2D(
            x=self._field_half_length, y=-self._field_half_width)
        self._our_top_corner = State2D(
            x=-self._field_half_length, y=self._field_half_width)
        self._our_bottom_corner = State2D(
            x=-self._field_half_length, y=-self._field_half_width)
        self._clear_pos_list = [
            self._their_top_corner,
            self._their_bottom_corner,
            self._our_top_corner,
            self._our_bottom_corner]

        self._present_shoot_pos_list: list[State2D] = []

        self._goalie_id = goalie_id

    def update(self, ball: PosVel,
               our_robots: dict[int, PosVel], their_robots: dict[int, PosVel]) -> None:
        self._ball = ball
        self._our_robots = our_robots
        self._their_robots = their_robots

        self._present_shoot_pos_list = self._search_shoot_pos_list()
        self._present_clear_pos_list = self._search_clear_pos_list()

    def get_shoot_pos_list(self) -> list[State2D]:
        return self._present_shoot_pos_list

    def get_clear_pos_list(self) -> list[State2D]:
        return self._present_clear_pos_list

    def search_receivers_list(self, my_robot_id: int, search_offset=0.0) -> list[int]:
        # パス可能なロボットIDのリストを返す関数
        # TODO(Roots): パスできるリストの探索と、シュートできるリストの探索は別関数に分けたほうが良い

        # 計算上の相手ロボットの半径（通常の倍の半径（直径）に設定）
        robot_r = 0.4
        # ロボットの位置座標取得から実際にパスを出すまでの想定時間
        dt = 0.5

        # パサーがフィールドにいない場合
        if my_robot_id not in self._our_robots.keys():
            return []

        my_robot_pos = self._our_robots[my_robot_id].pos()

        # パサーよりも前にいる味方ロボットIDのリストを取得
        forward_our_id_list = self._search_forward_robots(
            my_robot_pos, search_offset, search_our_robots=True,
            exclude_id=my_robot_id, our_goalie_id=self._goalie_id)
        forward_their_id_list = self._search_forward_robots(
            my_robot_pos, search_our_robots=False)

        # パサーよりも前にいるロボットがいなければ空のリストを返す
        if len(forward_our_id_list) == 0:
            return []

        # 自分と各ロボットまでの距離を基にロボットIDをソート
        # QUESTION: ソートは必要？
        # our_robot_id_list = self._sort_by_from_robot_distance(
        #     my_robot_pos, forward_our_robot_id, our_robots_pos)

        # パス可能なロボットIDを格納するリスト
        receiver_id_list: list[int] = []

        # 各レシーバー候補ロボットに対してパス可能か判定
        for friend_id in forward_our_id_list:
            friend_pos = self._our_robots[friend_id].pos()
            trans_me_to_friend = tool.Trans(
                my_robot_pos, tool.get_angle(my_robot_pos, friend_pos))

            # パサーとレシーバー候補ロボットの間にいる相手ロボットを計算対象とするときの処理
            obstacle_their_id_list = []
            for their_robot_id in forward_their_id_list:
                their_robot_pos = self._their_robots[their_robot_id].pos()
                if their_robot_pos.x < friend_pos.x:
                    obstacle_their_id_list.append(their_robot_id)

            if len(obstacle_their_id_list) == 0:
                receiver_id_list.append(friend_id)
                continue

            # 相手ロボットが存在するときの処理
            # 対象となる相手ロボット全てに対してパスコースを妨げるような動きをしているか計算
            check_count = 0
            for obstacle_robot_id in obstacle_their_id_list:
                obstacle_pos = self._their_robots[obstacle_robot_id].pos()
                obstacle_vel = self._their_robots[obstacle_robot_id].vel()
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
        return receiver_id_list

    # def _sort_by_from_robot_distance(self, my_robot_pos, robots_id, robots_pos):
    #     # 自ロボットから各ロボットまでの距離を計算し近い順にソートする関数

    #     # 自ロボットから各ロボットまでの距離、IDをリストに格納
    #     robots_dist_and_id = [
    #         [tool.get_distance(robots_pos[_id], my_robot_pos), _id] for _id in robots_id]

    #     # 距離が近い順にロボットIDをソート
    #     sorted_robots_id = [_id for _, _id in sorted(robots_dist_and_id)]

    #     return sorted_robots_id

    def _search_forward_robots(
            self, pos: State2D, search_offsset=0.0, search_our_robots=True,
            exclude_id=-1, our_goalie_id=-1) -> list[int]:
        # 指定した座標より前にいるロボットIDのリストを返す関数

        def search(pos: State2D, robots: dict[int, PosVel], search_offsset=0.0) -> list[int]:
            # 指定した座標より前にいるロボットIDのリストを返す関数
            forward_robots_id = []
            for robot_id, robot in robots.items():
                if robot_id == exclude_id or robot_id == our_goalie_id:
                    continue
                if pos.x < robot.pos().x + search_offsset:
                    forward_robots_id.append(robot_id)

            return forward_robots_id

        if search_our_robots:
            return search(pos, self._our_robots, search_offsset)
        else:
            return search(pos, self._their_robots)

    def _search_shoot_pos_list(self, search_ours=False) -> list[State2D]:
        # ボールからの直線上にロボットがいないシュート位置リストを返す
        TOLERANCE = 0.1  # ロボット半径 + alpha

        shoot_pos_list = []

        def obstacle_exists(target: State2D, robots: dict[int, PosVel]) -> bool:
            # ロボットがシュートポジションを妨害しているか判定
            for robot in robots.values():
                if tool.is_on_line(robot.pos(), self._ball.pos(), target, TOLERANCE):
                    return True
            return False

        for target in self._goal_pos_list:
            if obstacle_exists(target, self._our_robots) and search_ours:
                continue
            if obstacle_exists(target, self._their_robots):
                continue
            shoot_pos_list.append(target)

        return shoot_pos_list

    def _search_clear_pos_list(self, search_ours=False) -> list[State2D]:
        # ボールからの直線上にロボットがいないシュート位置リストを返す
        TOLERANCE = 0.1  # ロボット半径 + alpha

        clear_pos_list = []

        def obstacle_exists(target: State2D, robots: dict[int, PosVel]) -> bool:
            # ロボットがシュートポジションを妨害しているか判定
            for robot in robots.values():
                if tool.is_on_line(robot.pos(), self._ball.pos(), target, TOLERANCE):
                    return True
            return False

        for target in self._clear_pos_list:
            if obstacle_exists(target, self._our_robots) and search_ours:
                continue
            if obstacle_exists(target, self._their_robots):
                continue
            clear_pos_list.append(target)

        return clear_pos_list

    # def _forward_robots_id(
    #         self, my_robot_id, my_robot_pos, robots_id, robots_pos,
    #         robots_vel, robot_r, dt, skip_my_id=False):

    #     # パサーより前に存在するロボットIDをリスト化
    #     # TODO: ここ、geometry_toolsのTrans使えばきれいに書けそう
    #     # TODO: x座標でしか評価されていないので、自チーム側へのパスに対応できない
    #     forward_robots_id = []
    #     for robot_id in robots_id:
    #         # 自身のIDをスキップする場合
    #         if skip_my_id and my_robot_id == robot_id:
    #             # 自身のIDをスキップ
    #             continue

    #         target_robot_pos = robots_pos[robot_id]
    #         target_robot_vel = robots_vel[robot_id]

    #         # TODO(ShotaAk): ソフト構造を変更し、問題の根本解決をすべき
    #         if target_robot_pos is None:
    #             continue

    #         estimated_displacement = 0.0
    #         vel_norm = 0.0
    #         if target_robot_vel is not None:
    #             vel_norm = math.hypot(target_robot_vel.x, target_robot_vel.y)

    #             # dt時間後の移動距離を計算
    #             if not math.isclose(vel_norm, 0.0, abs_tol=0.000001):  # ゼロ除算回避
    #                 estimated_displacement = (abs(target_robot_vel.x) *
    #                                           dt + robot_r) * target_robot_vel.x / vel_norm

    #         if my_robot_pos.x < target_robot_pos.x + estimated_displacement:
    #             forward_robots_id.append(robot_id)

    #     # forward_robots_id = [
    #     #     _id for _id in robots_id
    #     #     if my_robot_pos[0] < robots_pos[_id][0] \
    #     #       + (abs(robots_vel[_id][0]) * dt + robot_r) * \
    #     #       robots_vel[_id][0] / math.sqrt(robots_vel[_id][0] ** 2 \
    #     #       + robots_vel[_id][1] ** 2)]

    #     return forward_robots_id
