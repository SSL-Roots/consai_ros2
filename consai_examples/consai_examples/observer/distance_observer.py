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

"""ロボットとボールの距離を観測するモジュール."""

from consai_examples.observer.pos_vel import PosVel

from consai_msgs.msg import State2D

from consai_tools.geometry import geometry_tools


class DistanceObserver:
    """ロボットとボールの距離を観測するクラス."""

    def __init__(self):
        """DistanceObserverを初期化する関数."""
        self._our_robots: dict[int, PosVel] = {}
        self._their_robots: dict[int, PosVel] = {}
        self._ball_pos = State2D()

    def update(self, ball_pos: State2D,
               our_robots: dict[int, PosVel], their_robots: dict[int, PosVel]) -> None:
        """ロボットとボールの状態を更新する関数."""
        self._ball_pos = ball_pos
        self._our_robots = our_robots
        self._their_robots = their_robots

    def ball_to_our_robots(self) -> dict:
        """ボールと自分のロボットの距離を計算する関数."""
        return self._calc_dist_ball_to_robots(self._our_robots)

    def ball_to_their_robots(self) -> dict:
        """ボールと相手のロボットの距離を計算する関数."""
        return self._calc_dist_ball_to_robots(self._their_robots)

    def _calc_dist_ball_to_robots(self, robots: dict[int, PosVel]) -> dict:
        """ロボットとボールの距離を計算する関数."""
        distance = {}
        for robot_id in robots.keys():
            robot_pos = robots[robot_id].pos()
            # distance.append(geometry_tools.get_distance(robot_pos, self._ball_pos))
            distance[robot_id] = geometry_tools.get_distance(robot_pos, self._ball_pos)
        return distance
