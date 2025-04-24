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

"""可視ロボットの識別と順序づけを行うモデル."""

from consai_game.world_model.robots_model import RobotsModel, Robot
from consai_game.world_model.ball_model import BallModel
from consai_tools.geometry import geometry_tools as tools


class RobotActivityModel:
    """ロボットの活動状態を保持するクラス."""

    def __init__(self):
        """ロボットの可視状態と順序リストの初期化関数."""
        self.ordered_our_visible_robots: list[int] = []
        self.ordered_their_visible_robots: list[int] = []
        self.our_visible_robots: list[int] = []
        self.their_visible_robots: list[int] = []
        self.our_robots_by_ball_distance: list[int] = []
        self.their_robots_by_ball_distance: list[int] = []

    def update(self, robots_model: RobotsModel, ball: BallModel):
        """ロボットの可視状態を更新し, 順序づけされたIDリストを更新する関数."""
        self.our_visible_robots = [robot.robot_id for robot in robots_model.our_robots.values() if robot.is_visible]
        self.their_visible_robots = [robot.robot_id for robot in robots_model.their_robots.values() if robot.is_visible]

        self.ordered_our_visible_robots = self.ordered_merge(
            self.ordered_our_visible_robots,
            self.our_visible_robots,
        )
        self.ordered_their_visible_robots = self.ordered_merge(
            self.ordered_their_visible_robots,
            self.their_visible_robots,
        )

        # ボールに近い順にリストを作る
        self.our_robots_by_ball_distance = [
            robot_id
            for robot_id, _ in self.robot_ball_distances(
                robots_model.our_visible_robots,
                ball,
            )
        ]
        self.their_robots_by_ball_distance = [
            robot_id
            for robot_id, _ in self.robot_ball_distances(
                robots_model.their_visible_robots,
                ball,
            )
        ]

    def ordered_merge(self, prev_list: list[int], new_list: list[int]) -> list[int]:
        """過去の順序を保ちながら, 新しいリストでマージする関数."""
        # new_listに存在するものを残す
        output_list = [r for r in prev_list if r in new_list]

        # 新しい要素を追加する
        output_list.extend([r for r in new_list if r not in output_list])

        return output_list

    def robot_ball_distances(self, robots: dict[int, Robot], ball: BallModel) -> list[tuple[int, float]]:
        """ロボットとボールの距離を計算し, 距離の昇順でソートしたリストを返す関数."""

        robot_ball_distances = []

        for robot in robots.values():
            distance = tools.get_distance(robot.pos, ball.pos)
            robot_ball_distances.append((robot.robot_id, distance))

        # 距離でソート
        robot_ball_distances.sort(key=lambda x: x[1])

        return robot_ball_distances
