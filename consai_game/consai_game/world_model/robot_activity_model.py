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


from consai_game.world_model.robots_model import RobotsModel


class RobotActivityModel:
    """ロボットの活動状態を保持するクラス."""

    def __init__(self):
        self.ordered_our_visible_robots: list[int] = []
        self.ordered_their_visible_robots: list[int] = []

    def update(self, robots_model: RobotsModel):
        our_visible_robots = [
            robot.robot_id for robot in robots_model.our_robots.values() if robot.is_visible]
        their_visible_robots = [
            robot.robot_id for robot in robots_model.their_robots.values() if robot.is_visible]

        self.ordered_our_visible_robots = self.ordered_merge(
            self.ordered_our_visible_robots,
            our_visible_robots,
        )
        self.ordered_their_visible_robots = self.ordered_merge(
            self.ordered_their_visible_robots,
            their_visible_robots,
        )

    def ordered_merge(self, prev_list: list[int], new_list: list[int]) -> list[int]:
        # new_listに存在するものを残す
        output_list = [r for r in prev_list if r in new_list]

        # 新しい要素を追加する
        output_list.extend([r for r in new_list if r not in output_list])

        return output_list
