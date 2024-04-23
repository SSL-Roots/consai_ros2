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
from consai_visualizer_msgs.msg import Objects
from consai_visualizer_msgs.msg import ShapeLine
import math

OurIDType = int
TheirIDType = int
InValidID = -1


# 指定した自チームのロボットが、どの相手ロボットをマークすべきかを判定するObserver
class ManMarkObserver:
    def __init__(self):
        self._mark_dict: dict[OurIDType, TheirIDType] = {}
        self._our_active_bot_ids: list[OurIDType] = []

    def update(self, ball: PosVel,
               our_robots: dict[OurIDType, PosVel], their_robots: dict[TheirIDType, PosVel]) -> None:
        DETECTION_THRESHOLD_X = 0.0

        self._remove_non_exist_mark(their_robots)

        # 自チームサイドの相手ロボットを全探索
        for their_bot_id, their_bot in their_robots.items():
            if their_bot.pos().x > DETECTION_THRESHOLD_X:
                self._remove_mark_via_their_id(their_bot_id)
                continue

            if self._is_marked(their_bot_id):
                continue

            self._assign_mark(
                self._search_nearest_our_bot(
                    their_bot.pos(), self._our_free_bot_ids(), our_robots), their_bot_id)

    def set_our_active_bot_ids(self, our_active_bot_ids: list[OurIDType]) -> None:
        self._our_active_bot_ids = our_active_bot_ids

    def get_mark_robot_id(self, our_id: OurIDType) -> TheirIDType:
        if our_id in self._mark_dict.keys():
            return self._mark_dict[our_id]
        else:
            return InValidID

    def to_visualize_msg(
            self, our_robots: dict[OurIDType, PosVel], their_robots: dict[TheirIDType, PosVel]) -> Objects:
        # アクティブロボットとマーク対象の相手ロボットを結ぶ直線を描画する

        vis_objects = Objects()

        vis_objects.layer = 'game'
        vis_objects.sub_layer = 'mark'
        vis_objects.z_order = 4

        for our_id, their_id in self._mark_dict.items():
            if our_id not in our_robots.keys() or their_id not in their_robots.keys():
                continue

            our_pos = our_robots[our_id].pos()
            their_pos = their_robots[their_id].pos()

            line = ShapeLine()
            line.p1.x = our_pos.x
            line.p1.y = our_pos.y
            line.p2.x = their_pos.x
            line.p2.y = their_pos.y
            line.size = 5
            line.color.name = 'deepskyblue'
            line.caption = 'mark' + str(our_id) + '->' + str(their_id)
            vis_objects.lines.append(line)

        return vis_objects

    def _remove_mark_via_their_id(self, their_id: TheirIDType) -> None:
        # 削除するキーを一時リストに保存
        to_remove = [our_id for our_id, their_id_ in self._mark_dict.items() if their_id == their_id_]
        # 実際に削除
        for our_id in to_remove:
            self._mark_dict.pop(our_id)

    def _remove_non_exist_mark(self, their_robots: dict[TheirIDType, PosVel]) -> None:
        # 存在しないキーを一時リストに保存
        to_remove = [our_id for our_id, their_id in self._mark_dict.items() if their_id not in their_robots]
        # 実際に削除
        for our_id in to_remove:
            self._mark_dict.pop(our_id)

    def _is_marked(self, their_bot_id: TheirIDType) -> bool:
        return their_bot_id in self._mark_dict.values()

    def _our_free_bot_ids(self) -> list[OurIDType]:
        return list(set(self._our_active_bot_ids) - set(self._mark_dict.keys()))

    def _search_nearest_our_bot(
            self, their_bot_pos: State2D,
            our_target_ids: list[OurIDType], our_robots: dict[OurIDType, PosVel]) -> OurIDType:

        nearest_id = InValidID
        nearest_dist = math.inf

        for our_id in our_target_ids:
            our_pos = our_robots[our_id].pos()
            dist = tool.get_distance(our_pos, their_bot_pos)

            if dist < nearest_dist:
                nearest_id = our_id
                nearest_dist = dist

        return nearest_id

    def _assign_mark(self, our_id: OurIDType, their_id: TheirIDType) -> None:
        if our_id != InValidID:
            self._mark_dict[our_id] = their_id
