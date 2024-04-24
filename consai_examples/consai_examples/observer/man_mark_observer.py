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

IDType = int
OurIDType = int
TheirIDType = int
InValidID = -1


# 指定した自チームのロボットが、どの相手ロボットをマークすべきかを判定するObserver
class ManMarkObserver:
    def __init__(self):
        self._mark_dict: dict[OurIDType, TheirIDType] = {}
        self._our_active_bot_ids: list[OurIDType] = []

    def update(self, ball: PosVel,
               our_robots: dict[OurIDType, PosVel],
               their_robots: dict[TheirIDType, PosVel]) -> None:
        self._remove_non_exist_robots_from_mark(our_robots, their_robots)

        # マーク条件に該当するロボットを抽出
        # ここの条件を変更することで、より賢いマークを実現できる
        markable_ids = self._detect_their_bots_in_our_side(their_robots)

        non_markable_ids = set(their_robots.keys()) - set(markable_ids)
        for non_markable_id in non_markable_ids:
            self._remove_mark_via_their_id(non_markable_id)

        # 既にマークされているロボットをマーク対象リストから削除
        already_marked_ids = [their_id for their_id in markable_ids if self._is_marked(their_id)]
        markable_ids = list(set(markable_ids) - set(already_marked_ids))

        for our_bot_id in self._our_active_bot_ids:
            if len(markable_ids) == 0:
                return

            if our_bot_id not in our_robots.keys():
                continue

            if not self._is_free_bot(our_bot_id):
                continue

            # 対象のロボットに最も近いロボットをマークに割り当てる
            our_bot = our_robots[our_bot_id]
            nearest_their_bot_id = self._search_nearest_bot_id(
                our_bot.pos(), their_robots, markable_ids)

            # アサインに成功したら、マーク対象リストからIDを削除
            if self._assign_mark(our_bot_id, nearest_their_bot_id):
                if nearest_their_bot_id in markable_ids:
                    markable_ids.remove(nearest_their_bot_id)

    def set_our_active_bot_ids(self, our_active_bot_ids: list[OurIDType]) -> None:
        self._our_active_bot_ids = our_active_bot_ids

    def get_mark_robot_id(self, our_id: OurIDType) -> TheirIDType:
        if our_id in self._mark_dict.keys():
            return self._mark_dict[our_id]
        else:
            return InValidID

    def to_visualize_msg(
            self, our_robots: dict[OurIDType, PosVel],
            their_robots: dict[TheirIDType, PosVel]) -> Objects:
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

    def _detect_their_bots_in_our_side(
            self, their_robots: dict[OurIDType, PosVel]) -> list[OurIDType]:
        DETECTION_THRESHOLD_X = 0.0

        bot_ids = []
        for bot_id, bot in their_robots.items():
            if bot.pos().x < DETECTION_THRESHOLD_X:
                bot_ids.append(bot_id)

        return bot_ids

    def _remove_mark_via_their_id(self, their_id: TheirIDType) -> None:
        # 削除するキーを一時リストに保存
        to_remove = [
            our_id for our_id, their_id_ in self._mark_dict.items() if their_id == their_id_]
        # 実際に削除
        for our_id in to_remove:
            self._mark_dict.pop(our_id)

    def _remove_non_exist_robots_from_mark(
            self, our_robots: dict[OurIDType, PosVel],
            their_robots: dict[TheirIDType, PosVel]) -> None:
        # 存在しないキーを一時リストに保存
        to_remove_via_theirs = [
            our_id for our_id, their_id in self._mark_dict.items() if their_id not in their_robots]
        to_remove_via_ours = [
            our_id for our_id in self._mark_dict.keys() if our_id not in our_robots]
        # 実際に削除
        for our_id in to_remove_via_theirs + to_remove_via_ours:
            self._mark_dict.pop(our_id)

    def _is_marked(self, their_bot_id: TheirIDType) -> bool:
        return their_bot_id in self._mark_dict.values()

    def _is_free_bot(self, our_id: OurIDType) -> bool:
        return our_id in self._our_free_bot_ids()

    def _our_free_bot_ids(self) -> list[OurIDType]:
        return list(set(self._our_active_bot_ids) - set(self._mark_dict.keys()))

    def _search_nearest_bot_id(
            self, my_pos: State2D,
            target_robots: dict[IDType, PosVel],
            markable_ids: list[IDType]) -> IDType:

        nearest_id = InValidID
        nearest_dist = math.inf

        for target_id, target_bot in target_robots.items():
            if target_id not in markable_ids:
                continue

            target_pos = target_bot.pos()
            dist = tool.get_distance(my_pos, target_pos)

            if dist < nearest_dist:
                nearest_id = target_id
                nearest_dist = dist

        return nearest_id

    def _assign_mark(self, our_id: OurIDType, their_id: TheirIDType) -> bool:
        if our_id != InValidID:
            self._mark_dict[our_id] = their_id
            return True
        return False
