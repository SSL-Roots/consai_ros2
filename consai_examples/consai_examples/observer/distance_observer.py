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

from consai_msgs.msg import State2D
from consai_examples.observer.pos_vel import PosVel
from consai_tools.geometry import geometry_tools


class DistanceObserver:
    def __init__(self):
        self._field_half_length = 6.0
        self._field_half_width = 4.5
        self._defense_area_length = 1.8
        self._defense_area_half_width = 1.8

        self._our_robots: dict[int, PosVel] = {}
        self._their_robots: dict[int, PosVel] = {}
        self._ball_pos = State2D()

    def update(self, ball_pos: State2D,
               our_robots: dict[int, PosVel], their_robots: dict[int, PosVel]) -> None:
        self._ball_pos = ball_pos
        self._our_robots = our_robots
        self._their_robots = their_robots

    def ball_to_our_robots(self) -> list:
        return self._calc_dist_ball_to_robots(self._our_robots)

    def ball_to_their_robots(self) -> list:
        return self._calc_dist_ball_to_robots(self._their_robots)

    def _calc_dist_ball_to_robots(self, robots: dict[int, PosVel]) -> list:
        distance = []
        for robot_id in robots.keys():
            robot_pos = robots[robot_id].pos()
            distance.append(geometry_tools.get_distance(robot_pos, self._ball_pos))
        return distance
        # return [geometry_tools.get_distance(robot.pos(), ball_pos()) for robot in robots]
