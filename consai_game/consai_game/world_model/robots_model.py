#!/usr/bin/env python3
# coding: UTF-8

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

"""
ロボットの状態を管理するモデルの定義.

SSL-Visionの検出結果からロボットの情報を抽出し, 自チームと相手チームの情報に分類して保持する処理.
"""

from dataclasses import dataclass, field

from consai_msgs.msg import State2D

from robocup_ssl_msgs.msg import RobotId, TrackedFrame, TrackedRobot


@dataclass
class Robot:
    """単一ロボットの状態情報を保持するデータ構造."""

    robot_id: int = 0
    is_yellow: bool = False
    pos: State2D = field(default_factory=State2D)
    vel: State2D = field(default_factory=State2D)
    is_visible: bool = False


class RobotsModel:
    """ロボットデータを保持するクラス."""

    def __init__(self, our_team_is_yellow: bool = False):
        """ロボットモデルの初期化関数."""
        self.our_team_is_yellow = our_team_is_yellow
        self.our_robots: dict[int, Robot] = {}
        self.their_robots: dict[int, Robot] = {}
        self.our_visible_robots: dict[int, Robot] = {}
        self.their_visible_robots: dict[int, Robot] = {}

        self.visibility_threshold = 0.2
        self.robot_radius = 0.1

    def parse_frame(self, msg: TrackedFrame):
        """フレームデータを解析し, ロボット情報をour/theirに分類して格納する関数."""

        def update_visible_robots(robots: dict[int, Robot], robot: Robot):
            if robot.is_visible:
                robots[robot.robot_id] = robot
            else:
                if robot.robot_id in robots:
                    del robots[robot.robot_id]

        for robot_frame in msg.robots:
            robot = self.to_robot_data(robot_frame)
            if robot.is_yellow == self.our_team_is_yellow:
                self.our_robots[robot.robot_id] = robot
                update_visible_robots(self.our_visible_robots, robot)
            else:
                self.their_robots[robot.robot_id] = robot
                update_visible_robots(self.their_visible_robots, robot)

    def to_robot_data(self, robot_frame: TrackedRobot) -> Robot:
        """TrackedRobotメッセージをRobotデータ構造に変換する関数."""
        robot = Robot()
        robot.robot_id = robot_frame.robot_id.id
        robot.is_yellow = robot_frame.robot_id.team_color == RobotId.TEAM_COLOR_YELLOW
        robot.pos.x = robot_frame.pos.x
        robot.pos.y = robot_frame.pos.y
        robot.pos.theta = robot_frame.orientation

        if robot_frame.vel and robot_frame.vel_angular:
            robot.vel.x = robot_frame.vel[0].x
            robot.vel.y = robot_frame.vel[0].y
            robot.vel.theta = robot_frame.vel_angular[0]

        if robot_frame.visibility:
            robot.is_visible = robot_frame.visibility[0] > self.visibility_threshold
        else:
            robot.is_visible = False

        return robot

    @property
    def blue_robot_num(self) -> int:
        """青チームのロボット数を取得する."""
        if self.our_team_is_yellow:
            return len(self.their_visible_robots)
        return len(self.our_visible_robots)

    @property
    def yellow_robot_num(self) -> int:
        """黄色チームのロボット数を取得する."""
        if self.our_team_is_yellow:
            return len(self.our_visible_robots)
        return len(self.their_visible_robots)
