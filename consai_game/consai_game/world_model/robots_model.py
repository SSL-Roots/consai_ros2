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


from consai_msgs.msg import State2D
from dataclasses import dataclass
from robocup_ssl_msgs.msg import RobotId
from robocup_ssl_msgs.msg import TrackedFrame
from robocup_ssl_msgs.msg import TrackedRobot


@dataclass
class Robot:
    robot_id: int = 0
    is_yellow: bool = False
    pos: State2D = State2D()
    vel: State2D = State2D()
    is_visible: bool = False


class RobotsModel:
    """ 未加工のロボットデータを保持するクラス. """

    def __init__(self, our_team_is_yellow: bool = False):
        self.our_team_is_yellow = our_team_is_yellow
        self.our_robots: dict[int, Robot] = {}
        self.their_robots: dict[int, Robot] = {}

        self.visibility_threshold = 0.2

    def parse_frame(self, msg: TrackedFrame):
        for robot_frame in msg.robots:
            robot = self.to_robot_data(robot_frame)
            if robot.is_yellow == self.our_team_is_yellow:
                self.our_robots[robot.robot_id] = robot
            else:
                self.their_robots[robot.robot_id] = robot

    def to_robot_data(self, robot_frame: TrackedRobot) -> Robot:
        robot = Robot()
        robot.robot_id = robot_frame.robot_id.id
        robot.is_yellow = (robot_frame.robot_id.team_color == RobotId.TEAM_COLOR_YELLOW)
        robot.pos.x = robot_frame.pos.x
        robot.pos.y = robot_frame.pos.y
        robot.pos.theta = robot_frame.orientation

        if robot_frame.vel and robot_frame.vel_angular:
            robot.vel.x = robot_frame.vel[0].x
            robot.vel.y = robot_frame.vel[0].y
            robot.vel.theta = robot_frame.vel_angular[0]

        if robot_frame.visibility:
            robot.is_visible = (robot_frame.visibility[0] > self.visibility_threshold)

        return robot
