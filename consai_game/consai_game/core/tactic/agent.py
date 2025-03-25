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

from consai_game.core.tactic.role import Role
from consai_msgs.msg import MotionCommand
from typing import Optional
import rclpy
import rclpy.logging


class Agent():
    def __init__(self):

        self.role = Role()
        self.present_tactic = 0

    def update(self) -> Optional[MotionCommand]:
        if self.role.robot_id < 0 or len(self.role.tactics) == 0:
            return None

        command = self.execute_tactic()

        if self.present_tactic >= len(self.role.tactics):
            self.present_tactic = 0

        return command

    def set_role(self, role: Role) -> None:
        self.role = role

    def execute_tactic(self) -> MotionCommand:
        command = MotionCommand()
        command.robot_id = self.role.robot_id
        command.mode = MotionCommand.MODE_NAVI
        command.desired_pose.x = self.role.robot_id * 0.2
        command.desired_pose.y = self.role.robot_id * 0.2
        command.desired_pose.theta = self.role.robot_id * 0.2

        return command
