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

from consai_game.utils.process_info import process_info
from consai_game.core.tactic.agent import Agent
from consai_game.core.tactic.role import Role
from consai_msgs.msg import MotionCommandArray
from rclpy.node import Node


class AgentSchedulerNode(Node):
    def __init__(self, update_hz: float = 10, team_is_yellow: bool = False, agent_num: int = 11):
        super().__init__("agent_scheduler_node")

        self.timer = self.create_timer(1.0 / update_hz, self.update)

        self.agents = [Agent() for _ in range(agent_num)]
        self.team_is_yellow = team_is_yellow

        self.pub_motion_commands = self.create_publisher(MotionCommandArray, "motion_commands", 1)

    def update(self):
        self.get_logger().debug(f"Agent Scheduler update, {process_info()}")

        motion_commands = MotionCommandArray()

        for agent in self.agents:
            if command := agent.update():
                motion_commands.commands.append(command)

        if len(motion_commands.commands) <= 0:
            return

        motion_commands.header.stamp = self.get_clock().now().to_msg()
        motion_commands.team_is_yellow = self.team_is_yellow
        self.pub_motion_commands.publish(motion_commands)

    def set_roles(self, roles: list[Role]):
        for role, agent in zip(roles, self.agents):
            agent.set_role(role)
