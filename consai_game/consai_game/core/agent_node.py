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
from rclpy.node import Node


class AgentNode(Node):
    def __init__(self, update_hz: float = 10, index: int = 0):
        super().__init__(f"agent_node_{index}")

        self.timer = self.create_timer(1.0 / update_hz, self.update)
        self.robot_id = None
        self.tactics = []
        self.present_tactic = 0

    def update(self):
        self.get_logger().info(f"Tactic update, {process_info()}")

        if self.robot_id is None:
            return

        if len(self.tactics) == 0:
            return

        if self.present_tactic >= len(self.tactics):
            self.present_tactic = 0
