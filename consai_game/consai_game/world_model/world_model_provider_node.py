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
from consai_game.world_model.world_model import WorldModel
from rclpy.node import Node


class WorldModelProviderNode(Node):
    def __init__(self, update_hz: float = 10):
        super().__init__('world_model_provider_node')
        self.timer = self.create_timer(1.0/update_hz, self.update)

    def update(self):
        self.get_logger().info(f'WorldModelProvider update, {process_info()}')
