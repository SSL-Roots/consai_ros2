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

import argparse
from consai_game.utils.process_info import process_info
from consai_game.play.play_book import PlayBook
from rclpy.node import Node


class PlayNode(Node):
    def __init__(self, update_hz: float = 10, playbook_dir: str = ''):
        super().__init__('play_node')

        self.timer = self.create_timer(1.0/update_hz, self.update)
        self.play_book = PlayBook.load_from_directory(playbook_dir)

        if len(self.play_book.plays) == 0:
            raise ValueError('No plays found in playbook')

    @staticmethod
    def add_arguments(parser: argparse.ArgumentParser):
        parser.add_argument('--playbook',
                            default="",
                            type=str,
                            help='playbook directory')

    def update(self):
        self.get_logger().info(f'Play update, {process_info()}')
