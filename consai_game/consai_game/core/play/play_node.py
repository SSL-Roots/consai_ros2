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
from consai_game.core.play.play import Play
from consai_game.core.play.librarian import PlaybooksLibrarian
from consai_game.utils.process_info import process_info
from consai_game.world_model.world_model import WorldModel
from rclpy.node import Node


class PlayNode(Node):
    librarian = PlaybooksLibrarian()

    def __init__(self, update_hz: float = 10, book_name: str = ""):
        super().__init__("play_node")

        self.timer = self.create_timer(1.0 / update_hz, self.update)
        self.play_book = self.librarian.get(book_name)
        self.current_play = None

        self.world_model = WorldModel()

    @classmethod
    def add_arguments(cls, parser: argparse.ArgumentParser):
        parser.add_argument(
            "--playbook", default="", type=str,
            help=f"Set playbook name from: [{', '.join(cls.librarian.keys())}]"
        )

    @classmethod
    def register_playbook(cls, name: str, plays: list[Play]):
        cls.librarian.register(name, plays)

    def set_world_model(self, world_model: WorldModel):
        self.world_model = world_model

    def update(self):
        self.get_logger().info(f"Play update, {process_info()}")

        self.evaluate_play()

        if self.current_play is None:
            self.current_play = self.select_play()
            self.get_logger().info(f"Selected play: {self.current_play.name}")
            self.evaluate_play()

    def select_play(self) -> Play:
        applicable_plays = [
            play
            for play in self.play_book
            if play.is_applicable(self.world_model)
        ]

        if not applicable_plays:
            raise ValueError("No applicable plays found")

        # TODO: 評価関数を実装して選択する
        return applicable_plays[0]

    def execute_play(self):
        # TODO: current_playを実行する
        pass

    def evaluate_play(self):
        # TODO: current_playを継続して実行できるか評価する
        pass
