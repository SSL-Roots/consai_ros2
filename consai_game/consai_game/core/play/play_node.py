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
Playの実行とRoleの割り当てを管理するノード.

指定されたplaybookを読み込み.WorldModelに応じて適切なPlayを選択し.
RoleAssignorを用いて各ロボットにRoleを割り当て.Playの更新を行う.
"""

import argparse
import importlib.util
from pathlib import Path
import threading
from typing import Callable, Optional

from rclpy.node import Node

from consai_game.core.play.play import Play
from consai_game.core.role_assignment.assignor import RoleAssignor
from consai_game.core.role_assignment.factory import (
    create_method as create_role_assignment_method,
)
from consai_game.core.role_assignment.factory import RoleAssignmentMethods
from consai_game.core.tactic.role import Role
from consai_game.utils.process_info import process_info
from consai_game.world_model.world_model import WorldModel


UpdateRoleCallback = Callable[[list[Role]], None]


class PlayNode(Node):
    """Playの実行とRoleの割り当てを行うクラス."""

    def __init__(self, update_hz: float = 10, book_name: str = ""):
        """PlayNodeを初期化する."""
        super().__init__("play_node")
        self.lock = threading.Lock()

        self.timer = self.create_timer(1.0 / update_hz, self.update)
        self.playbook = self.load_playbook(book_name)
        self.current_play = None

        self.world_model = WorldModel()
        self.role_assignor: RoleAssignor = None

        self.update_role_callback: UpdateRoleCallback = None

        # Playをリセットするためにロボット数を保存する
        self.our_robots_num = 0

    @classmethod
    def add_arguments(cls, parser: argparse.ArgumentParser):
        """コマンドライン引数を追加する関数."""
        parser.add_argument(
            "--playbook",
            default="",
            type=str,
            help="Set playbook file path (e.g.: path/to/playbook.py)",
        )
        parser.add_argument(
            "--assign",
            choices=[m.value for m in RoleAssignmentMethods],
            required=True,
        )
        parser.add_argument(
            "--goalie",
            default=0,
            type=int,
            help="Set goalie id",
        )

    def load_playbook(self, file_path: str) -> list[Play]:
        """指定されたファイルパスからplaybookを読み込む関数."""
        file_path = Path(file_path).resolve()
        spec = importlib.util.spec_from_file_location(file_path.stem, file_path)
        if spec and spec.loader:
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)

            if hasattr(module, "plays"):
                print(f"Loaded playbook: {file_path}")
                return module.plays
            else:
                raise ValueError(f"Playbook: {file_path} does not have 'plays'")
        else:
            raise ValueError(f"Failed to load playbook: {file_path}")

    def set_world_model(self, world_model: WorldModel):
        """WorldModelを設定する関数."""
        with self.lock:
            self.world_model = world_model

    def set_update_role_callback(self, callback: UpdateRoleCallback):
        """Role更新用のコールバック関数を設定する関数."""
        with self.lock:
            self.update_role_callback = callback

    def select_role_assignment_method(self, name: str, goalie_id: int):
        """RoleAssignorを生成して設定する関数."""
        with self.lock:
            self.role_assignor = RoleAssignor(create_role_assignment_method(name), goalie_id)

            if self.role_assignor is None:
                text = "Role assignment method is not set"
                self.get_logger().error(text)
                raise ValueError(text)

    def update(self):
        """Playを更新してRoleを割り当てる関数."""
        with self.lock:
            self.get_logger().debug(f"Play update, {process_info()}")

            if self.current_play is None:
                self.current_play = self.select_play()
                if self.current_play is None:
                    return

                self.update_role()
                self.get_logger().info(f"Selected play: {self.current_play.name}")

            self.execute_play()

            self.evaluate_play()

    def select_play(self) -> Optional[Play]:
        """WorldModelに基づいて適用可能なPlayを選択する関数."""
        applicable_plays = [play for play in self.playbook if play.is_applicable(self.world_model)]

        if not applicable_plays:
            self.get_logger().warn("No applicable plays found")
            return None

        # TODO: 評価関数を実装して選択する
        return applicable_plays[0]

    def execute_play(self):
        """Playを実行する関数（未実装）."""
        # TODO: current_playを実行する
        pass

    def evaluate_play(self):
        """Playの継続可否を評価する関数."""
        # current_playを継続して実行できるか評価する
        if self.current_play.should_abort(self.world_model):
            self.get_logger().info(f"Play aborted: {self.current_play.name}")
            self.current_play = None

        # ロボットの台数が変わったらPlayをリセットする
        visible_robots_num = len(self.world_model.robot_activity.our_visible_robots)
        if self.our_robots_num != visible_robots_num:
            self.our_robots_num = visible_robots_num
            self.get_logger().info(f"Robots num changed. Play aborted: {self.current_play.name}")
            self.current_play = None

    def update_role(self):
        """Roleを更新し, コールバックに渡す関数."""
        # Role(tacticとrobot_id)を更新し、callback関数にセットする
        assigned_ids = self.role_assignor.assign(self.current_play.roles, self.world_model)

        roles = []
        for i in range(len(self.current_play.roles)):
            roles.append(Role(tactics=self.current_play.roles[i], robot_id=assigned_ids[i]))

        self.update_role_callback(roles)
