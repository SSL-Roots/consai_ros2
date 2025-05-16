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
エージェントの動作を管理するモジュール.

エージェントはロールに基づき戦術を実行し, 状態に応じて次の戦術を選択する.
"""

from typing import Optional

from consai_msgs.msg import MotionCommand

from consai_game.core.tactic.role import Role, RoleConst
from consai_game.core.tactic.tactic_base import TacticState
from consai_game.core.tactic.robot_tactic_status import RobotTacticStatus
from consai_game.world_model.world_model import WorldModel


class Agent:
    """エージェントの動作を管理するクラス."""

    def __init__(self):
        """エージェントの初期化を行う関数."""
        self.role = Role()
        self.present_tactic = None
        self.present_tactic_index = 0

    def update(self, world_model: WorldModel) -> Optional[MotionCommand]:
        """エージェントの動作を更新し, 次のMotionCommandを返す関数."""
        if self.present_tactic is None:
            return None

        if self.role.robot_id == RoleConst.INVALID_ROLE_ID:
            return None

        # ロボットがvisibleでなければ終了
        if self.role.robot_id not in world_model.robots.our_visible_robots.keys():
            return None

        # Tacticの処理が終了していたら、次のTacticに移る
        if self.present_tactic.state == TacticState.FINISHED:
            self.present_tactic_index += 1

            # すべてのTacticを実行したら、最初のTacticに戻る
            if self.present_tactic_index >= len(self.role.tactics):
                self.present_tactic_index = 0
            self.reset_tactic()

        return self.present_tactic.run(world_model=world_model)

    def set_role(self, role: Role) -> None:
        """エージェントに新しいロールを設定する関数."""
        if len(role.tactics) == 0:
            raise ValueError("Tactics is empty")

        if not (RoleConst.MIN_VALID_ROLE_ID <= role.robot_id <= RoleConst.MAX_VALID_ROLE_ID):
            if role.robot_id != RoleConst.INVALID_ROLE_ID:
                raise ValueError(f"Invalid robot_id: {role.robot_id}")

        self.role.tactics = role.tactics

        # 実行中のTacticをリセットしないための処理
        if self.role.robot_id == role.robot_id and self.present_tactic is role.tactics[0]:
            return

        self.role.robot_id = role.robot_id
        self.present_tactic_index = 0
        self.reset_tactic()

    def reset_tactic(self) -> None:
        """戦術をリセットし, 次の戦術を設定する関数."""
        if self.present_tactic is not None:
            self.present_tactic.exit()

        self.present_tactic = self.role.tactics[self.present_tactic_index]
        self.present_tactic.reset(self.role.robot_id)

    def get_robot_tactic_status(self) -> Optional[RobotTacticStatus]:
        """ロボットが実行しているTacticの状態を取得する関数.

        TacticかロボットIDが無効な場合はNoneを返す.
        """

        if self.present_tactic is None:
            return None

        if self.role.robot_id == RoleConst.INVALID_ROLE_ID:
            return None

        return RobotTacticStatus(
            robot_id=self.role.robot_id,
            tactic_name=self.present_tactic.name,
            tactic_state="none",  # TODO: tacticが状態遷移器を持っていたら、状態をセットする
        )
