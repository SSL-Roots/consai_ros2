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

"""Refereeメッセージを解析し, ゲームの状態を抽象化したモデルに変換するモジュール."""

from dataclasses import dataclass

from robocup_ssl_msgs.msg import Referee
from consai_game.utils.geometry import Point


@dataclass
class RefereeModel:
    """Refereeメッセージを抽象化したゲーム状態を表現するクラス."""

    halt: bool = False
    stop: bool = False
    normal_start: bool = False
    our_free_kick: bool = False
    their_free_kick: bool = False
    our_kick_off: bool = False
    their_kick_off: bool = False
    our_kick_off_start: bool = False
    their_kick_off_start: bool = False
    our_penalty_kick: bool = False
    their_penalty_kick: bool = False
    our_penalty_kick_start: bool = False
    their_penalty_kick_start: bool = False
    our_goal: bool = False
    their_goal: bool = False
    our_timeout: bool = False
    their_timeout: bool = False
    our_ball_placement: bool = False
    their_ball_placement: bool = False
    running: bool = False
    placement_pos: Point = Point(0.0, 0.0)


def parse_referee_msg(
    msg: Referee, prev_data: RefereeModel, our_team_is_yellow: bool, invert: bool, ball_is_moving: bool
) -> RefereeModel:
    """Refereeメッセージを解析し, 現在のゲーム状態を表すRefereeModelを返す関数."""
    data = RefereeModel()

    data.halt = msg.command == Referee.COMMAND_HALT
    data.stop = msg.command == Referee.COMMAND_STOP
    data.running = msg.command == Referee.COMMAND_FORCE_START
    data.normal_start = msg.command == Referee.COMMAND_NORMAL_START

    def parse_team_command(command_blue, command_yellow) -> tuple[bool, bool]:
        """チームごとのコマンド種別を解析する内部関数."""
        is_our_command = (msg.command == command_blue and not our_team_is_yellow) or (
            msg.command == command_yellow and our_team_is_yellow
        )
        is_their_command = (msg.command == command_yellow and not our_team_is_yellow) or (
            msg.command == command_blue and our_team_is_yellow
        )
        return is_our_command, is_their_command

    data.our_free_kick, data.their_free_kick = parse_team_command(
        Referee.COMMAND_DIRECT_FREE_BLUE, Referee.COMMAND_DIRECT_FREE_YELLOW
    )
    data.our_kick_off, data.their_kick_off = parse_team_command(
        Referee.COMMAND_PREPARE_KICKOFF_BLUE, Referee.COMMAND_PREPARE_KICKOFF_YELLOW
    )
    data.our_penalty_kick, data.their_penalty_kick = parse_team_command(
        Referee.COMMAND_PREPARE_PENALTY_BLUE, Referee.COMMAND_PREPARE_PENALTY_YELLOW
    )
    data.our_goal, data.their_goal = parse_team_command(Referee.COMMAND_GOAL_BLUE, Referee.COMMAND_GOAL_YELLOW)
    data.our_timeout, data.their_timeout = parse_team_command(
        Referee.COMMAND_TIMEOUT_BLUE, Referee.COMMAND_TIMEOUT_YELLOW
    )
    data.our_ball_placement, data.their_ball_placement = parse_team_command(
        Referee.COMMAND_BALL_PLACEMENT_BLUE, Referee.COMMAND_BALL_PLACEMENT_YELLOW
    )

    # NORMAL_STARTはKICKOFFとPENALTYを兼任しているので、前回のコマンドをもとにコマンドを判別しなければならない
    if prev_data.our_kick_off:
        data.our_kick_off_start = data.normal_start
    if prev_data.their_kick_off:
        data.their_kick_off_start = data.normal_start
    if prev_data.our_penalty_kick:
        data.our_penalty_kick_start = data.normal_start
    if prev_data.their_penalty_kick:
        data.their_penalty_kick_start = data.normal_start

    # normal_startの継続処理
    if prev_data.our_kick_off_start:
        data.our_kick_off_start = data.normal_start
    if prev_data.their_kick_off_start:
        data.their_kick_off_start = data.normal_start
    if prev_data.our_penalty_kick_start:
        data.our_penalty_kick_start = data.normal_start
    if prev_data.their_penalty_kick_start:
        data.their_penalty_kick_start = data.normal_start

    # runningへの切り替え判断
    # kick_offやfree_kickにはcurrent_action_time_remainingという制限時間が設けられている
    # NOTE: penaltyにも設けられているが、Playの切り替えを防ぐためrunningには切り替えない
    if data.our_free_kick or data.their_free_kick or data.our_kick_off_start or data.their_kick_off_start:
        # 一度runningになったあとは、commandが変わるまで継続する
        if prev_data.running:
            data.running = True
        else:
            # 一定時間が経過したらrunningに切り替わる
            if msg.current_action_time_remaining:
                data.running = msg.current_action_time_remaining[0] < 0

            # ボールが動いたらrunningに切り替わる
            if ball_is_moving:
                data.running = True

    # ボールプレースメント位置
    if len(msg.designated_position) > 0:
        data.placement_pos.x = msg.designated_position[0].x * 0.001  # mm to meters
        data.placement_pos.y = msg.designated_position[0].y * 0.001  # mm to meters

        # フィールドサイドを反転しているときは、目標座標も反転させる
        if invert:
            data.placement_pos.x *= -1.0
            data.placement_pos.y *= -1.0

    return data
