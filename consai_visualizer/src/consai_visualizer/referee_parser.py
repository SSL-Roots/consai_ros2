# coding: UTF-8

# Copyright 2021 Roots
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

import math

from robocup_ssl_msgs.msg import Referee

def parse_stage(ref_stage):
    # レフェリーステージを文字列に変換する
    output = "STAGE"

    if ref_stage == Referee.STAGE_NORMAL_FIRST_HALF_PRE:
        output = "FIRST HALF PRE"
    elif ref_stage == Referee.STAGE_NORMAL_FIRST_HALF:
        output = "FIRST HALF"
    elif ref_stage == Referee.STAGE_NORMAL_HALF_TIME:
        output = "HALF TIME"
    elif ref_stage == Referee.STAGE_NORMAL_SECOND_HALF_PRE:
        output = "SECOND HALF PRE"
    elif ref_stage == Referee.STAGE_NORMAL_SECOND_HALF:
        output = "SECOND HALF"
    elif ref_stage == Referee.STAGE_EXTRA_TIME_BREAK:
        output = "EX TIME BREAK"
    elif ref_stage == Referee.STAGE_EXTRA_FIRST_HALF_PRE:
        output = "EX FIRST HALF PRE"
    elif ref_stage == Referee.STAGE_EXTRA_FIRST_HALF:
        output = "EX FIRST HALF"
    elif ref_stage == Referee.STAGE_EXTRA_HALF_TIME:
        output = "EX HALF TIME"
    elif ref_stage == Referee.STAGE_EXTRA_SECOND_HALF_PRE:
        output = "EX SECOND HALF PRE"
    elif ref_stage == Referee.STAGE_EXTRA_SECOND_HALF:
        output = "EX SECOND HALF"
    elif ref_stage == Referee.STAGE_PENALTY_SHOOTOUT_BREAK:
        output = "PENALTY SHOOTOUT BREAK"
    elif ref_stage == Referee.STAGE_PENALTY_SHOOTOUT:
        output = "PENALTY SHOOTOUT"
    elif ref_stage == Referee.STAGE_POST_GAME:
        output = "POST_GAME"

    return output

def parse_command(ref_command):
    # レフェリーコマンドを文字列に変換する
    output = "COMMAND"

    if ref_command == Referee.COMMAND_HALT:
        output = "HALT"
    elif ref_command == Referee.COMMAND_STOP:
        output = "STOP"
    elif ref_command == Referee.COMMAND_NORMAL_START:
        output = "NORMAL START"
    elif ref_command == Referee.COMMAND_FORCE_START:
        output = "FORCE START"
    elif ref_command == Referee.COMMAND_PREPARE_KICKOFF_YELLOW:
        output = "PREPARE KICK OFF YELLOW"
    elif ref_command == Referee.COMMAND_PREPARE_KICKOFF_BLUE:
        output = "PREPARE KICK OFF BLUE"
    elif ref_command == Referee.COMMAND_PREPARE_PENALTY_YELLOW:
        output = "PREPARE PENALTY YELLOW"
    elif ref_command == Referee.COMMAND_PREPARE_PENALTY_BLUE:
        output = "PREPARE PENALTY BLUE"
    elif ref_command == Referee.COMMAND_DIRECT_FREE_YELLOW:
        output = "DIRECT FREE YELLOW"
    elif ref_command == Referee.COMMAND_DIRECT_FREE_BLUE:
        output = "DIRECT FREE BLUE"
    elif ref_command == Referee.COMMAND_INDIRECT_FREE_YELLOW:
        output = "INDIRECT FREE YELLOW"
    elif ref_command == Referee.COMMAND_INDIRECT_FREE_BLUE:
        output = "INDIRECT FREE BLUE"
    elif ref_command == Referee.COMMAND_TIMEOUT_YELLOW:
        output = "TIMEOUT YELLOW"
    elif ref_command == Referee.COMMAND_TIMEOUT_BLUE:
        output = "TIMEOUT BLUE"
    elif ref_command == Referee.COMMAND_BALL_PLACEMENT_YELLOW:
        output = "BALL PLACEMENT YELLOW"
    elif ref_command == Referee.COMMAND_BALL_PLACEMENT_BLUE:
        output = "BALL PLACEMENT BLUE"

    return output

def _microseconds_to_text(microseconds):
    minutes, seconds = divmod(math.ceil(microseconds * 1e-6), 60)  # ceilで小数点切り上げ
    return '{} : {:0=2}'.format(minutes, seconds)  # 秒はゼロで埋める

def parse_stage_time_left(ref_stage_time_left):
    # レフェリーステージの残り時間(usec)を文字列に変換する
    return "STAGE: " + _microseconds_to_text(ref_stage_time_left)

def parse_action_time_remaining(ref_action_time_remaining):
    # アクション残り時間(usec)を文字列に変換する
    text = "0:00"
    if ref_action_time_remaining > 0:
        text = _microseconds_to_text(ref_action_time_remaining)
    return "ACT: " + text

def parse_red_cards(ref_team_red_cards):
    return str(ref_team_red_cards)

def parse_yellow_cards(ref_team_yellow_cards):
    return str(ref_team_yellow_cards)

def parse_yellow_card_times(yellow_card_times):
    text = ""
    for time in yellow_card_times:
        text += _microseconds_to_text(time) + ", "
    return text

def parse_timeouts(timeouts):
    return 'Timeouts: ' + str(timeouts)

def parse_timeout_time(timeout_time):
    return _microseconds_to_text(timeout_time)

def is_ball_placement(ref_command):
    if ref_command == Referee.COMMAND_BALL_PLACEMENT_YELLOW:
        return True
    elif ref_command == Referee.COMMAND_BALL_PLACEMENT_BLUE:
        return True
    return False