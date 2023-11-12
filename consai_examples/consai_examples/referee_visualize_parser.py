# coding: UTF-8

# Copyright 2023 Roots
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

from consai_visualizer_msgs.msg import Objects
from consai_visualizer_msgs.msg import ShapeAnnotation
from robocup_ssl_msgs.msg import Referee

def to_visualizer_objects(referee: Referee):
    # レフェリー情報を描画オブジェクトに変換する
    MARGIN_X = 0.02
    TEXT_HEIGHT = 0.05
    STAGE_COMMAND_WIDTH = 0.15
    STAGE_COMMAND_X = 0.0 + MARGIN_X
    TIMER_WIDTH = 0.15
    TIMER_X = STAGE_COMMAND_X + STAGE_COMMAND_WIDTH + MARGIN_X
    BOTS_WIDTH = 0.2
    BOTS_X = TIMER_X + TIMER_WIDTH + MARGIN_X
    CARDS_WIDTH = 0.1
    CARDS_X = BOTS_X + BOTS_WIDTH + MARGIN_X
    YELLOW_CARD_TIMES_WIDTH = 0.1
    YELLOW_CARD_TIMES_X = CARDS_X + CARDS_WIDTH + MARGIN_X
    TIMEOUT_WIDTH = 0.1
    TIMEOUT_X = YELLOW_CARD_TIMES_X + YELLOW_CARD_TIMES_WIDTH + MARGIN_X

    vis_objects = Objects()
    vis_objects.layer = 'referee'
    vis_objects.sub_layer = 'status'

    # 左端にSTAGEとCOMMANDを表示
    vis_annotation = ShapeAnnotation()
    vis_annotation.text = parse_stage(referee.stage)
    vis_annotation.normalized_x = STAGE_COMMAND_X
    vis_annotation.normalized_y = 0.0
    vis_annotation.normalized_width = STAGE_COMMAND_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    vis_annotation = ShapeAnnotation()
    vis_annotation.text = parse_command(referee.command)
    vis_annotation.normalized_x = STAGE_COMMAND_X
    vis_annotation.normalized_y = TEXT_HEIGHT
    vis_annotation.normalized_width = STAGE_COMMAND_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    # 残り時間とACT_TIMEを表示
    if referee.stage_time_left:
        vis_annotation = ShapeAnnotation()
        vis_annotation.text = parse_stage_time_left(referee.stage_time_left[0])
        vis_annotation.normalized_x = TIMER_X
        vis_annotation.normalized_y = 0.0
        vis_annotation.normalized_width = TIMER_WIDTH
        vis_annotation.normalized_height = TEXT_HEIGHT
        vis_objects.annotations.append(vis_annotation)

    if referee.current_action_time_remaining:
        vis_annotation = ShapeAnnotation()
        vis_annotation.text = parse_action_time_remaining(referee.current_action_time_remaining[0])
        vis_annotation.normalized_x = TIMER_X
        vis_annotation.normalized_y = TEXT_HEIGHT
        vis_annotation.normalized_width = TIMER_WIDTH
        vis_annotation.normalized_height = TEXT_HEIGHT
        vis_objects.annotations.append(vis_annotation)

    # ロボット数
    if referee.blue.max_allowed_bots:
        blue_bots = 11  # TODO(ShotaAk): 実際のロボット数を入力する
        vis_annotation = ShapeAnnotation()
        vis_annotation.text = 'BLUE BOTS: {}/{}'.format(
            blue_bots, referee.blue.max_allowed_bots[0])
        vis_annotation.normalized_x = BOTS_X
        vis_annotation.normalized_y = 0.0
        vis_annotation.normalized_width = BOTS_WIDTH
        vis_annotation.normalized_height = TEXT_HEIGHT
        vis_objects.annotations.append(vis_annotation)

    if referee.yellow.max_allowed_bots:
        yellow_bots = 11  # TODO(ShotaAk): 実際のロボット数を入力する
        vis_annotation = ShapeAnnotation()
        vis_annotation.text = 'YELLOW BOTS: {}/{}'.format(
            yellow_bots, referee.yellow.max_allowed_bots[0])
        vis_annotation.normalized_x = BOTS_X
        vis_annotation.normalized_y = TEXT_HEIGHT
        vis_annotation.normalized_width = BOTS_WIDTH
        vis_annotation.normalized_height = TEXT_HEIGHT
        vis_objects.annotations.append(vis_annotation)
    
    # カード数
    vis_annotation = ShapeAnnotation()
    vis_annotation.text = 'R: {}, Y:{}'.format(
        referee.blue.red_cards,
        referee.blue.yellow_cards)
    vis_annotation.normalized_x = CARDS_X
    vis_annotation.normalized_y = 0.0
    vis_annotation.normalized_width = CARDS_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    vis_annotation = ShapeAnnotation()
    vis_annotation.text = 'R: {}, Y:{}'.format(
        referee.yellow.red_cards,
        referee.yellow.yellow_cards)
    vis_annotation.normalized_x = CARDS_X
    vis_annotation.normalized_y = TEXT_HEIGHT
    vis_annotation.normalized_width = CARDS_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    # イエローカードの時間
    vis_annotation = ShapeAnnotation()
    vis_annotation.text = parse_yellow_card_times(referee.blue.yellow_card_times)
    vis_annotation.normalized_x = YELLOW_CARD_TIMES_X
    vis_annotation.normalized_y = 0.0
    vis_annotation.normalized_width = YELLOW_CARD_TIMES_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    vis_annotation = ShapeAnnotation()
    vis_annotation.text = parse_yellow_card_times(referee.yellow.yellow_card_times)
    vis_annotation.normalized_x = YELLOW_CARD_TIMES_X
    vis_annotation.normalized_y = TEXT_HEIGHT
    vis_annotation.normalized_width = YELLOW_CARD_TIMES_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    # タイムアウト
    vis_annotation = ShapeAnnotation()
    vis_annotation.text = parse_timeouts(referee.blue.timeouts, referee.blue.timeout_time)
    vis_annotation.normalized_x = TIMEOUT_X
    vis_annotation.normalized_y = 0.0
    vis_annotation.normalized_width = TIMEOUT_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    vis_annotation = ShapeAnnotation()
    vis_annotation.text = parse_timeouts(referee.yellow.timeouts, referee.yellow.timeout_time)
    vis_annotation.normalized_x = TIMEOUT_X
    vis_annotation.normalized_y = TEXT_HEIGHT
    vis_annotation.normalized_width = TIMEOUT_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    return vis_objects

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
    if len(yellow_card_times) == 0:
        return "NO CARDS"

    text = ""
    for i in range(len(yellow_card_times)):
        text += _microseconds_to_text(yellow_card_times[i])
        if i != len(yellow_card_times) - 1:
            text += "\n"

    return text


def parse_timeouts(timeouts, timeout_time):
    return 'Timeouts: {}\n {}'.format(
        timeouts, _microseconds_to_text(timeout_time))


def parse_timeout_time(timeout_time):
    return _microseconds_to_text(timeout_time)


def is_ball_placement(ref_command):
    if ref_command == Referee.COMMAND_BALL_PLACEMENT_YELLOW:
        return True
    elif ref_command == Referee.COMMAND_BALL_PLACEMENT_BLUE:
        return True
    return False
