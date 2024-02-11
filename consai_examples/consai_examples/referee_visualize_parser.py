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

from consai_msgs.msg import ParsedReferee
from consai_visualizer_msgs.msg import Objects
from consai_visualizer_msgs.msg import ShapeAnnotation
from consai_visualizer_msgs.msg import ShapeCircle
from consai_visualizer_msgs.msg import ShapeTube
from robocup_ssl_msgs.msg import Point
from robocup_ssl_msgs.msg import Referee
from robocup_ssl_msgs.msg import Vector3


def vis_info(referee: Referee, blue_bots: int, yellow_bots: int,
             placement_pos: Point):
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
    COLOR_TEXT_BLUE = 'deepskyblue'
    COLOR_TEXT_YELLOW = 'yellow'
    COLOR_TEXT_WARNING = 'red'

    vis_objects = Objects()
    vis_objects.layer = 'referee'
    vis_objects.sub_layer = 'info'
    vis_objects.z_order = 2

    # 左端にSTAGEとCOMMANDを表示
    vis_annotation = ShapeAnnotation()
    vis_annotation.text = parse_stage(referee.stage)
    vis_annotation.color.name = 'white'
    vis_annotation.normalized_x = STAGE_COMMAND_X
    vis_annotation.normalized_y = 0.0
    vis_annotation.normalized_width = STAGE_COMMAND_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    vis_annotation = ShapeAnnotation()
    vis_annotation.text, vis_annotation.color.name = parse_command(
        referee, COLOR_TEXT_BLUE, COLOR_TEXT_YELLOW)
    vis_annotation.normalized_x = STAGE_COMMAND_X
    vis_annotation.normalized_y = TEXT_HEIGHT
    vis_annotation.normalized_width = STAGE_COMMAND_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    # 残り時間とACT_TIMEを表示
    if referee.stage_time_left:
        vis_annotation = ShapeAnnotation()
        vis_annotation.text = parse_stage_time_left(referee.stage_time_left[0])
        vis_annotation.color.name = 'white'
        vis_annotation.normalized_x = TIMER_X
        vis_annotation.normalized_y = 0.0
        vis_annotation.normalized_width = TIMER_WIDTH
        vis_annotation.normalized_height = TEXT_HEIGHT
        vis_objects.annotations.append(vis_annotation)

    if referee.current_action_time_remaining:
        vis_annotation = ShapeAnnotation()
        vis_annotation.text = parse_action_time_remaining(referee.current_action_time_remaining[0])
        vis_annotation.color.name = 'white'
        vis_annotation.normalized_x = TIMER_X
        vis_annotation.normalized_y = TEXT_HEIGHT
        vis_annotation.normalized_width = TIMER_WIDTH
        vis_annotation.normalized_height = TEXT_HEIGHT
        vis_objects.annotations.append(vis_annotation)

    # ロボット数
    if referee.blue.max_allowed_bots:
        vis_annotation = ShapeAnnotation()
        vis_annotation.color.name = COLOR_TEXT_BLUE
        # 許可台数よりロボットが多い場合は色を変える
        if blue_bots > referee.blue.max_allowed_bots[0]:
            vis_annotation.color.name = COLOR_TEXT_WARNING
        vis_annotation.text = 'BLUE BOTS: {}/{}'.format(
            blue_bots, referee.blue.max_allowed_bots[0])
        vis_annotation.normalized_x = BOTS_X
        vis_annotation.normalized_y = 0.0
        vis_annotation.normalized_width = BOTS_WIDTH
        vis_annotation.normalized_height = TEXT_HEIGHT
        vis_objects.annotations.append(vis_annotation)

    if referee.yellow.max_allowed_bots:
        vis_annotation = ShapeAnnotation()
        vis_annotation.color.name = COLOR_TEXT_YELLOW
        # 許可台数よりロボットが多い場合は色を変える
        if yellow_bots > referee.yellow.max_allowed_bots[0]:
            vis_annotation.color.name = COLOR_TEXT_WARNING
        vis_annotation.text = 'YELLOW BOTS: {}/{}'.format(
            yellow_bots, referee.yellow.max_allowed_bots[0])
        vis_annotation.normalized_x = BOTS_X
        vis_annotation.normalized_y = TEXT_HEIGHT
        vis_annotation.normalized_width = BOTS_WIDTH
        vis_annotation.normalized_height = TEXT_HEIGHT
        vis_objects.annotations.append(vis_annotation)

    # カード数
    vis_annotation = ShapeAnnotation()
    vis_annotation.color.name = COLOR_TEXT_BLUE
    vis_annotation.text = 'R: {}, Y:{}'.format(
        referee.blue.red_cards,
        referee.blue.yellow_cards)
    vis_annotation.normalized_x = CARDS_X
    vis_annotation.normalized_y = 0.0
    vis_annotation.normalized_width = CARDS_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    vis_annotation = ShapeAnnotation()
    vis_annotation.color.name = COLOR_TEXT_YELLOW
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
    vis_annotation.color.name = COLOR_TEXT_BLUE
    vis_annotation.text = parse_yellow_card_times(referee.blue.yellow_card_times)
    vis_annotation.normalized_x = YELLOW_CARD_TIMES_X
    vis_annotation.normalized_y = 0.0
    vis_annotation.normalized_width = YELLOW_CARD_TIMES_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    vis_annotation = ShapeAnnotation()
    vis_annotation.color.name = COLOR_TEXT_YELLOW
    vis_annotation.text = parse_yellow_card_times(referee.yellow.yellow_card_times)
    vis_annotation.normalized_x = YELLOW_CARD_TIMES_X
    vis_annotation.normalized_y = TEXT_HEIGHT
    vis_annotation.normalized_width = YELLOW_CARD_TIMES_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    # タイムアウト
    vis_annotation = ShapeAnnotation()
    vis_annotation.color.name = COLOR_TEXT_BLUE
    vis_annotation.text = parse_timeouts(referee.blue.timeouts, referee.blue.timeout_time)
    vis_annotation.normalized_x = TIMEOUT_X
    vis_annotation.normalized_y = 0.0
    vis_annotation.normalized_width = TIMEOUT_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    vis_annotation = ShapeAnnotation()
    vis_annotation.color.name = COLOR_TEXT_YELLOW
    vis_annotation.text = parse_timeouts(referee.yellow.timeouts, referee.yellow.timeout_time)
    vis_annotation.normalized_x = TIMEOUT_X
    vis_annotation.normalized_y = TEXT_HEIGHT
    vis_annotation.normalized_width = TIMEOUT_WIDTH
    vis_annotation.normalized_height = TEXT_HEIGHT
    vis_objects.annotations.append(vis_annotation)

    # プレースメント位置
    if referee.command == Referee.COMMAND_BALL_PLACEMENT_BLUE or \
       referee.command == Referee.COMMAND_BALL_PLACEMENT_YELLOW:
        vis_circle = ShapeCircle()
        vis_circle.center.x = placement_pos.x
        vis_circle.center.y = placement_pos.y
        vis_circle.radius = 0.15
        vis_circle.line_color.name = 'aquamarine'
        vis_circle.fill_color.name = 'aquamarine'
        vis_circle.line_size = 1
        vis_circle.caption = 'placement pos'
        vis_objects.circles.append(vis_circle)

    return vis_objects


def vis_prohibited_area(parsed_referee: ParsedReferee, ball_pos: Vector3):
    COLOR_LINE = 'black'
    COLOR_FILL = 'crimson'
    FILL_ALPHA = 0.3
    LINE_SIZE = 4
    vis_objects = Objects()
    vis_objects.layer = 'referee'
    vis_objects.sub_layer = 'prohibited_area'
    vis_objects.z_order = 1

    if parsed_referee.is_placement:
        # プレースメント時の禁止エリア
        # https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_placement_interference
        vis_tube = ShapeTube()
        vis_tube.p1.x = parsed_referee.designated_position.x
        vis_tube.p1.y = parsed_referee.designated_position.y
        vis_tube.p2.x = ball_pos.x
        vis_tube.p2.y = ball_pos.y
        vis_tube.radius = 0.5
        vis_tube.line_color.name = COLOR_LINE
        vis_tube.fill_color.name = COLOR_FILL
        vis_tube.fill_color.alpha = FILL_ALPHA
        vis_tube.line_size = LINE_SIZE
        vis_tube.caption = 'Rule 8.4.3'
        vis_objects.tubes.append(vis_tube)

    if not parsed_referee.is_inplay and \
       not parsed_referee.is_placement and \
       not parsed_referee.is_our_setplay:
        # ストップ中のボール周り
        vis_circle = ShapeCircle()
        vis_circle.center.x = ball_pos.x
        vis_circle.center.y = ball_pos.y
        vis_circle.radius = 0.5
        vis_circle.line_color.name = COLOR_LINE
        vis_circle.fill_color.name = COLOR_FILL
        vis_circle.fill_color.alpha = FILL_ALPHA
        vis_circle.line_size = LINE_SIZE
        vis_circle.caption = 'Rule 5.1.1'
        vis_objects.circles.append(vis_circle)

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


def parse_command(referee: Referee,
                  blue_color: str = 'blue', yellow_color: str = 'yellow') -> (str, str):
    # レフェリーコマンドを文字列と文字色に変換する
    output = "COMMAND"
    text_color = 'white'

    if len(referee.designated_position) > 0:
        placement_pos_x = referee.designated_position[0].x * 0.001
        placement_pos_y = referee.designated_position[0].y * 0.001

    ref_command = referee.command

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
        text_color = yellow_color
    elif ref_command == Referee.COMMAND_PREPARE_KICKOFF_BLUE:
        output = "PREPARE KICK OFF BLUE"
        text_color = blue_color
    elif ref_command == Referee.COMMAND_PREPARE_PENALTY_YELLOW:
        output = "PREPARE PENALTY YELLOW"
        text_color = yellow_color
    elif ref_command == Referee.COMMAND_PREPARE_PENALTY_BLUE:
        output = "PREPARE PENALTY BLUE"
        text_color = blue_color
    elif ref_command == Referee.COMMAND_DIRECT_FREE_YELLOW:
        output = "DIRECT FREE YELLOW"
        text_color = yellow_color
    elif ref_command == Referee.COMMAND_DIRECT_FREE_BLUE:
        output = "DIRECT FREE BLUE"
        text_color = blue_color
    elif ref_command == Referee.COMMAND_INDIRECT_FREE_YELLOW:
        output = "INDIRECT FREE YELLOW"
        text_color = yellow_color
    elif ref_command == Referee.COMMAND_INDIRECT_FREE_BLUE:
        output = "INDIRECT FREE BLUE"
        text_color = blue_color
    elif ref_command == Referee.COMMAND_TIMEOUT_YELLOW:
        output = "TIMEOUT YELLOW"
        text_color = yellow_color
    elif ref_command == Referee.COMMAND_TIMEOUT_BLUE:
        output = "TIMEOUT BLUE"
        text_color = blue_color
    elif ref_command == Referee.COMMAND_BALL_PLACEMENT_YELLOW:
        output = "BALL PLACEMENT YELLOW"
        output += "(x: {:.1f}, y: {:.1f})".format(placement_pos_x, placement_pos_y)
        text_color = yellow_color
    elif ref_command == Referee.COMMAND_BALL_PLACEMENT_BLUE:
        output = "BALL PLACEMENT BLUE"
        output += "(x: {:.1f}, y: {:.1f})".format(placement_pos_x, placement_pos_y)
        text_color = blue_color

    return (output, text_color)


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
