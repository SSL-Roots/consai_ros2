#!/usr/bin/env python3
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

"""レフェリー情報を解析するモジュール."""

import math

import consai_examples.referee_visualize_parser as ref_vis_parser

from consai_msgs.msg import ParsedReferee, State2D

from consai_visualizer_msgs.msg import Objects

from rclpy import qos
from rclpy.node import Node

from robocup_ssl_msgs.msg import Referee, RobotId, TrackedBall, TrackedFrame, Vector3


# refereeトピックを解読するノード
class RefereeParser(Node):
    """
    レフェリー情報を解析するノード.

    このノードは, 'referee'トピックを受信し, ゲームの状態やコマンドに基づいてレフェリー情報を解析し,
    解析結果を'parsed_referee'トピックに発行する. また, 'visualizer_objects'トピックを通じて, 可視化のための情報を提供する.

    Attributes:
        _our_team_is_yellow (bool): 自チームが黄色かどうか.
        _invert_placement_pos (bool): ボールプレースメントの座標を反転するかどうか.
        _division_a (bool): ゲームがDivision Aかどうか.
        _ball (TrackedBall): ボールの情報.
        _ball_pos_at_command_changing (Vector3): コマンドが変更された時のボールの位置.
        _stage (int): ゲームのステージ.
        _command_counter (int): コマンドカウンター.
        _current_command (int): 現在のコマンド.
        _prev_command (int): 前回のコマンド.
        _placement_pos (State2D): ボールプレースメントの目標座標.
        _max_allowed_our_bots (int): 許可されている自チームのロボット数.
        _num_of_blue_bots (int): 青チームのロボット数.
        _num_of_yellow_bots (int): 黄色チームのロボット数.
        _command_elapsed_time (float): コマンドの経過時間.
    """

    _STAGE_STR_DICT = {
        Referee.STAGE_NORMAL_FIRST_HALF_PRE: "NORMAL_FIRST_HALF_PRE",
        Referee.STAGE_NORMAL_FIRST_HALF: "NORMAL_FIRST_HALF",
        Referee.STAGE_NORMAL_HALF_TIME: "NORMAL_HALF_TIME",
        Referee.STAGE_NORMAL_SECOND_HALF_PRE: "NORMAL_SECOND_HALF_PRE",
        Referee.STAGE_NORMAL_SECOND_HALF: "NORMAL_SECOND_HALF",
        Referee.STAGE_EXTRA_TIME_BREAK: "EXTRA_TIME_BREAK",
        Referee.STAGE_EXTRA_FIRST_HALF_PRE: "EXTRA_FIRST_HALF_PRE",
        Referee.STAGE_EXTRA_FIRST_HALF: "EXTRA_FIRST_HALF",
        Referee.STAGE_EXTRA_HALF_TIME: "EXTRA_HALF_TIME",
        Referee.STAGE_EXTRA_SECOND_HALF_PRE: "EXTRA_SECOND_HALF_PRE",
        Referee.STAGE_EXTRA_SECOND_HALF: "EXTRA_SECOND_HALF",
        Referee.STAGE_PENALTY_SHOOTOUT_BREAK: "PENALTY_SHOOTOUT_BREAK",
        Referee.STAGE_PENALTY_SHOOTOUT: "PENALTY_SHOOTOUT",
        Referee.STAGE_POST_GAME: "POST_GAME",
    }

    _COMMAND_INPLAY = 99
    _COMMAND_OUR_PENALTY_INPLAY = 199
    _COMMAND_THEIR_PENALTY_INPLAY = 200

    def __init__(self, our_team_is_yellow=False, invert_placement_pos=False, division_a=True):
        """
        初期化関数.

        Args:
            our_team_is_yellow (bool): 自チームが黄色かどうかのフラグ.
            invert_placement_pos (bool): ボールの配置座標を反転させるかどうかのフラグ.
            division_a (bool): Division A の試合かどうかのフラグ.
        """
        super().__init__("referee_parser")

        self._our_team_is_yellow = our_team_is_yellow
        self._invert_placement_pos = invert_placement_pos
        self._division_a = division_a

        if self._our_team_is_yellow:
            self.get_logger().info("ourteamはyellowです")
        else:
            self.get_logger().info("ourteamはblueです")

        if self._invert_placement_pos:
            self.get_logger().info("ball placementの目標座標を反転します")

        self._COMMAND_OUR_PREPARE_KICKOFF = Referee.COMMAND_PREPARE_KICKOFF_BLUE
        self._COMMAND_THEIR_PREPARE_KICKOFF = Referee.COMMAND_PREPARE_KICKOFF_YELLOW
        self._COMMAND_OUR_PREPARE_PENALTY = Referee.COMMAND_PREPARE_PENALTY_BLUE
        self._COMMAND_THEIR_PREPARE_PENALTY = Referee.COMMAND_PREPARE_PENALTY_YELLOW
        self._COMMAND_OUR_DIRECT_FREE = Referee.COMMAND_DIRECT_FREE_BLUE
        self._COMMAND_THEIR_DIRECT_FREE = Referee.COMMAND_DIRECT_FREE_YELLOW
        self._COMMAND_OUR_INDIRECT_FREE = Referee.COMMAND_INDIRECT_FREE_BLUE
        self._COMMAND_THEIR_INDIRECT_FREE = Referee.COMMAND_INDIRECT_FREE_YELLOW
        self._COMMAND_OUR_TIMEOUT = Referee.COMMAND_TIMEOUT_BLUE
        self._COMMAND_THEIR_TIMEOUT = Referee.COMMAND_TIMEOUT_YELLOW
        self._COMMAND_OUR_BALL_PLACEMENT = Referee.COMMAND_BALL_PLACEMENT_BLUE
        self._COMMAND_THEIR_BALL_PLACEMENT = Referee.COMMAND_BALL_PLACEMENT_YELLOW

        if self._our_team_is_yellow:
            self._COMMAND_OUR_PREPARE_KICKOFF = Referee.COMMAND_PREPARE_KICKOFF_YELLOW
            self._COMMAND_THEIR_PREPARE_KICKOFF = Referee.COMMAND_PREPARE_KICKOFF_BLUE
            self._COMMAND_OUR_PREPARE_PENALTY = Referee.COMMAND_PREPARE_PENALTY_YELLOW
            self._COMMAND_THEIR_PREPARE_PENALTY = Referee.COMMAND_PREPARE_PENALTY_BLUE
            self._COMMAND_OUR_DIRECT_FREE = Referee.COMMAND_DIRECT_FREE_YELLOW
            self._COMMAND_THEIR_DIRECT_FREE = Referee.COMMAND_DIRECT_FREE_BLUE
            self._COMMAND_OUR_INDIRECT_FREE = Referee.COMMAND_INDIRECT_FREE_YELLOW
            self._COMMAND_THEIR_INDIRECT_FREE = Referee.COMMAND_INDIRECT_FREE_BLUE
            self._COMMAND_OUR_TIMEOUT = Referee.COMMAND_TIMEOUT_YELLOW
            self._COMMAND_THEIR_TIMEOUT = Referee.COMMAND_TIMEOUT_BLUE
            self._COMMAND_OUR_BALL_PLACEMENT = Referee.COMMAND_BALL_PLACEMENT_YELLOW
            self._COMMAND_THEIR_BALL_PLACEMENT = Referee.COMMAND_BALL_PLACEMENT_BLUE

        self._ball = TrackedBall()
        self._ball_pos_at_command_changing = Vector3()
        self._stage = -1
        self._command_counter = 0
        self._current_command = 0
        self._prev_command = 0
        self._placement_pos = State2D()
        self._max_allowed_our_bots = 0
        self._num_of_blue_bots: int = 0
        self._num_of_yellow_bots: int = 0
        self._command_elapsed_time = 0.0

        self._pub_parsed_referee = self.create_publisher(ParsedReferee, "parsed_referee", 10)
        self._pub_visualizer_objects = self.create_publisher(Objects, "visualizer_objects", qos.qos_profile_sensor_data)
        self._sub_detection_tracked = self.create_subscription(
            TrackedFrame, "detection_tracked", self._detection_tracked_callback, 10
        )
        self._sub_referee = self.create_subscription(Referee, "referee", self._referee_callback, 10)

    def _detection_tracked_callback(self, msg: TrackedFrame) -> None:
        """
        トラッキングデータを処理し、ボールの位置を更新する.

        Args:
            msg (TrackedFrame): トラッキングされたフレームのデータ.
        """
        if len(msg.balls) > 0:
            self._ball = msg.balls[0]

        self._update_num_of_bots(msg)

    def _referee_callback(self, msg):
        """
        レフェリーからのメッセージを受信し、状態を更新する.

        Args:
            msg (Referee): レフェリーからのメッセージ.
        """
        self._stage = msg.stage

        # コマンドのカウント値が変化したら、コマンドを更新する
        if msg.command_counter != self._command_counter:
            self._prev_command = self._current_command
            self._current_command = msg.command
            self._command_counter = msg.command_counter
            self._ball_pos_at_command_changing = self._ball.pos

        # in playの判定
        self._check_inplay(msg)

        # ボールプレースメントの目標座標
        if len(msg.designated_position) > 0:
            self._placement_pos.x = msg.designated_position[0].x * 0.001  # mm to meters
            self._placement_pos.y = msg.designated_position[0].y * 0.001  # mm to meters

            # フィールドサイドを反転しているときは、目標座標も反転させる
            if self._invert_placement_pos:
                self._placement_pos.x *= -1.0
                self._placement_pos.y *= -1.0

        # フィールドに出せるロボットの台数
        if self._our_team_is_yellow:
            if msg.yellow.max_allowed_bots:
                self._max_allowed_our_bots = msg.yellow.max_allowed_bots[0]
        else:
            if msg.blue.max_allowed_bots:
                self._max_allowed_our_bots = msg.blue.max_allowed_bots[0]

        # 解釈したレフェリー情報をpublishする
        parsed_ref_msg = self._bundle_parsed_referee()
        self._pub_parsed_referee.publish(parsed_ref_msg)

        # 可視化用のメッセージをpublishする
        self._publish_visualizer_objects(msg, parsed_ref_msg)

    def _bundle_parsed_referee(self) -> ParsedReferee:
        """
        解析したレフェリー情報をまとめる.

        Returns:
            ParsedReferee: 解析後のレフェリー情報.
        """
        referee = ParsedReferee()
        referee.designated_position.x = self._placement_pos.x
        referee.designated_position.y = self._placement_pos.y
        referee.is_placement = self.our_ball_placement() or self.their_ball_placement()
        referee.is_inplay = self.inplay() or self.our_penalty_inplay() or self.their_penalty_inplay()
        referee.is_our_setplay = (
            self.our_direct()
            or self.our_indirect()
            or self.our_kickoff()
            or self.our_penalty()
            or self.our_ball_placement()
            or self.our_pre_kickoff()
            or self.our_pre_penalty()
        )
        referee.is_their_setplay = (
            self.their_direct()
            or self.their_indirect()
            or self.their_kickoff()
            or self.their_penalty()
            or self.their_ball_placement()
            or self.their_pre_kickoff()
            or self.their_pre_penalty()
        )
        return referee

    def _publish_visualizer_objects(self, msg: Referee, parsed_ref_msg: ParsedReferee) -> None:
        """
        レフェリー情報を可視化するためのメッセージをpublishする.

        Args:
            msg (Referee): レフェリーからのメッセージ.
            parsed_ref_msg (ParsedReferee): 解析後のレフェリー情報.
        """
        self._pub_visualizer_objects.publish(
            ref_vis_parser.vis_info(msg, self._num_of_blue_bots, self._num_of_yellow_bots, self._placement_pos)
        )
        self._pub_visualizer_objects.publish(ref_vis_parser.vis_prohibited_area(parsed_ref_msg, self._ball.pos))

    def _update_num_of_bots(self, msg: TrackedFrame) -> None:
        """
        フィールド上のロボット台数を計上する.

        Args:
            msg (TrackedFrame): トラッキングされたフレームのデータ.
        """
        VISIBILITY_THRESHOLD = 0.01
        blue_robot_num = 0
        yellow_robot_num = 0
        for robot in msg.robots:
            if len(robot.visibility) <= 0:
                continue

            if robot.visibility[0] < VISIBILITY_THRESHOLD:
                continue

            if robot.robot_id.team_color == RobotId.TEAM_COLOR_YELLOW:
                yellow_robot_num += 1
            else:
                blue_robot_num += 1
        self._num_of_blue_bots = blue_robot_num
        self._num_of_yellow_bots = yellow_robot_num

    def _check_inplay(self, msg):
        """
        referee情報とフィールド情報をもとに、インプレイ状態を判定する.

        Args:
            msg (Referee): レフェリーからのメッセージ.
        """
        # インプレイ状態であれば、self._current_commandを上書きする

        # Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_in_and_out_of_play
        # force start has been issued.

        if self._current_command == Referee.COMMAND_FORCE_START:
            self.get_logger().info("force startによりinplayに変わります")
            self._current_command = self._COMMAND_INPLAY

        # the ball moved at least 0.05 meters following a kick-off, free kick or penalty kick.
        if (
            self.our_kickoff()
            or self.their_kickoff()
            or self.our_direct()
            or self.their_direct()
            or self.our_indirect()
            or self.their_indirect()
            or self.our_penalty()
            or self.their_penalty()
        ):

            diff_x = self._ball.pos.x - self._ball_pos_at_command_changing.x
            diff_y = self._ball.pos.y - self._ball_pos_at_command_changing.y

            if math.hypot(diff_x, diff_y) > 0.05:
                self.get_logger().info("ボールが0.05 meter動いたためinplayに変わります")
                # ペナルティキック時のインプレイではロボットが自由に動けないため、別のコマンドフラグを用意する
                if self.our_penalty():
                    self._current_command = self._COMMAND_OUR_PENALTY_INPLAY
                elif self.their_penalty():
                    self._current_command = self._COMMAND_THEIR_PENALTY_INPLAY
                else:
                    self._current_command = self._COMMAND_INPLAY

        # 10 seconds passed following a kick-off.
        elapsed_time = (msg.packet_timestamp - msg.command_timestamp) * 1e-6  # microseconds to seconds
        if self.our_kickoff() or self.their_kickoff():
            if elapsed_time > 10.0:
                self.get_logger().info("kickoffから10秒経過したのでinplayに変わります")
                self._current_command = self._COMMAND_INPLAY

        # 5 seconds (Division A) or 10 seconds (Division B) passed following a free kick.
        if self.our_direct() or self.our_indirect() or self.their_direct() or self.their_indirect():
            if self._division_a and elapsed_time > 5.0:
                self.get_logger().info("free kickから5秒経過したのでinplayに変わります")
                self._current_command = self._COMMAND_INPLAY
            elif not self._division_a and elapsed_time > 10.0:
                self.get_logger().info("free kickから10秒経過したのでinplayに変わります")
                self._current_command = self._COMMAND_INPLAY

        # コマンドの経過時間を格納
        self._command_elapsed_time = elapsed_time

    def present_stage(self):
        """
        ステージのテキストを返す.

        Returns:
            str: 現在のステージを表すテキスト.
        """
        return self._STAGE_STR_DICT.get(self._referee.stage, "NONE")

    def get_command(self):
        """
        レフェリーコマンドを返す.

        Returns:
            int: 現在のレフェリーコマンド.

        コマンドの更新を判定するために使用できる
        """
        return self._current_command

    def halt(self):
        """
        現在のコマンドがHALTであるかを判定する.

        Returns:
            bool: 現在のコマンドがHALTの場合はTrue、それ以外はFalse.
        """
        return self._current_command == Referee.COMMAND_HALT

    def stop(self):
        """
        現在のコマンドがSTOPであるかを判定する.

        Returns:
            bool: 現在のコマンドがSTOPの場合はTrue、それ以外はFalse.
        """
        return self._current_command == Referee.COMMAND_STOP

    def inplay(self):
        """
        現在のコマンドがINPLAYであるかを判定する.

        Returns:
            bool: 現在のコマンドがINPLAYの場合はTrue、それ以外はFalse.
        """
        return self._current_command == self._COMMAND_INPLAY

    def force_start(self):
        """
        現在のコマンドがFORCE_STARTであるかを判定する.

        Returns:
            bool: 現在のコマンドがFORCE_STARTの場合はTrue、それ以外はFalse.
        """
        return self._current_command == Referee.COMMAND_FORCE_START

    def our_penalty_inplay(self):
        """
        現在のコマンドが自チームのペナルティインプレイであるかを判定する.

        Returns:
            bool: 現在のコマンドが自チームのペナルティインプレイの場合はTrue、それ以外はFalse.
        """
        return self._current_command == self._COMMAND_OUR_PENALTY_INPLAY

    def their_penalty_inplay(self):
        """
        現在のコマンドが相手チームのペナルティインプレイであるかを判定する.

        Returns:
            bool: 現在のコマンドが相手チームのペナルティインプレイの場合はTrue、それ以外はFalse.
        """
        return self._current_command == self._COMMAND_THEIR_PENALTY_INPLAY

    def our_pre_kickoff(self):
        """
        現在のコマンドが自チームのプレキックオフであるかを判定する.

        Returns:
            bool: 現在のコマンドが自チームのプレキックオフの場合はTrue、それ以外はFalse.
        """
        return self._current_command == self._COMMAND_OUR_PREPARE_KICKOFF

    def our_kickoff(self):
        """
        現在のコマンドが自チームのキックオフであるかを判定する.

        Returns:
            bool: 現在のコマンドが自チームのキックオフの場合はTrue、それ以外はFalse.
        """
        return (
            self._prev_command == self._COMMAND_OUR_PREPARE_KICKOFF
            and self._current_command == Referee.COMMAND_NORMAL_START
        )

    def their_pre_kickoff(self):
        """
        現在のコマンドが相手チームのプレキックオフであるかを判定する.

        Returns:
            bool: 現在のコマンドが相手チームのプレキックオフの場合はTrue、それ以外はFalse.
        """
        return self._current_command == self._COMMAND_THEIR_PREPARE_KICKOFF

    def their_kickoff(self):
        """
        現在のコマンドが相手チームのキックオフであるかを判定する.

        Returns:
            bool: 現在のコマンドが相手チームのキックオフの場合はTrue、それ以外はFalse.
        """
        return (
            self._prev_command == self._COMMAND_THEIR_PREPARE_KICKOFF
            and self._current_command == Referee.COMMAND_NORMAL_START
        )

    def our_pre_penalty(self):
        """
        現在のコマンドが自チームのプレペナルティであるかを判定する.

        Returns:
            bool: 現在のコマンドが自チームのプレペナルティの場合はTrue、それ以外はFalse.
        """
        return self._current_command == self._COMMAND_OUR_PREPARE_PENALTY

    def our_penalty(self):
        """
        現在のコマンドが自チームのペナルティであるかを判定する.

        Returns:
            bool: 現在のコマンドが自チームのペナルティの場合はTrue、それ以外はFalse.
        """
        return (
            self._prev_command == self._COMMAND_OUR_PREPARE_PENALTY
            and self._current_command == Referee.COMMAND_NORMAL_START
        )

    def their_pre_penalty(self):
        """
        現在のコマンドが相手チームのプレペナルティであるかを判定する.

        Returns:
            bool: 現在のコマンドが相手チームのプレペナルティの場合はTrue、それ以外はFalse.
        """
        return self._current_command == self._COMMAND_THEIR_PREPARE_PENALTY

    def their_penalty(self):
        """
        現在のコマンドが相手チームのペナルティであるかを判定する.

        Returns:
            bool: 現在のコマンドが相手チームのペナルティの場合はTrue、それ以外はFalse.
        """
        return (
            self._prev_command == self._COMMAND_THEIR_PREPARE_PENALTY
            and self._current_command == Referee.COMMAND_NORMAL_START
        )

    def our_direct(self):
        """
        現在のコマンドが自チームの直接フリーキックであるかを判定する.

        Returns:
            bool: 現在のコマンドが自チームの直接フリーキックの場合はTrue、それ以外はFalse.
        """
        return self._current_command == self._COMMAND_OUR_DIRECT_FREE

    def their_direct(self):
        """
        現在のコマンドが相手チームの直接フリーキックであるかを判定する.

        Returns:
            bool: 現在のコマンドが相手チームの直接フリーキックの場合はTrue、それ以外はFalse.
        """
        return self._current_command == self._COMMAND_THEIR_DIRECT_FREE

    def our_indirect(self):
        """
        現在のコマンドが自チームの間接フリーキックであるかを判定する.

        Returns:
            bool: 現在のコマンドが自チームの間接フリーキックの場合はTrue、それ以外はFalse.
        """
        return self._current_command == self._COMMAND_OUR_INDIRECT_FREE

    def their_indirect(self):
        """
        現在のコマンドが相手チームの間接フリーキックであるかを判定する.

        Returns:
            bool: 現在のコマンドが相手チームの間接フリーキックの場合はTrue、それ以外はFalse.
        """
        return self._current_command == self._COMMAND_THEIR_INDIRECT_FREE

    def our_timeout(self):
        """
        現在のコマンドが自チームのタイムアウトであるかを判定する.

        Returns:
            bool: 現在のコマンドが自チームのタイムアウトの場合はTrue、それ以外はFalse.
        """
        return self._current_command == self._COMMAND_OUR_TIMEOUT

    def their_timeout(self):
        """
        現在のコマンドが相手チームのタイムアウトであるかを判定する.

        Returns:
            bool: 現在のコマンドが相手チームのタイムアウトの場合はTrue、それ以外はFalse.
        """
        return self._current_command == self._COMMAND_THEIR_TIMEOUT

    def our_ball_placement(self):
        """
        現在のコマンドが自チームのボールプレースメントであるかを判定する.

        Returns:
            bool: 現在のコマンドが自チームのボールプレースメントの場合はTrue、それ以外はFalse.
        """
        return self._current_command == self._COMMAND_OUR_BALL_PLACEMENT

    def their_ball_placement(self):
        """
        現在のコマンドが相手チームのボールプレースメントであるかを判定する.

        Returns:
            bool: 現在のコマンドが相手チームのボールプレースメントの場合はTrue、それ以外はFalse.
        """
        return self._current_command == self._COMMAND_THEIR_BALL_PLACEMENT

    def goal_blue(self):
        """
        現在のコマンドが青チームのゴールであるかを判定する.

        Returns:
            bool: 現在のコマンドが青チームのゴールの場合はTrue、それ以外はFalse.
        """
        return self._current_command == Referee.COMMAND_GOAL_BLUE

    def goal_yellow(self):
        """
        現在のコマンドが黄チームのゴールであるかを判定する.

        Returns:
            bool: 現在のコマンドが黄チームのゴールの場合はTrue、それ以外はFalse.
        """
        return self._current_command == Referee.COMMAND_GOAL_YELLOW

    def placement_position(self) -> State2D:
        """
        ボールのプレースメント位置を返す.

        Returns:
            State2D: ボールのプレースメント位置.
        """
        return self._placement_pos

    def max_allowed_our_bots(self):
        """
        自チームの最大許可ロボット台数を返す.

        Returns:
            int: 自チームの最大許可ロボット台数.
        """
        return self._max_allowed_our_bots

    def command_elapsed_time(self):
        """
        コマンドが発行されてからの経過時間を返す.

        Returns:
            float: 経過時間（秒単位）.
        """
        return self._command_elapsed_time
