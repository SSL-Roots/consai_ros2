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
BallModel モジュール.

ロボカップSSLのフレーム情報からボールの状態を解析
位置・速度・次の予測位置・可視性を保持する BallModel クラスを定義
"""

from consai_msgs.msg import State2D

from robocup_ssl_msgs.msg import TrackedFrame


class BallModel:
    """ボールデータを保持するクラス."""

    def __init__(self):
        """Initialize the BallModel tactic."""
        self.pos = State2D()
        self.vel = State2D()
        self.is_visible = False
        self.visibility_threshold = 0.2

    def parse_frame(self, msg: TrackedFrame):
        """フレームメッセージからボール情報を抽出して内部状態を更新."""
        if len(msg.balls) == 0:
            return

        ball_frame = msg.balls[0]

        self.pos.x = ball_frame.pos.x
        self.pos.y = ball_frame.pos.y

        if ball_frame.vel:
            self.vel.x = ball_frame.vel[0].x
            self.vel.y = ball_frame.vel[0].y
        else:
            self.vel = State2D(x=0.0, y=0.0)
            self.next_pos = self.pos

        if ball_frame.visibility:
            self.is_visible = ball_frame.visibility[0] > self.visibility_threshold
        else:
            self.is_visible = False
